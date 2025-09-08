
#include <stdio.h>
#include <assert.h>

#include <HD/hd.h>
#include <HDU/hduVector.h>
#include <HDU/hduError.h>

#include "ros/ros.h"
#include "std_msgs/String.h"
#include "std_msgs/Int32.h"
#include "std_msgs/Float64MultiArray.h"
#include "geometry_msgs/Point.h"
#include "geometry_msgs/Twist.h"
#include "geometry_msgs/Vector3.h"
#include "geometry_msgs/Pose.h"
#include "geometry_msgs/Wrench.h"

#define DEVICE_NAME_1  "Left Device"
#define DEVICE_NAME_2  "Right Device"

HHD hHD_1, hHD_2;
hduVector3Dd force_1, force_2;
hduVector3Dd position_1, position_2;
hduVector3Dd linear_velocity_1, linear_velocity_2;
hduVector3Dd angular_velocity_1, angular_velocity_2;
hduVector3Dd joint_angles_1, joint_angles_2;
hduVector3Dd gimbal_angles_1, gimbal_angles_2;
HDdouble transform_1[4][4], transform_2[4][4];
int buttons_1, buttons_2;

// NUEVAS VARIABLES para guardar la posición anterior
hduVector3Dd prev_position_1 = {0.0, 0.0, 0.0};
hduVector3Dd prev_position_2 = {0.0, 0.0, 0.0};

void chatterCallbackLeft(const geometry_msgs::Wrench::ConstPtr& data)
{
    float k_1 = 1;
    force_1[0] = data->force.x * k_1;
    force_1[1] = data->force.z * k_1;
    force_1[2] = -data->force.y * k_1;
}

void chatterCallbackRight(const geometry_msgs::Wrench::ConstPtr& data)
{
    float k_2 = 1;
    force_2[0] = data->force.x * k_2;
    force_2[1] = data->force.z * k_2;
    force_2[2] = -data->force.y * k_2;
}

HDCallbackCode HDCALLBACK Callback(void *data)
{
    hdBeginFrame(hHD_1);
    hdGetDoublev(HD_CURRENT_POSITION, position_1);
    hdGetDoublev(HD_CURRENT_VELOCITY, linear_velocity_1);
    hdGetDoublev(HD_CURRENT_ANGULAR_VELOCITY, angular_velocity_1);
    hdGetDoublev(HD_CURRENT_JOINT_ANGLES, joint_angles_1);
    hdGetDoublev(HD_CURRENT_GIMBAL_ANGLES, gimbal_angles_1);
    hdGetDoublev(HD_CURRENT_TRANSFORM, &transform_1[0][0]);
    hdGetIntegerv(HD_LAST_BUTTONS, &buttons_1);
    hdSetDoublev(HD_CURRENT_FORCE, force_1);
    hdEndFrame(hHD_1);

    hdBeginFrame(hHD_2);
    hdGetDoublev(HD_CURRENT_POSITION, position_2);
    hdGetDoublev(HD_CURRENT_VELOCITY, linear_velocity_2);
    hdGetDoublev(HD_CURRENT_ANGULAR_VELOCITY, angular_velocity_2);
    hdGetDoublev(HD_CURRENT_JOINT_ANGLES, joint_angles_2);
    hdGetDoublev(HD_CURRENT_GIMBAL_ANGLES, gimbal_angles_2);
    hdGetDoublev(HD_CURRENT_TRANSFORM, &transform_2[0][0]);
    hdGetIntegerv(HD_LAST_BUTTONS, &buttons_2);
    hdSetDoublev(HD_CURRENT_FORCE, force_2);
    hdEndFrame(hHD_2);

    HDErrorInfo error;
    if (HD_DEVICE_ERROR(error = hdGetError()))
    {
        hduPrintError(stderr, &error, "Error detected during main scheduler callback\n");
        if (hduIsSchedulerError(&error))
        {
            return HD_CALLBACK_DONE;
        }
    }

    return HD_CALLBACK_CONTINUE;
}

int main(int argc, char *argv[])
{
    HDErrorInfo error;

    hHD_1 = hdInitDevice(DEVICE_NAME_1);
    if (HD_DEVICE_ERROR(error = hdGetError()))
    {
        hduPrintError(stderr, &error, "Failed to initialize left haptic device");
        return -1;
    }

    hHD_2 = hdInitDevice(DEVICE_NAME_2);
    if (HD_DEVICE_ERROR(error = hdGetError()))
    {
        hduPrintError(stderr, &error, "Failed to initialize right haptic device");
        return -1;
    }

    hdEnable(HD_FORCE_OUTPUT);
    hdStartScheduler();
    if (HD_DEVICE_ERROR(error = hdGetError()))
    {
        hduPrintError(stderr, &error, "Failed to start the scheduler");
        return -1;
    }

    HDCallbackCode hCallback = hdScheduleAsynchronous(Callback, 0, HD_DEFAULT_SCHEDULER_PRIORITY);

    ros::init(argc, argv, "talker");
    ros::NodeHandle nh;

    ros::Subscriber sub_left = nh.subscribe("/forces_left", 1000, chatterCallbackLeft);
    ros::Subscriber sub_right = nh.subscribe("/forces_right", 1000, chatterCallbackRight);

    ros::Publisher pub_position_left = nh.advertise<geometry_msgs::Point>("position_left", 1000);
    ros::Publisher pub_position_right = nh.advertise<geometry_msgs::Point>("position_right", 1000);
    ros::Publisher pub_velocity_left = nh.advertise<geometry_msgs::Twist>("velocity_left", 1000);
    ros::Publisher pub_velocity_right = nh.advertise<geometry_msgs::Twist>("velocity_right", 1000);
    ros::Publisher pub_joint_angles_left = nh.advertise<geometry_msgs::Vector3>("joint_angles_left", 1000);
    ros::Publisher pub_joint_angles_right = nh.advertise<geometry_msgs::Vector3>("joint_angles_right", 1000);
    ros::Publisher pub_gimbal_angles_left = nh.advertise<geometry_msgs::Vector3>("gimbal_angles_left", 1000);
    ros::Publisher pub_gimbal_angles_right = nh.advertise<geometry_msgs::Vector3>("gimbal_angles_right", 1000);
    ros::Publisher pub_buttons_left = nh.advertise<std_msgs::Int32>("/buttons_left", 1000);
    ros::Publisher pub_buttons_right = nh.advertise<std_msgs::Int32>("/buttons_right", 1000);
    ros::Publisher pub_transform_left = nh.advertise<std_msgs::Float64MultiArray>("transform_left", 1000);
    ros::Publisher pub_transform_right = nh.advertise<std_msgs::Float64MultiArray>("transform_right", 1000);

    // NUEVOS PUBLISHERS PARA POSICIÓN RELATIVA
    ros::Publisher pub_relative_left = nh.advertise<geometry_msgs::Point>("relative_position_left", 1000);
    ros::Publisher pub_relative_right = nh.advertise<geometry_msgs::Point>("relative_position_right", 1000);

    ros::Rate loop_rate(10); // 10 Hz

    while (ros::ok())
    {
        // POSICIONES ABSOLUTAS
        geometry_msgs::Point msg_position_left, msg_position_right;
        msg_position_left.x = position_1[0];
        msg_position_left.y = position_1[1];
        msg_position_left.z = position_1[2];
        msg_position_right.x = position_2[0];
        msg_position_right.y = position_2[1];
        msg_position_right.z = position_2[2];

        pub_position_left.publish(msg_position_left);
        pub_position_right.publish(msg_position_right);

        // VELOCIDADES
        geometry_msgs::Twist msg_velocity_left, msg_velocity_right;
        msg_velocity_left.linear.x = linear_velocity_1[0];
        msg_velocity_left.linear.y = linear_velocity_1[1];
        msg_velocity_left.linear.z = linear_velocity_1[2];
        msg_velocity_left.angular.x = angular_velocity_1[0];
        msg_velocity_left.angular.y = angular_velocity_1[1];
        msg_velocity_left.angular.z = angular_velocity_1[2];

        msg_velocity_right.linear.x = linear_velocity_2[0];
        msg_velocity_right.linear.y = linear_velocity_2[1];
        msg_velocity_right.linear.z = linear_velocity_2[2];
        msg_velocity_right.angular.x = angular_velocity_2[0];
        msg_velocity_right.angular.y = angular_velocity_2[1];
        msg_velocity_right.angular.z = angular_velocity_2[2];

        pub_velocity_left.publish(msg_velocity_left);
        pub_velocity_right.publish(msg_velocity_right);

        // JOINT ANGLES
        geometry_msgs::Vector3 msg_joint_left, msg_joint_right;
        msg_joint_left.x = joint_angles_1[0];
        msg_joint_left.y = joint_angles_1[1];
        msg_joint_left.z = joint_angles_1[2];
        msg_joint_right.x = joint_angles_2[0];
        msg_joint_right.y = joint_angles_2[1];
        msg_joint_right.z = joint_angles_2[2];

        pub_joint_angles_left.publish(msg_joint_left);
        pub_joint_angles_right.publish(msg_joint_right);

        // GIMBAL ANGLES
        geometry_msgs::Vector3 msg_gimbal_left, msg_gimbal_right;
        msg_gimbal_left.x = gimbal_angles_1[0];
        msg_gimbal_left.y = gimbal_angles_1[1]; 
        msg_gimbal_left.z = gimbal_angles_1[2]; 

        msg_gimbal_right.x = gimbal_angles_2[2];
        msg_gimbal_right.y = gimbal_angles_2[0];
        msg_gimbal_right.z = gimbal_angles_2[1];

        pub_gimbal_angles_left.publish(msg_gimbal_left);
        pub_gimbal_angles_right.publish(msg_gimbal_right);

        // BUTTONS
        std_msgs::Int32 msg_buttons_left, msg_buttons_right;
        msg_buttons_left.data = buttons_1;
        msg_buttons_right.data = buttons_2;

        pub_buttons_left.publish(msg_buttons_left);
        pub_buttons_right.publish(msg_buttons_right);

        // MATRIZ DE TRANSFORMACIÓN
        std_msgs::Float64MultiArray msg_transform_left, msg_transform_right;
        msg_transform_left.data.clear();
        msg_transform_right.data.clear();

        for (int i = 0; i < 4; ++i)
        {
            for (int j = 0; j < 4; ++j)
            {
                msg_transform_left.data.push_back(transform_1[i][j]);
                msg_transform_right.data.push_back(transform_2[i][j]);
            }
        }

        pub_transform_left.publish(msg_transform_left);
        pub_transform_right.publish(msg_transform_right);

        // POSICIÓN RELATIVA (actual - anterior)
        geometry_msgs::Point msg_relative_left, msg_relative_right;

        msg_relative_left.x = position_1[0] - prev_position_1[0];
        msg_relative_left.y = position_1[1] - prev_position_1[1];
        msg_relative_left.z = position_1[2] - prev_position_1[2];

        msg_relative_right.x = position_2[0] - prev_position_2[0];
        msg_relative_right.y = position_2[1] - prev_position_2[1];
        msg_relative_right.z = position_2[2] - prev_position_2[2];

        pub_relative_left.publish(msg_relative_left);
        pub_relative_right.publish(msg_relative_right);

        // ACTUALIZAR posición anterior
        prev_position_1 = position_1;
        prev_position_2 = position_2;

        ros::spinOnce();
        loop_rate.sleep();
    }

    hdStopScheduler();
    hdUnschedule(hCallback);
    hdDisableDevice(hHD_1);
    hdDisableDevice(hHD_2);
}


