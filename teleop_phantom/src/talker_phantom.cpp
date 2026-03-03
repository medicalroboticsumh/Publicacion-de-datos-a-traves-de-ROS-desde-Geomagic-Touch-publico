#include <HD/hd.h>
#include <HDU/hduVector.h>
#include <HDU/hduError.h>

#include <ros/ros.h>
#include <std_msgs/Float32MultiArray.h>

#include <geometry_msgs/PointStamped.h>
#include <geometry_msgs/WrenchStamped.h>
#include <geometry_msgs/Vector3Stamped.h>
#include <geometry_msgs/TransformStamped.h>
#include <geometry_msgs/TwistStamped.h>


#include <std_msgs/Int32.h>

#include <iostream>

#include <tf2/LinearMath/Matrix3x3.h>
#include <tf2/LinearMath/Quaternion.h>

// ================= CONFIG =================
#define DEVICE_NAME "Left"
double k = 0.02;                 // Ganancia de fuerza
const int OFFSET_SAMPLES = 200;  // Número de muestras para promedio del offset
// =========================================

// ---------- Variables Phantom ----------
// son variables globales que en el callback haptico se actualizan 
HHD hHD;

hduVector3Dd position          = {0.0, 0.0, 0.0}; // mm
hduVector3Dd linear_velocity   = {0.0, 0.0, 0.0}; // mm/s
hduVector3Dd angular_velocity  = {0.0, 0.0, 0.0};  //rad/s
hduVector3Dd joint_angles      = {0.0, 0.0, 0.0};  // rad
hduVector3Dd gimbal_angles     = {0.0, 0.0, 0.0};  // rad
hduVector3Dd force_phantom_rtde     = {0.0, 0.0, 0.0};  //N
hduVector3Dd force_offset = {0.0, 0.0, 0.0};  // N·m
//hduVector3Dd torque_phantom = {0.0, 0.0, 0.0};// para iniciar en 0 con CURRENT (actualizacion)
//hduVector3Dd force_phantom = {0.0, 0.0, 0.0};// para iniciar en 0 con CURRENT (actualizacion)
hduVector3Dd torque_phantom = {0.0, 0.0, 0.0};
hduVector3Dd force_phantom = {0.0, 0.0, 0.0};  //si le doy valores con el set y el get se aplican (son valores fijos)

HDdouble transform[4][4];
int buttons = 0;

// Variables para calcular promedio del offset
bool offset_calculated = false;
int offset_count = 0;
hduVector3Dd offset_accum = {0.0, 0.0, 0.0};

// ---------------- Transformación SR Phantom-UR3e----------------
void robot2phantomTransform(const hduVector3Dd& f_robot, hduVector3Dd& f_phantom)
{
    // Rotaciones: Z -45°, X -90°
    double fx = f_robot[0];
    double fy = f_robot[1];
    double fz = f_robot[2];

    // Rotación Z -45°
    double fx1 = 0.7071 * fx + 0.7071 * fy;
    double fy1 = -0.7071 * fx + 0.7071 * fy;
    double fz1 = fz;

    // Rotación X -90°
    f_phantom[0] = fx1;
    f_phantom[1] = fz1;
    f_phantom[2] = -fy1;
}

// ---------------- Callback ROS ----------------


//para que esto funcione tiene que estar lanzado el archivo que lee los sensores del ur3e y lo que hace
// es ver la fuerza de compensacion del phantom segun la fuerza recibida del robot (no contempla torques)
void chatterCallback(const std_msgs::Float32MultiArray::ConstPtr& msg)
{
    if(msg->data.size() < 6) return; // Validar tamaño

    // Lectura de fuerzas TCP desde RTDE
    hduVector3Dd f_robot = {msg->data[0], msg->data[1], msg->data[2]};

    // Transformar al frame del Phantom
    hduVector3Dd f_phantom;
    robot2phantomTransform(f_robot, f_phantom);

    // ===================== Cálculo del offset =====================
    if (!offset_calculated)
    {
        offset_accum[0] += f_phantom[0];
        offset_accum[1] += f_phantom[1];
        offset_accum[2] += f_phantom[2];
        offset_count++;

        if (offset_count >= OFFSET_SAMPLES) //cuando llegua a 200 muestras, durante el calculo del offset no se aplica fuerza
        {
            force_offset[0] = offset_accum[0] / offset_count;
            force_offset[1] = offset_accum[1] / offset_count;
            force_offset[2] = offset_accum[2] / offset_count;
            offset_calculated = true;

            std::cout << "=== Offset calculado ===" << std::endl;
            std::cout << "Fuerza offset promedio: ["
                      << force_offset[0] << ", "
                      << force_offset[1] << ", "
                      << force_offset[2] << "]" << std::endl;
        }
        else
        {
            std::cout << "Calculando offset... (" << offset_count << "/" << OFFSET_SAMPLES << ")\r";
            std::cout.flush();
            return; // No aplicar fuerza mientras se calcula el offset
        }
    }
    // ===============================================================

    // Aplicar offset y ganancia
    force_phantom_rtde[0] = (f_phantom[0] - force_offset[0]) * k;
    force_phantom_rtde[1] = (f_phantom[1] - force_offset[1]) * k;
    force_phantom_rtde[2] = (f_phantom[2] - force_offset[2]) * k;

    // Debug
    //std::cout << "Fuerza Phantom (compensada): ["
        //      << force_phantom[0] << ", "
          //    << force_phantom[1] << ", "
            //  << force_phantom[2] << "]" << std::endl;
}

void forceSliderCallback(const geometry_msgs::WrenchStamped::ConstPtr& msg)
{
    force_phantom[0] = msg->wrench.force.x;
    force_phantom[1] = msg->wrench.force.y;
    force_phantom[2] = msg->wrench.force.z;

}


// ---------- Callback háptico ----------
//esto corre a alta frecuencia NO ES ROS, ES EL SCHEDULER DEL PHANTOM
HDCallbackCode HDCALLBACK hapticCallback(void* pUserData)
{
    hdBeginFrame(hHD);

    hdGetDoublev(HD_CURRENT_POSITION, position);
    hdGetDoublev(HD_CURRENT_VELOCITY, linear_velocity);
    hdGetDoublev(HD_CURRENT_ANGULAR_VELOCITY, angular_velocity);
    hdGetDoublev(HD_CURRENT_JOINT_ANGLES, joint_angles);
    hdGetDoublev(HD_CURRENT_GIMBAL_ANGLES, gimbal_angles);
    hdGetDoublev(HD_CURRENT_TRANSFORM, &transform[0][0]);
    hdGetIntegerv(HD_LAST_BUTTONS, &buttons);
    
    //fuerzas y torques 'reales' del phantom
    hdGetDoublev(HD_CURRENT_JOINT_TORQUE, torque_phantom);
    //hdGetDoublev(HD_CURRENT_TORQUE, torque_phantom); 
    //hdGetDoublev(HD_CURRENT_TORQUE_SENSOR, torque_phantom); -- para cuando actualices driver

    hdSetDoublev(HD_CURRENT_FORCE,force_phantom); // aplica la fuerza indicada arriba como variable 

    hdGetDoublev(HD_CURRENT_FORCE, force_phantom);//lee (y para publicar) la fuerza que tu les indiques en el hdSetDoublev
    //hdGetDoublev(HD_CURRENT_FORCE_SENSOR, force_phantom);

    hdEndFrame(hHD);

    HDErrorInfo error;
    if (HD_DEVICE_ERROR(error = hdGetError()))
        return HD_CALLBACK_DONE;

    return HD_CALLBACK_CONTINUE;
}

// -------------------- MAIN --------------------
int main(int argc, char** argv)
{
    ros::init(argc, argv, "talker_phantom"); //nodo que crea
    ros::NodeHandle nh;
    
    // Suscriptor: datos RTDE -- coge los datos que se recogen en /rtde_data, provienen del sensor de fuerzas del UR3e
    ros::Subscriber sub_rtde = nh.subscribe("/rtde_data", 1, chatterCallback);
    
    // Suscriptor para recoger la fuerza mandada desde slider en Simulink
    ros::Subscriber sub_force = nh.subscribe("/phantom_force", 1, forceSliderCallback);

    // ---------- Publishers (TODOS con header) ----------
        // Publicadores de estado
    
    // 1000 ES EL TAMANYO DE LA COLA (QUEUE SIZE) DEL PUBLICADOR 
    // ROS no envia el mensaje instantanemanete, los mensajes se meten en cola y si es suscriptor es mas lento que el publicador la cola se llena. Ros descarta los 	mensajes mas antiguos y mantiene los mas nuevos
    
    // el tamanyo debe depende de la frencuencia, y como publicamos a 10 Hz , puede que 1000 sea un buffer enorme !!!
    ros::Publisher pub_position =
        nh.advertise<geometry_msgs::PointStamped>("position_left", 100);

    ros::Publisher pub_velocity =
    	nh.advertise<geometry_msgs::TwistStamped>("velocity_left", 100);
    
    ros::Publisher pub_force =
        nh.advertise<geometry_msgs::WrenchStamped>("wrench_left", 100);

    ros::Publisher pub_joint =
        nh.advertise<geometry_msgs::Vector3Stamped>("joint_angles_left", 100);

    ros::Publisher pub_gimbal =
        nh.advertise<geometry_msgs::Vector3Stamped>("gimbal_angles_left", 100);

    ros::Publisher pub_buttons =
        nh.advertise<std_msgs::Int32>("buttons_left", 100);

    ros::Publisher pub_transform =
        nh.advertise<geometry_msgs::TransformStamped>("transform_left", 100);

    // ---------- Init Phantom ----------
    HDErrorInfo error;
    hHD = hdInitDevice(DEVICE_NAME);
    if (HD_DEVICE_ERROR(error = hdGetError()))
    {
        hduPrintError(stderr, &error, "hdInitDevice");
        return -1;
    }

    hdMakeCurrentDevice(hHD);
    hdEnable(HD_FORCE_OUTPUT);

    hdScheduleAsynchronous(hapticCallback, 0, HD_MAX_SCHEDULER_PRIORITY);
    hdStartScheduler();// ESTO ACTIVA EL LOOP DE FUERZA EN TIEMPO REAL

    ros::Rate rate(100); //LOOP PRINCIAL ROS (10 HZ) - moveL (solo 10)

    while (ros::ok())
    {
    	ros::Time now = ros::Time::now(); // captura el tiempo exacto de esta iteración

        // ---------- POSITION ----------
        geometry_msgs::PointStamped msg_pos;
        msg_pos.header.stamp = now; // asigna timestamp al mensaje       
        msg_pos.header.frame_id = "phantom_left";
        msg_pos.point.x = position[0];
        msg_pos.point.y = position[1];
        msg_pos.point.z = position[2];
        pub_position.publish(msg_pos);

        // ---------- VELOCITY ----------
        geometry_msgs::TwistStamped msg_vel;
        msg_vel.header.stamp = now;
        msg_vel.header.frame_id = "phantom_left";
	 // linear mm/s ; angular rad/s
        msg_vel.twist.linear.x  = linear_velocity[0];
        msg_vel.twist.linear.y  = linear_velocity[1];
        msg_vel.twist.linear.z  = linear_velocity[2];

        msg_vel.twist.angular.x = angular_velocity[0];
        msg_vel.twist.angular.y = angular_velocity[1];
        msg_vel.twist.angular.z = angular_velocity[2];

        pub_velocity.publish(msg_vel);
        
        // ---------- FORCE ----------
        geometry_msgs::WrenchStamped msg_force;
        msg_force.header.stamp = now;
        msg_force.header.frame_id = "phantom_left";

        msg_force.wrench.force.x  = force_phantom[0];
        msg_force.wrench.force.y  = force_phantom[1];
        msg_force.wrench.force.z  = force_phantom[2];

        msg_force.wrench.torque.x = torque_phantom[0];
        msg_force.wrench.torque.y = torque_phantom[1];
        msg_force.wrench.torque.z = torque_phantom[2];

        pub_force.publish(msg_force);

        // ---------- JOINT ANGLES ----------
        geometry_msgs::Vector3Stamped msg_joint;
        msg_joint.header.stamp = now;
        msg_joint.header.frame_id = "phantom_left";
        msg_joint.vector.x = joint_angles[0];
        msg_joint.vector.y = joint_angles[1];
        msg_joint.vector.z = joint_angles[2];
        pub_joint.publish(msg_joint);

        // ---------- GIMBAL ----------
        geometry_msgs::Vector3Stamped msg_gimbal;
        msg_gimbal.header.stamp = now;
        msg_gimbal.header.frame_id = "phantom_left";
        msg_gimbal.vector.x = gimbal_angles[0];
        msg_gimbal.vector.y = gimbal_angles[1];
        msg_gimbal.vector.z = gimbal_angles[2];
        pub_gimbal.publish(msg_gimbal);

        // ---------- BUTTONS ----------
        std_msgs::Int32 msg_buttons;
        msg_buttons.data = buttons;
        pub_buttons.publish(msg_buttons);

        // ---------- TRANSFORM ----------
        geometry_msgs::TransformStamped msg_tf;
        msg_tf.header.stamp = now;
        msg_tf.header.frame_id = "world";
        msg_tf.child_frame_id = "phantom_left";

        msg_tf.transform.translation.x = position[0]; //cogemos pos para que refleje el mov del phantom 
        msg_tf.transform.translation.y = position[1];
        msg_tf.transform.translation.z = position[2];

        tf2::Matrix3x3 R(
	    transform[0][0], transform[0][1], transform[0][2],
	    transform[1][0], transform[1][1], transform[1][2],
	    transform[2][0], transform[2][1], transform[2][2]
	);

	tf2::Quaternion q;
	R.getRotation(q);
	msg_tf.transform.rotation.x = q.x();
	msg_tf.transform.rotation.y = q.y();
	msg_tf.transform.rotation.z = q.z();
	msg_tf.transform.rotation.w = q.w();

        pub_transform.publish(msg_tf);

        ros::spinOnce();
        rate.sleep();
    }

    hdStopScheduler();
    hdDisableDevice(hHD);
    return 0;
}




