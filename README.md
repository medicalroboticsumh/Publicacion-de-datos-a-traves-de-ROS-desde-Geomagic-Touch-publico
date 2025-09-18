# Publicación-de-datos-a-través-de-ROS-desde-Geomagic-Touch
## Descripción
Este repositorio contiene un paquete de **ROS** orientado a la publicación en tiempo real de los datos generados por el dispositivo háptico **Geomagic Touch** (anteriormente conocido como Phantom Omni). Para ello, se utilizan las bibliotecas **OpenHaptics**, que permiten capturar de manera continua la información del efector final del dispositivo, incluyendo su posición, orientación y el estado de los botones del stylus, entre otros parámetros.

El sistema está diseñado para organizar dichos datos en estructuras estandarizadas compatibles con ROS, facilitando su transmisión, visualización y posterior procesamiento dentro de aplicaciones de teleoperación. En este contexto, el nodo principal actúa como un talker, publicando de forma constante los datos en diferentes tópicos de ROS.

Una característica diferencial de este desarrollo es la posibilidad de trabajar con dos dispositivos Phantom de manera simultánea —uno ubicado a la izquierda del operador y otro a la derecha—, lo cual constituye la base para la futura teleoperación bimanual. De este modo, ambos dispositivos son gestionados desde un único nodo, que se encarga de capturar y publicar en paralelo la información procedente de cada uno en sus respectivos tópicos de ROS.

Gracias a esta infraestructura, el repositorio sienta las bases necesarias para la integración del Geomagic Touch en sistemas de teleoperación quirúrgica, proporcionando un entorno robusto, escalable y fácilmente extensible hacia implementaciones más complejas de control maestro-esclavo con retroalimentación háptica.

## Estructura del Proyecto  
El repositorio está organizado de la siguiente forma:  

- `CMakeLists.txt`: Archivo de configuración para la compilación del paquete con `catkin_make`.  
  En este archivo se incluyen los encabezados y las bibliotecas de OpenHaptics, necesarias para trabajar con el Geomagic Touch.  

- `/msg`: Carpeta que contiene definiciones de mensajes personalizados:  
  - `GimbalAngles.msg`: define los ángulos de tipo gimbal (roll, pitch, yaw).  
  - `Num.msg`: mensaje simple con un número entero.  

- `package.xml`: Archivo de metadatos del paquete, donde se especifican las dependencias requeridas (`rospy`, `roscpp`, `std_msgs`).  

- `/src`: Carpeta destinada al código fuente en C++.  
  Contiene la implementación del nodo ROS encargado de publicar la información del Geomagic Touch en diferentes tópicos.
  
# Requisitos Previos
Antes de ejecutar el sistema, asegúrese de cumplir con los siguientes requisitos:

1. **Conexión y encendido del dispositivo háptico Geomagic Touch.**  
   - El dispositivo debe estar correctamente conectado al equipo mediante USB y encendido.  

2. **Instalación del SDK OpenHaptics.**  
   - Es necesario disponer de las bibliotecas y cabeceras de OpenHaptics para acceder a las funciones de bajo nivel del Geomagic Touch.
   - Su descarga se realiza desde https://support.3dsystems.com/s/article/OpenHaptics-for-Linux-Developer-Edition-v34?language=en_US.  

3. **Instalación de drivers del Geomagic Touch en Linux.**  
   - Los drivers son imprescindibles para que el dispositivo sea reconocido y controlado correctamente por el sistema operativo.  
   - Deben instalarse antes de compilar o ejecutar cualquier nodo relacionado con el Phantom.
   - Su descarga se realiza desde https://support.3dsystems.com/s/article/OpenHaptics-for-Linux-Developer-Edition-v34?language=en_US.

4. **Entorno ROS instalado y configurado.**  
   - Se recomienda ROS Noetic (Ubuntu 20.04) o una distribución compatible.  

5. **Encender y configurar los equipos.**  
   - Equipo **'Master'**
     - Contraseña: nbio.
   - Equipo **'Gauss'**.
     - Contraseña: NBIO.

## Configuración de Red
Es necesario comprobar y establecer las siguientes direcciones IP:
- **Equipo 'Gauss'**: `192.168.0.115`
- **Equipo 'Master'**: `192.168.0.114` *(comprobar, esta IP puede variar)*

## Pasos de Ejecución
1. Iniciar `roscore` en el equipo **'Master'**:
   ```bash
   roscore
   ```
2. Lanzar el driver del Geomagic Touch:
   - Abrir un nuevo terminal en el equipo **'Gauss'**.
   - Acceder a la ruta `/home/mru/Desktop/DriverPhantom/TouchDriver_2024_09_19/bin`. *(corresponde a la ruta donde se encuentra ubicado el driver)*
     ```bash
     cd /home/mru/Desktop/DriverPhantom/TouchDriver_2024_09_19/bin
     ```
   - Dentro del mismo terminal, ejecutar el siguiente comando:
     ```bash
     ./Touch_AdvancedConfig
     ```
   - Se abrirá una ventana para la configuración del dispositivo.
     - En la pestaña `Settings`, puede asociarse un número de serie (desplegable `Serial Number`) con un nombre personalizado (campo `Device Name`) para cada dispositivo conectado.
     - La pestaña `Custom Calibration` permite definir un procedimiento de calibración personalizado, sustituyendo al proceso automático por defecto. *(Esta opción solo debe modificarse en caso de requerir una configuración especial, ya que un ajuste incorrecto puede comprometer la precisión y estabilidad del dispositivo)*.

3. Comprobar que la dirección IP del ROS Master esté correctamente especificada en la variable de entorno `ROS_MASTER_URI`, y que el nombre del host local coincida con el valor de `ROS_HOSTNAME`.
   Para ello, ejecute los siguientes comandos:
   ```bash
   echo $ROS_MASTER_URI
   echo $ROS_HOSTNAME
   ```
   En este caso, el primer comando debe dar como salida `http://192.168.0.114:11311`, que corresponde a la dirección IP del equipo **'Master'**; mientras que el segundo comando debe devolver `192.168.0.115`, que es la dirección IP del equipo local con el que se está trabajando (**'Gauss'**).

4. Abrir un nuevo terminal en el equipo **'Gauss'** y lanzar el nodo ROS para la publicación de la información del Geomagic Touch:
   ```bash
   cd catkin_ws
   source devel/setup.bash
   rosrun teleop_phantom talker_phantom
   ```
5. Para verificar que el nodo está funcionando correctamente y publicando datos en la red ROS, se puede visualizar la lista de topics activos.
   Para ello, abra un nuevo terminal en el equipo **'Gauss'** y ejecute el siguiente comando:
   ```bash
   rostopic list
   ```
6. Por último, puede comprobar que los datos se recogen correctamente en los topics visualizando la información en tiempo real de uno de ellos.
   Para ello, en el mismo terminal, imprima la información de uno de los topics que aparecieron en la lista anterior. Por ejemplo:
   ``bash
   rostopic echo /position_left
   ```
   Ahora puede realizar movimientos con el Geomagic Touch sobre sus 3 ejes y comprobar como sus valores varían en tiempo real.
   
   <p align="center">
      <img width="670" height="629" alt="Screenshot from 2025-09-18 10-42-08" src="https://github.com/user-attachments/assets/488b2ad1-f44a-4361-9a63-8dbf960bbb2e" />
   <p/>

