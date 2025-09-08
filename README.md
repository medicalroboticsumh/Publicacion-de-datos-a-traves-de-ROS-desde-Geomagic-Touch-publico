# Publicación-de-datos-a-través-de-ROS-desde-Geomagic-Touch
## Descripción
Este repositorio contiene un paquete de ROS orientado a la publicación en tiempo real de los datos generados por el dispositivo háptico Geomagic Touch (anteriormente conocido como Phantom Omni). Para ello, se utilizan las bibliotecas OpenHaptics, que permiten capturar de manera continua la información del efector final del dispositivo, incluyendo su posición, orientación y el estado de los botones del stylus, entre otros parámetros.

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
- Conectar 
- Encender y configurar los siguientes equipos:
  - Equipo 'Gauss'
  - Equipo 'Master'
