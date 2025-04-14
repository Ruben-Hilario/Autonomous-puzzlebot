# Autonomous Puzzlebot 🤖🧩

[![Ball Tracking Nodes](thumbnail.png)](https://youtu.be/6bQcEN82LVg)

Este proyecto implementa diferentes funcionalidades de control y navegación para el *Puzzlebot* utilizando ROS 2 y Gazebo. Incluye control en lazo abierto, seguimiento de rutas personalizadas, y un sistema avanzado de seguimiento de pelota basado en visión por computadora.

---

## 🚀 Simulación en Gazebo

Para lanzar la simulación en Gazebo con el robot cargado:

```bash
ros2 launch puzzlebot_description gazebo.launch.py
```

---

## 🕹️ Movimiento del Robot

El robot se controla mediante el tópico `/cmd_vel`, que es de tipo:

```bash
geometry_msgs/msg/Twist
```

---

## 🧭 Open Loop Controller

Hay dos formas principales de probar el movimiento del carrito en lazo abierto:

### 🟩 Movimiento en Cuadrado

Corre el siguiente comando para que el robot se mueva automáticamente en forma de cuadrado:

```bash
ros2 run puzzlebot_control open_loop_control
```

### ✏️ Ruta Personalizada

Puedes ejecutar un recorrido personalizado usando:

```bash
ros2 run puzzlebot_control route
```

Para modificar la ruta, edita el archivo `route.py`, específicamente la sección `waypoints`, donde puedes establecer los puntos de navegación que prefieras.

---

## 🎯 Ball Tracking

El sistema de seguimiento de pelota permite que el robot siga dinámicamente una pelota detectada por la cámara.

### 🔧 Lanzamiento

Para iniciar todos los nodos relacionados con el seguimiento de pelota:

```bash
ros2 launch puzzlebot_control ball.launch.py
```

Este comando lanza tres nodos clave:

- `PID_Ball_Tracker`: Implementa los controladores PID para el seguimiento.
- `ball_test`: Genera un movimiento senoidal de la pelota para pruebas.
- `ball_tracker`: Publica la ubicación de la pelota utilizando la información de la cámara.

---

### 🧠 Descripción de los Controladores

Se implementaron **dos controladores principales**:

1. **Controlador basado en OpenCV**  
   - Usa la imagen de la cámara para mantener la pelota centrada en el campo visual.  
   - A partir del error de posición de la pelota, se calculan las velocidades angulares necesarias para girar el robot.  
   - Se determinan los ángulos de giro de cada llanta para asegurar que el seguimiento sea suave y preciso.

2. **Controlador basado en distancia**  
   - Calcula la distancia entre el *Puzzlebot* y la pelota.  
   - Ajusta la velocidad lineal del robot: si la pelota está lejos, acelera; si está cerca, reduce la velocidad.  
   - Este comportamiento está parametrizado, permitiendo definir una distancia deseada (*setpoint*) entre el robot y la pelota.

---

## Demo

Haz clic en la imagen del inicio para ver un demo en video del sistema de seguimiento en acción.

---
