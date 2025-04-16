# Autonomous Puzzlebot

[![Ball Tracking Nodes](thumbnail.png)](https://youtu.be/6bQcEN82LVg)

Este proyecto implementa diferentes funcionalidades de control y navegación para el *Puzzlebot* utilizando ROS2 y Gazebo. Incluye control en lazo abierto y cerrado, seguimiento de rutas personalizadas, y un sistema avanzado de seguimiento de pelota basado en visión por computadora.

---

## Simulación en Gazebo

Para lanzar la simulación en Gazebo con el robot cargado:

```bash
ros2 launch puzzlebot_description gazebo.launch.py
```

---

## Movimiento del Robot

El robot se controla mediante el tópico `/cmd_vel`, que es de tipo:

```bash
geometry_msgs/msg/Twist
```

---

## Open Loop Controller

Hay dos formas principales de probar el movimiento del carrito en lazo abierto:

### Movimiento en Cuadrado

Corre el siguiente comando para que el robot se mueva automáticamente en forma de cuadrado:

```bash
ros2 run puzzlebot_control open_loop_control
```

### Ruta Personalizada

Puedes ejecutar un recorrido personalizado usando:

```bash
ros2 run puzzlebot_control route
```

Para modificar la ruta, edita el archivo `route.py` para establecer los puntos de navegación que prefieras.

---

## Ball Tracking

El sistema de seguimiento de pelota permite que el robot siga dinámicamente una pelota detectada por la cámara.

### Lanzamiento

Para iniciar todos los nodos relacionados con el seguimiento de pelota:

```bash
ros2 launch puzzlebot_control ball.launch.py
```

- `PID_Ball_Tracker`: Control PID para el seguimiento.
- `ball_test`: Genera movimiento senoidal de la pelota.
- `ball_tracker`: Publica la ubicación de la pelota utilizando la información de la cámara.

---
### Modelo de Control: Accionamiento Diferencial

El controlador del *Puzzlebot* se basa en el modelo de cinemática de un robot de accionamiento diferencial.

$$
\mathbf{v} = \frac{R}{2} (\omega_r + \omega_l)
$$

$$\omega = \frac{R}{L} (\omega_r - \omega_l)\$$

Estrategias de Control

    Basado en OpenCV (PID): Ajusta la velocidad angular según la posición de la pelota.

    Basado en Distancia: Ajusta la velocidad lineal para mantener una distancia deseada.
---

## Demo

Haz clic en la imagen del inicio para ver un demo en video del sistema de seguimiento en acción.

---


