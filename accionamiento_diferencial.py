import math
import matplotlib.pyplot as plt
import matplotlib.animation as animation
from matplotlib.widgets import Slider, Button, TextBox
import matplotlib as mpl

# Configurar el estilo de matplotlib
plt.style.use('ggplot')
mpl.rcParams['font.size'] = 10
mpl.rcParams['axes.titlesize'] = 14
mpl.rcParams['axes.labelsize'] = 12

def calcular_velocidades(omega_der, omega_izq, radio_rueda, distancia_ruedas):
    """
    Calcula la velocidad lineal y la velocidad angular del robot.

    omega_der: velocidad angular de la rueda derecha (rad/s)
    omega_izq: velocidad angular de la rueda izquierda (rad/s)
    radio_rueda: radio de las ruedas (m)
    distancia_ruedas: distancia entre las ruedas (m)
    """
    # Convertir velocidades angulares a velocidades lineales
    v_der = radio_rueda * omega_der
    v_izq = radio_rueda * omega_izq
    
    # Velocidad lineal es el promedio de las velocidades de las ruedas
    v_lineal = (v_der + v_izq) / 2.0
    
    # Velocidad angular del robot
    v_angular = (v_der - v_izq) / distancia_ruedas
    return v_lineal, v_angular

def calcular_componentes(v_lineal, orientacion):
    """
    Calcula las componentes de la velocidad en el eje x y en el eje y.
    
    v_lineal: velocidad lineal del robot (m/s)
    orientacion: ángulo de orientación del robot (radianes)
    """
    v_x = v_lineal * math.cos(orientacion)
    v_y = v_lineal * math.sin(orientacion)
    return v_x, v_y

def actualizar_posicion(x, y, orientacion, v_x, v_y, v_angular, dt):
    """
    Actualiza la posición (x, y) y la orientación (theta) del robot.

    x, y: posición actual
    orientacion: ángulo actual (radianes)
    v_x, v_y: componentes de la velocidad en x e y (m/s)
    v_angular: velocidad angular (rad/s)
    dt: intervalo de tiempo (s)
    """
    x_new = x + v_x * dt
    y_new = y + v_y * dt
    orientacion_new = orientacion + v_angular * dt
    return x_new, y_new, orientacion_new

# Parámetros del robot y simulación
radio_rueda = 0.05         # en metros (por ejemplo, 5 cm)
distancia_ruedas = 0.2     # en metros (distancia entre ruedas, por ejemplo, 20 cm)
omega_der_base = 10.0      # velocidad angular base de la rueda derecha (rad/s)
omega_izq_base = 8.0       # velocidad angular base de la rueda izquierda (rad/s)
omega_der = omega_der_base # valor inicial
omega_izq = omega_izq_base # valor inicial

# Parámetros para la función seno
amplitud_der = 2.0         # amplitud de la oscilación para la rueda derecha
amplitud_izq = 2.0         # amplitud de la oscilación para la rueda izquierda
frecuencia = 0.5           # frecuencia de la oscilación (Hz)

# Parámetros para control hacia punto objetivo
objetivo_x = 1.5           # coordenada x del objetivo
objetivo_y = 1.0           # coordenada y del objetivo
k_p_lineal = 1.0           # ganancia proporcional para control de velocidad lineal
k_p_angular = 3.0          # ganancia proporcional para control de velocidad angular
velocidad_max = 2.0        # velocidad lineal máxima (m/s)
velocidad_angular_max = 2.0 # velocidad angular máxima (rad/s)
modo_control = "manual"    # "manual" o "punto_objetivo"

# Estado inicial
x = 0.0
y = 0.0
orientacion = 0.0          # en radianes

dt = 0.1                   # intervalo de tiempo (s)
tiempo_total = 10.0        # tiempo total de simulación (s)
frames = int(tiempo_total / dt)

# Listas para almacenar la trayectoria
trayectoria_x = [x]
trayectoria_y = [y]

# Configuración de la gráfica
fig, ax = plt.subplots(figsize=(12, 8))
plt.subplots_adjust(left=0.1, bottom=0.4, right=0.95, top=0.95)  # Hacer espacio para los controles
ax.set_xlim(-2, 2)
ax.set_ylim(-2, 2)
ax.set_aspect('equal')
ax.grid(True, linestyle='--', alpha=0.7)
ax.set_title("Simulación 2D de un Robot Diferencial", fontweight='bold', pad=10)
ax.set_xlabel("X (m)")
ax.set_ylabel("Y (m)")

# Crear un panel para los controles con un color de fondo
panel_rect = plt.Rectangle((0.05, 0.05), 0.9, 0.3, 
                        fill=True, color='#f0f0f0', 
                        transform=fig.transFigure, alpha=0.8,
                        zorder=-1)
fig.patches.extend([panel_rect])

# Línea que muestra la trayectoria
line, = ax.plot(trayectoria_x, trayectoria_y, 'b-', linewidth=2, label='Trayectoria')

# Flecha que representa la orientación actual del robot
# Se utiliza un quiver para facilitar su actualización
quiver = ax.quiver(x, y, math.cos(orientacion), math.sin(orientacion), 
                  angles='xy', scale_units='xy', scale=1, color='r', 
                  width=0.005, label='Orientación')

# Punto que representa el objetivo
punto_objetivo, = ax.plot(objetivo_x, objetivo_y, 'go', markersize=12, 
                         markeredgecolor='black', markeredgewidth=2, 
                         zorder=5, label='Objetivo')

# Punto que representa el robot
robot_punto, = ax.plot(x, y, 'ro', markersize=10, 
                      markeredgecolor='black', markeredgewidth=1, 
                      zorder=4, label='Robot')

# Área de recogida para el objetivo
area_objetivo = plt.Circle((objetivo_x, objetivo_y), 0.05, color='g', alpha=0.2)
ax.add_patch(area_objetivo)

ax.legend(loc='upper right', framealpha=0.9, frameon=True, shadow=True)

# Título de secciones
fig.text(0.1, 0.35, "CONTROL DE VELOCIDAD", fontsize=12, fontweight='bold')
fig.text(0.6, 0.35, "CONTROL DE OBJETIVO", fontsize=12, fontweight='bold')
fig.text(0.1, 0.16, "PARÁMETROS DE OSCILACIÓN", fontsize=12, fontweight='bold')
fig.text(0.6, 0.16, "PARÁMETROS DE NAVEGACIÓN", fontsize=12, fontweight='bold')

# Colores para los sliders y widgets
color_velocidad = '#c8deff'
color_objetivo = '#d8f8d8'
color_oscilacion = '#ffe8c8'
color_navegacion = '#f8d8f8'

# Añadir los sliders para controlar los parámetros en tiempo real
# Velocidades base
ax_omega_der = plt.axes([0.15, 0.30, 0.3, 0.03], facecolor=color_velocidad)
ax_omega_izq = plt.axes([0.15, 0.25, 0.3, 0.03], facecolor=color_velocidad)

# Parámetros de oscilación
ax_amplitud = plt.axes([0.15, 0.11, 0.3, 0.03], facecolor=color_oscilacion)
ax_frecuencia = plt.axes([0.15, 0.06, 0.3, 0.03], facecolor=color_oscilacion)

# Parámetros de navegación
ax_kp_lineal = plt.axes([0.65, 0.11, 0.3, 0.03], facecolor=color_navegacion)
ax_kp_angular = plt.axes([0.65, 0.06, 0.3, 0.03], facecolor=color_navegacion)

# Crear sliders con colores 
slider_omega_der = Slider(ax_omega_der, 'Omega Der Base', 0.0, 20.0, valinit=omega_der_base, 
                         color='#4080ff')
slider_omega_izq = Slider(ax_omega_izq, 'Omega Izq Base', 0.0, 20.0, valinit=omega_izq_base, 
                         color='#4080ff')
slider_amplitud = Slider(ax_amplitud, 'Amplitud', 0.0, 5.0, valinit=amplitud_der, 
                        color='#ff8040')
slider_frecuencia = Slider(ax_frecuencia, 'Frecuencia', 0.1, 2.0, valinit=frecuencia, 
                          color='#ff8040')
slider_kp_lineal = Slider(ax_kp_lineal, 'Kp Lineal', 0.1, 5.0, valinit=k_p_lineal, 
                         color='#8040ff')
slider_kp_angular = Slider(ax_kp_angular, 'Kp Angular', 0.1, 10.0, valinit=k_p_angular, 
                          color='#8040ff')

# TextBox para introducir las coordenadas del objetivo
ax_objetivo_x = plt.axes([0.65, 0.30, 0.12, 0.03], facecolor=color_objetivo)
ax_objetivo_y = plt.axes([0.65, 0.25, 0.12, 0.03], facecolor=color_objetivo)
text_box_x = TextBox(ax_objetivo_x, 'Objetivo X', initial=str(objetivo_x))
text_box_y = TextBox(ax_objetivo_y, 'Objetivo Y', initial=str(objetivo_y))

# Botones con colores mejorados
ax_reset = plt.axes([0.1, 0.36, 0.15, 0.04])
ax_modo = plt.axes([0.3, 0.36, 0.15, 0.04])
ax_set_punto = plt.axes([0.83, 0.28, 0.12, 0.03])

boton_reset = Button(ax_reset, 'Reiniciar', color='#ffcccb', hovercolor='#ff9999')
boton_modo = Button(ax_modo, 'Modo: Manual', color='#ccffcc', hovercolor='#99ff99')
boton_set_punto = Button(ax_set_punto, 'Establecer', color='#ccffcc', hovercolor='#99ff99')

def calcular_omegas_para_punto_objetivo(x_actual, y_actual, orientacion_actual, x_objetivo, y_objetivo):
    """
    Calcula las velocidades angulares necesarias para dirigir el robot hacia un punto objetivo.
    
    Utiliza un control proporcional para ajustar la velocidad lineal y angular.
    """
    # Calcular la distancia al objetivo
    dx = x_objetivo - x_actual
    dy = y_objetivo - y_actual
    distancia = math.sqrt(dx*dx + dy*dy)
    
    # Calcular el ángulo hacia el objetivo
    angulo_objetivo = math.atan2(dy, dx)
    
    # Calcular el error angular (diferencia entre orientación actual y ángulo hacia el objetivo)
    error_angular = angulo_objetivo - orientacion_actual
    # Normalizar el error angular entre -pi y pi
    error_angular = math.atan2(math.sin(error_angular), math.cos(error_angular))
    
    # Control proporcional para velocidad lineal y angular
    k_p_lineal_actual = slider_kp_lineal.val
    k_p_angular_actual = slider_kp_angular.val
    
    # Velocidad lineal proporcional a la distancia (con un máximo)
    v_lineal = min(k_p_lineal_actual * distancia, velocidad_max)
    
    # Velocidad angular proporcional al error angular (con un máximo)
    v_angular = min(max(k_p_angular_actual * error_angular, -velocidad_angular_max), velocidad_angular_max)
    
    # Convertir velocidades lineal y angular a velocidades de ruedas
    # v_lineal = (v_der + v_izq) / 2
    # v_angular = (v_der - v_izq) / distancia_ruedas
    # Por lo tanto:
    v_der = v_lineal + (v_angular * distancia_ruedas / 2)
    v_izq = v_lineal - (v_angular * distancia_ruedas / 2)
    
    # Convertir velocidades lineales de ruedas a velocidades angulares
    omega_der_objetivo = v_der / radio_rueda
    omega_izq_objetivo = v_izq / radio_rueda
    
    return omega_der_objetivo, omega_izq_objetivo, distancia

def actualizar_omegas(tiempo):
    """
    Actualiza las velocidades angulares de las ruedas según el modo de operación.
    
    tiempo: tiempo actual de la simulación (s)
    """
    global omega_der, omega_izq, objetivo_x, objetivo_y
    
    if modo_control == "manual":
        # Modo manual: usar la función seno para variar las velocidades
        omega_der_base_actual = slider_omega_der.val
        omega_izq_base_actual = slider_omega_izq.val
        amplitud_actual = slider_amplitud.val
        frecuencia_actual = slider_frecuencia.val
        
        omega_der = omega_der_base_actual + amplitud_actual * math.sin(2 * math.pi * frecuencia_actual * tiempo)
        omega_izq = omega_izq_base_actual + amplitud_actual * math.sin(2 * math.pi * frecuencia_actual * tiempo)
    else:
        # Modo punto objetivo: calcular velocidades para dirigirse al objetivo
        omega_der, omega_izq, distancia = calcular_omegas_para_punto_objetivo(
            x, y, orientacion, objetivo_x, objetivo_y
        )
        
        # Si estamos muy cerca del objetivo, detenemos el robot
        if distancia < 0.05:  # 5 cm de tolerancia
            omega_der = 0
            omega_izq = 0
    
    return omega_der, omega_izq

def actualizar_objetivo(event):
    global objetivo_x, objetivo_y
    try:
        objetivo_x = float(text_box_x.text)
        objetivo_y = float(text_box_y.text)
        punto_objetivo.set_data([objetivo_x], [objetivo_y])
        # Actualizar el área de recogida
        area_objetivo.center = (objetivo_x, objetivo_y)
        # Asegurar que el punto se dibuje por encima de otras capas
        punto_objetivo.set_zorder(5)  
        # Forzar la actualización del lienzo
        plt.draw()
        fig.canvas.flush_events()
    except ValueError:
        pass  # Ignorar si no es un número válido

def cambiar_modo(event):
    global modo_control
    if modo_control == "manual":
        modo_control = "punto_objetivo"
        boton_modo.label.set_text("Modo: Objetivo")
    else:
        modo_control = "manual"
        boton_modo.label.set_text("Modo: Manual")
    fig.canvas.draw_idle()

def reset_trayectoria(event):
    global x, y, orientacion, trayectoria_x, trayectoria_y
    x = 0.0
    y = 0.0
    orientacion = 0.0
    trayectoria_x = [x]
    trayectoria_y = [y]
    line.set_data(trayectoria_x, trayectoria_y)
    quiver.set_offsets([x, y])
    quiver.set_UVC(math.cos(orientacion), math.sin(orientacion))
    fig.canvas.draw_idle()

# Conectar los botones y textbox a los eventos
boton_reset.on_clicked(reset_trayectoria)
boton_modo.on_clicked(cambiar_modo)
boton_set_punto.on_clicked(actualizar_objetivo)
text_box_x.on_submit(actualizar_objetivo)
text_box_y.on_submit(actualizar_objetivo)

def update(frame):
    global x, y, orientacion, trayectoria_x, trayectoria_y
    
    # Tiempo actual
    tiempo_actual = frame * dt
    
    # Actualización de las velocidades angulares según el modo
    actualizar_omegas(tiempo_actual)
    
    # Cálculo de velocidades
    v_lineal, v_angular = calcular_velocidades(omega_der, omega_izq, radio_rueda, distancia_ruedas)
    v_x, v_y = calcular_componentes(v_lineal, orientacion)
    
    # Actualización de la posición y orientación
    x, y, orientacion = actualizar_posicion(x, y, orientacion, v_x, v_y, v_angular, dt)
    
    # Almacenar la nueva posición
    trayectoria_x.append(x)
    trayectoria_y.append(y)
    
    # Actualizar la línea de la trayectoria
    line.set_data(trayectoria_x, trayectoria_y)
    
    # Actualizar la flecha (quiver) de orientación
    quiver.set_offsets([x, y])
    quiver.set_UVC(math.cos(orientacion), math.sin(orientacion))
    
    # Actualizar posición del punto del robot
    robot_punto.set_data([x], [y])
    
    # Asegurar que el punto objetivo se mantenga visible
    punto_objetivo.set_data([objetivo_x], [objetivo_y])
    
    return line, quiver, punto_objetivo, robot_punto

# Creación de la animación
ani = animation.FuncAnimation(fig, update, frames=None, blit=True, interval=dt*1000, repeat=True)

plt.show()
