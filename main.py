import time
import sys
from Adafruit_IO import Client,Feed, MQTCClient

ADAFRUIT_IO_USERNAME = 'San19231'
ADAFRUIT_IO_KEY = 'aio_BzEB66dFqevjFBNf2p1Ly87nTaTE'

io = Adafruit_IO.Client(ADAFRUIT_IO_USERNAME, ADAFRUIT_IO_KEY)  

# Inicializar el controlador de PCA9685
pwm = Adafruit_PCA9685.PCA9685()
pwm.set_pwm_freq(50)  # Establecer frecuencia de actualización de los servos

# Definir las claves de los feeds de Adafruit IO para cada servo
servo1_feed_key = 'servo1'
servo2_feed_key = 'servo2'
servo3_feed_key = 'servo3'
servo4_feed_key = 'servo4'

def set_servo_position(feed_key, position):
    """Establece la posición de un servo"""
    pwm_value = int(position * 4095)  # Convertir la posición a un valor PWM de 12 bits (0-4095)
    pwm.set_pwm(feed_key, 0, pwm_value)

while True:
    # Obtener el valor actual del feed de Adafruit IO para cada servo
    servo1_position = io.receive(servo1_feed_key).value
    servo2_position = io.receive(servo2_feed_key).value
    servo3_position = io.receive(servo3_feed_key).value
    servo4_position = io.receive(servo4_feed_key).value

    # Convertir los valores a números flotantes
    servo1_position = float(servo1_position)
    servo2_position = float(servo2_position)
    servo3_position = float(servo3_position)
    servo4_position = float(servo4_position)

    # Establecer la posición de cada servo
    set_servo_position(0, servo1_position)  # Canal 0 del controlador para el servo 1
    set_servo_position(1, servo2_position)  # Canal 1 del controlador para el servo 2
    set_servo_position(2, servo3_position)  # Canal 2 del controlador para el servo 3
    set_servo_position(3, servo4_position)  # Canal 3 del controlador para el servo 4

    time.sleep(0.1)  # Pequeña pausa antes de obtener nuevos valores del feed
