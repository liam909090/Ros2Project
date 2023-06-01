#! /usr/bin/python3

# impoteerd modules
import time
import rclpy
import RPi.GPIO as GPIO
from rclpy.node import Node
from std_msgs.msg import String
from sensor_msgs.msg import Joy
from std_msgs.msg import Int16

pinTrigger = 17
pinEcho = 18
PinLight = 25
max_distance = 10

GPIO.setmode(GPIO.BCM)
GPIO.setwarnings(False)
GPIO.setup(pinTrigger, GPIO.OUT)
GPIO.setup(pinEcho, GPIO.IN)
GPIO.setup(PinLight, GPIO.IN)


# zet de node op voor ros2
class Wielen(Node):
    speed_int = 40  # variable om de snelheid makkelijk aan te passen

    def __init__(self):
        super().__init__("_Robot_")

        self.wheels = Wheels()

        self._joy_subscription = self.create_subscription(
            Joy, "joy", self._joy_callback, 5
        )
        self.subscription = self.create_subscription(
            Int16, "Afstand", self.Sonic_Sensor_callback, 10
        )

    def Sonic_Sensor_callback(self, msg):
        command = msg.data
        print(command)
        if int(command) < 10:
            print("stop")
            self.wheels.stop()

    def _joy_callback(self, msg):
        # to-do: uitzoeken welke knop wat is
        # Knop 0 = vierkant
        # Knop 1 = x
        # Knop 2 = o
        # Knop 3 = driehoek
        # Knop 4 = L1
        # Knop 5 = R1
        # Knop 6 = L2
        # Knop 7 = R2
        if msg.buttons[0]:  # stoppen
            self.wheels.stop()
        elif msg.axes[1] > 0.10:
            speed = msg.axes[1] * 100
            self.wheels.goForward(speed)
        elif msg.axes[1] < -0.10:
            speed = msg.axes[1] * -100
            self.wheels.goBackward(speed)
        elif msg.axes[0] > 0.10:
            speed = msg.axes[0] * 100
            self.wheels.goLeft(speed)
        elif msg.axes[0] < -0.10:
            speed = msg.axes[0] * -100
            self.wheels.goRight(speed)
        elif abs(msg.axes[0] < 0.10) & abs(msg.axes[1] < 0.10):
            self.wheels.stop()


# class om de motors aan te sturen
class Motor:
    def __init__(self, pinFwd, pinBack, frequency=20, maxSpeed=100):
        #  Stelt de GPIO in
        GPIO.setup(pinFwd, GPIO.OUT)
        GPIO.setup(pinBack, GPIO.OUT)

        #  Get a handle to PWM
        self._frequency = frequency
        self._maxSpeed = maxSpeed
        self._pwmFwd = GPIO.PWM(pinFwd, frequency)
        self._pwmBack = GPIO.PWM(pinBack, frequency)
        self._pwmFwd.start(0)
        self._pwmBack.start(0)

    def forwards(self, speed):
        self.move(speed)

    def backwards(self, speed):
        self.move(-speed)

    def stop(self):
        self.move(0)

    def move(self, speed):
        #  Zet snelheids limit in
        if speed > self._maxSpeed:
            speed = self._maxSpeed
        if speed < -self._maxSpeed:
            speed = -self._maxSpeed

        #  doet de moters aan
        if speed < 0:
            self._pwmFwd.ChangeDutyCycle(0)
            self._pwmBack.ChangeDutyCycle(-speed)
        else:
            self._pwmFwd.ChangeDutyCycle(speed)
            self._pwmBack.ChangeDutyCycle(0)


# class om de wielen te laten rijden
class Wheels:
    def __init__(self):
        self.rightWheel = Motor(10, 9)
        self.leftWheel = Motor(8, 7)

    def stop(self):
        self.leftWheel.stop()
        self.rightWheel.stop()

    def goForward(self, speed):
        self.rightWheel.forwards(speed)
        self.leftWheel.forwards(speed)

    def goBackward(self, speed):
        self.rightWheel.backwards(speed)
        self.leftWheel.backwards(speed)

    def goRight(self, speed):
        self.rightWheel.backwards(speed)
        self.leftWheel.forwards(speed)

    def goLeft(self, speed):
        self.rightWheel.forwards(speed)
        self.leftWheel.backwards(speed)


def main(args=None):
    rclpy.init(args=args)
    rclpy.spin(Wielen())
    Wielen().wheels.stop()
    Wielen().destroy_node()
    GPIO.cleanup()
    rclpy.shutdown()


if __name__ == "__main__":
    main()
