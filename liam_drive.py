#! /usr/bin/python3

# impoteerd modules
import time
import rclpy
from rclpy.node import Node
from std_msgs.msg import String
from sensor_msgs.msg import Joy
import RPi.GPIO as GPIO

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
    def __init__(self):
        super().__init__("_Wielen_")
        self.subscription = self.create_subscription(
            String, "Rijden", self.listener_callback_Wielen, 10
        )
        self.wheels = Wheels()

        self._joy_subscription = self.create_subscription(
            Joy, "joy", self._joy_callback, 5
        )

    # luisterd naar commands en onderneemd acties op basis van het command
    def listener_callback_Wielen(self, msg):
        command = msg.data
        wheels = Wheels
        if command == "forward":
            if GPIO.input(PinLight) == 1:
                if distance() > max_distance:
                    self.wheels.goForward()
                else:
                    self.wheels.goRight()
        elif command == "backwards":
            self.wheels.goBackward()
        elif command == "stop":
            self.wheels.stop()
        elif command == "right":
            self.wheels.goRight()
        elif command == "distance1":
            max_distance += 1
        elif command == "distancemin1":
            max_distance -= 1
        elif command == "distance10":
            max_distance += 10
        elif command == "distancemin10":
            max_distance -= 10

    def _joy_callback(self, msg):
        # to-do: uitzoeken welke knop wat is

        if msg.buttons[1] == 1:  # snelheid gaat naar beneden met knop 1
            self.wheels.speed_int = -10
        elif msg.buttons[2] == 1:  # snelheid gaat omhoog met knop 2
            self.wheels.speed_int = +10
        elif msg.buttons[3] == 1:  # gaat naar voren als knop 3 word ingedrukt
            if GPIO.input(PinLight) == 1:
                if distance() > max_distance:
                    self.wheels.goForward()
                else:
                    self.wheels.stop()
        elif msg.buttons[4] == 1:  # gaat naar achter waneer knop 4 word ingedrukt
            self.wheels.goBackward()
        elif msg.buttons[5] == 1:  # gaat naar links waneer knop 5 word ingedrukt
            self.wheels.goLeft()
        elif msg.buttons[6] == 1:  # gaat naar rechts waneer knop 6 word ingedrukt
            self.wheels.goRight()
        elif msg.buttons[7]:  # stopt met knop 7
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
    speed_int = 40  # variable om de snelheid makkelijk aan te passen

    def __init__(self):
        self.rightWheel = Motor(10, 9)
        self.leftWheel = Motor(8, 7)

    def stop(self):
        self.leftWheel.stop()
        self.rightWheel.stop()

    def goForward(self, speed=speed_int):
        self.rightWheel.forwards(speed)
        self.leftWheel.forwards(speed)

    def goBackward(self, speed=speed_int):
        self.rightWheel.backwards(speed)
        self.leftWheel.backwards(speed)

    def goRight(self, speed=speed_int):
        self.rightWheel.backwards(speed)
        self.leftWheel.forwards(speed)

    def goLeft(self, speed=speed_int):
        self.rightWheel.forwards(speed)
        self.leftWheel.backwards(speed)


def distance():
    GPIO.output(pinTrigger, False)
    time.sleep(0.5)

    #  Stuurt een pulse
    print("send pulse")
    GPIO.output(pinTrigger, True)
    time.sleep(0.00001)
    GPIO.output(pinTrigger, False)

    #  wacht tot de echo terug komt
    StartTime = time.time()
    while GPIO.input(pinEcho) == 0:
        StartTime = time.time()
    StopTime = time.time()
    while GPIO.input(pinEcho) == 1:
        StopTime = time.time()
        if StopTime - StartTime >= 0.04:
            StopTime = StartTime
            print("Too Close")
            break

    ElapsedTime = StopTime - StartTime
    distanceIPS = ElapsedTime * 13504 / 2
    print("%.1f in" % distanceIPS)
    return distanceIPS


def main(args=None):
    rclpy.init(args=args)
    rclpy.spin(Wielen())
    Wielen().wheels.stop()
    Wielen().destroy_node()
    GPIO.cleanup()
    rclpy.shutdown()


if __name__ == "__main__":
    main()
