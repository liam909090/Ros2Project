#! /usr/bin/python3

#impoteerd modules
import time
import rclpy
from rclpy.node import Node
from std_msgs.msg import String
import RPi.GPIO as GPIO

pinTrigger = 17
pinEcho = 18
PinLight = 25

GPIO.setmode(GPIO.BCM)
GPIO.setwarnings(False)
GPIO.setup(pinTrigger, GPIO.OUT)
GPIO.setup(pinEcho, GPIO.IN)
GPIO.setup(PinLight, GPIO.IN)

# zet de node op voor ros2
class WallE(Node):
    def __init__(self):
        super().__init__('Wall_E')
        self.subscription = self.create_subscription(
            String,
            'Rijden',
            self.listener_callback,
            10
        )
        self.wheelie = Wheelie()

    # luisterd naar commands en onderneemd acties op basis van het command
    def listener_callback(self, msg):
        command = msg.data
        wheelie = Wheelie
        if command == 'forward' :
            if GPIO.input(PinLight) == 1:
                if distance() > 10:
                    self.wheelie.goForward()
                else:
                    self.wheelie.goRight()
        elif command == 'backwards' :
            self.wheelie.goBackward()
        elif command == 'stop' :
            self.wheelie.stop()
        elif command == "right" :
            self.wheelie.goRight()


class Motor:
    def __init__(self, pinFwd, pinBack, frequency=20, maxSpeed=100):
        #  Steld de GPIO in
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

# class om de wielen aan te sturen
class Wheelie:
    def __init__(self):
        self.rightWheel = Motor(10, 9)
        self.leftWheel = Motor(8, 7)

    def stop(self):
        self.leftWheel.stop()
        self.rightWheel.stop()

    def goForward(self, speed=40):
        self.rightWheel.forwards(speed)
        self.leftWheel.forwards(speed)

    def goBackward(self, speed=40):
        self.rightWheel.backwards(speed)
        self.leftWheel.backwards(speed)

    def goRight(self, speed=40):
        self.rightWheel.backwards(speed)
        self.leftWheel.forwards(speed)

    def goLeft(self, speed=40):
        self.rightWheel.forwards(speed)
        self.leftWheel.backwards(speed)


def distance():
    GPIO.output(pinTrigger, False)
    time.sleep(0.5)

    #  Send a 10us pulse
    print("send pulse")
    GPIO.output(pinTrigger, True)
    time.sleep(0.00001)
    GPIO.output(pinTrigger, False)

    #  Wait for echo to go high, then low
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
    rclpy.spin(WallE())

    WallE().wheelie.stop()
    GPIO.cleanup()
    WallE().destroy_node()
    rclpy.shutdown()

if __name__ == '__main__' :
    main()