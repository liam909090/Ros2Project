#! /usr/bin/python3

# impoteerd modules
import time
import rclpy
import RPi.GPIO as GPIO
from rclpy.node import Node
from std_msgs.msg import String
from sensor_msgs.msg import Joy

pinTrigger = 17
pinEcho = 18
PinLight = 25
max_distance = 10

GPIO.setmode(GPIO.BCM)
GPIO.setwarnings(False)
GPIO.setup(pinTrigger, GPIO.OUT)
GPIO.setup(pinEcho, GPIO.IN)
GPIO.setup(PinLight, GPIO.IN)

class Sensor(Node):
    def __init__(self):
        super().__init__("_Sensor_")

        self._Sensor_subscription = self.create_subscription(
            String, "Sensor", self._Sensor_callback, 5
        )
    
    def _Sensor_callback(self, msg):
    
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