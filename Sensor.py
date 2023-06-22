#! /usr/bin/python3

# impoteerd modules
import time
import rclpy
import RPi.GPIO as GPIO
from rclpy.node import Node
from std_msgs.msg import String
from std_msgs.msg import Int16

pinTrigger = 17
pinEcho = 18

GPIO.setmode(GPIO.BCM)
GPIO.setwarnings(False)
GPIO.setup(pinTrigger, GPIO.OUT)
GPIO.setup(pinEcho, GPIO.IN)


class Sonic_sensor(Node):
    # published de data van de afstand sensor naar de topic Afstand
    def __init__(self):
        super().__init__("sonic_sensor")
        self.publisher_ = self.create_publisher(Int16, "Afstand", 10)
        timer_period = 0.2  # seconden
        self.timer = self.create_timer(timer_period, self._Afstand_publisher)
        self.i = 0

    def _Afstand_publisher(self):
        msg = Int16()
        msg.data = int(Sonic_sensor.distance())
        self.publisher_.publish(msg)

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

        ElapsedTime = StopTime - StartTime
        distanceIPS = ElapsedTime * 13504 / 2
        print("%.1f in" % distanceIPS)
        return distanceIPS


def main(args=None):
    rclpy.init(args=args)
    publisher = Sonic_sensor()
    rclpy.spin(publisher)
    publisher.destroy_node()
    rclpy.shutdown()


if __name__ == "__main__":
    main()
