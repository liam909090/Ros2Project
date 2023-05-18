#! /usr/bin/python3

import RPi.GPIO as GPIO
import time

pinTrigger = 17
pinEcho = 18

GPIO.setmode(GPIO.BCM)
GPIO.setwarnings(False)
GPIO.setup(pinTrigger, GPIO.OUT)
GPIO.setup(pinEcho, GPIO.IN)


class Motor:
    def __init__(self, pinFwd, pinBack, frequency=20, maxSpeed=100):
        #  Configure GPIO
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
        self._move(speed)

    def backwards(self, speed):
        self._move(-speed)

    def stop(self):
        self._move(0)

    def _move(self, speed):
        #  Set limits
        if speed > self._maxSpeed:
            speed = self._maxSpeed
        if speed < -self._maxSpeed:
            speed = -self._maxSpeed

        #  Turn on the motors
        if speed < 0:
            self._pwmFwd.ChangeDutyCycle(0)
            self._pwmBack.ChangeDutyCycle(-speed)
        else:
            self._pwmFwd.ChangeDutyCycle(speed)
            self._pwmBack.ChangeDutyCycle(0)


class Wheelie:
    def __init__(self):
        self.rightWheel = Motor(10, 9)
        self.leftWheel = Motor(8, 7)

    def stop(self):
        self.leftWheel.stop()
        self.rightWheel.stop()

    def goForward(self, speed=100):
        self.rightWheel.forwards(speed)
        self.leftWheel.forwards(speed)

    def goBackward(self, speed=100):
        self.rightWheel.backwards(speed)
        self.leftWheel.backwards(speed)

    def goLeft(self, speed=100):
        self.rightWheel.backwards(speed)
        self.leftWheel.forwards(speed)

    def goRight(self, speed=100):
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


def main():
    wheelie = Wheelie()
    x = 1
    try:
        while True:
            if distance() > 1:
                wheelie.goForward()
            if distance() < 1:
                wheelie.stop()
    finally:
        GPIO.cleanup()


if __name__ == "__main__":
    main()