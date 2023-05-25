#! /usr/bin/python3

# impoteerd modules
import time
import rclpy
from rclpy.node import Node
from std_msgs.msg import String
import RPi.GPIO as GPIO
from geometry_msgs.msg import Twist, Vector3
from sensor_msgs.msg import Joy, Range

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
        self.wheels = Wheels()  # Moet dit iets anders worden?

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


# node voor de sensor, luisterd of er een commando komt voor het aanpassen van de sensor
class Sensor(Node):
    def __init__(self):
        super().__init__("_sensor_")
        self.subscription = self.create_subscription(
            String, "SensorAfstand", self.listener_callback_sensor, 10
        )

    # luisterd naar commands en onderneemd acties op basis van het command
    def listener_callback_sensor(self, msg):
        command = msg.data
        if command == "distance1":
            max_distance += 1
        elif command == "distancemin1":
            max_distance -= 1
        elif command == "distance10":
            max_distance += 10
        elif command == "distancemin10":
            max_distance -= 10

    def _joy_callback(self, msg):
        """Translate XBox buttons into speed and spin

        Just use the left joystick (for now):
        LSB left/right  axes[0]     +1 (left) to -1 (right)
        LSB up/down     axes[1]     +1 (up) to -1 (back)
        LB              buttons[5]  1 pressed, 0 otherwise
        """

        if abs(msg.axes[0]) > 0.10:
            self.spin = msg.axes[0]
        else:
            self.spin = 0.0

        if abs(msg.axes[1]) > 0.10:
            self.speed = msg.axes[1]
        else:
            self.speed = 0.0

        if msg.buttons[5] == 1:
            self.speed = 0
            self.spin = 0

        Motor._set_motor_speeds()


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

    def _set_motor_speeds(self):
        # TODO: inject a stop() if no speeds seen for a while
        #
        # Scary math ahead.
        #
        # First figure out the speed of each wheel based on spin: each wheel
        # covers self._wheel_base meters in one radian, so the target speed
        # for each wheel in meters per sec is spin (radians/sec) times
        # wheel_base divided by wheel_diameter
        #
        right_twist_mps = self.spin * self._wheel_base / self._wheel_diameter
        left_twist_mps = -1.0 * self.spin * self._wheel_base / self._wheel_diameter
        #
        # Now add in forward motion.
        #
        left_mps = self.speed + left_twist_mps
        right_mps = self.speed + right_twist_mps
        #
        # Convert meters/sec into RPM: for each revolution, a wheel travels
        # pi * diameter meters, and each minute has 60 seconds.
        #
        left_target_rpm = (left_mps * 60.0) / (math.pi * self._wheel_diameter)
        right_target_rpm = (right_mps * 60.0) / (math.pi * self._wheel_diameter)
        #
        left_percentage = (left_target_rpm / self._left_max_rpm) * 100.0
        right_percentage = (right_target_rpm / self._right_max_rpm) * 100.0
        #
        # clip to +- 100%
        left_percentage = max(min(left_percentage, 100.0), -100.0)
        right_percentage = max(min(right_percentage, 100.0), -100.0)
        #
        # Add in a governor to cap forward motion when we're about
        # to collide with something (but still backwards motion)
        governor = 1.0
        if self.distance < self.tooclose:
            governor = 0.0
        elif self.distance < self.close:
            governor = (self.distance - self.tooclose) / (self.close - self.tooclose)
        if right_percentage > 0:
            right_percentage *= governor
        if left_percentage > 0:
            left_percentage *= governor
        #
        self._rightWheel.run(right_percentage)
        self._leftWheel.run(left_percentage)


# class om de wielen aan te sturen
class Wheels:
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
    Sensor().distance() == 10
    Sensor().destroy_node()
    GPIO.cleanup()
    rclpy.shutdown()


if __name__ == "__main__":
    main()
