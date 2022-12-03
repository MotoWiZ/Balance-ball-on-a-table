import time
import os
os.system("sudo pigpiod")
time.sleep(0.5)
import pigpio
pi = pigpio.pi()
servo1 = 12
servo2 = 13

pi.set_servo_pulsewidth(servo1, 670)

#pi.set_servo_pulsewidth(servo2, 1100)
time.sleep(3)
os.system("sudo killall pigpiod")

