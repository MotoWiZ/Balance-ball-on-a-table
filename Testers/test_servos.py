
import RPi.GPIO as GPIO
import time

SERVO_PIN1 = 32
SERVO_PIN2 = 33

if __name__ == '__main__':
	# -> Setting the GPIO Mode to BOARD
	GPIO.setmode(GPIO.BOARD)

	GPIO.setup(SERVO_PIN1,GPIO.OUT)
	GPIO.setup(SERVO_PIN2,GPIO.OUT)

	# Disable the warning from the GPIO Library
	GPIO.setwarnings(False)

	# Starting the PWM and setting the initial position of the servo with 50Hz frequency 
	servo1 = GPIO.PWM(SERVO_PIN1,50)
	servo1.start(0)
	servo2 = GPIO.PWM(SERVO_PIN2,50)
	servo2.start(0)
	while True:
		try:
			# Changing the Duty Cycle to rotate the motor 
			"""
					NOTE: In our case the servos should only move to PWM:
					SERVO 1: MAX -> 5.5 min -> 3.3
					SERVO 2: MAX -> 7.5 min -> 5
			"""
			servo1.ChangeDutyCycle(5.5)
			#time.sleep(2)
			servo2.ChangeDutyCycle(7.5)
			time.sleep(0.5)
			servo1.ChangeDutyCycle(3.3)
			#time.sleep(2)
			servo2.ChangeDutyCycle(5)
			time.sleep(0.5)

		except KeyboardInterrupt:
			servo1.stop()
			servo2.stop()
			GPIO.cleanup()
			quit()
# End of the Script
