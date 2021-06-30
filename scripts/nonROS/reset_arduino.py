import RPi.GPIO as GPIO
import time

GPIO.setwarnings(False)
GPIO.setmode(GPIO.BOARD)
GPIO.setup(37, GPIO.OUT, initial=GPIO.HIGH)

GPIO.output(37, GPIO.LOW)
time.sleep(1)
GPIO.output(37, GPIO.HIGH)

GPIO.cleanup()
print('Bye')
