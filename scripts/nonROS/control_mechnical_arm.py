import RPi.GPIO as GPIO
import time

GPIO.setmode(GPIO.BOARD)
GPIO.setwarnings(False)
GPIO.setup(35, GPIO.OUT)

op_d = 17
cl_d = 13
sp_d = 15
t = 1

pwm = GPIO.PWM(35, 100)
pwm.start(sp_d)
time.sleep(t)
pwm.ChangeDutyCycle(sp_d)
while True:
    cmd = input('Open or Close or Exit [o/c/e] ')
    if cmd == 'o':
        pwm.ChangeDutyCycle(op_d)
        time.sleep(t)
        pwm.ChangeDutyCycle(sp_d)
    elif cmd == 'c':
        pwm.ChangeDutyCycle(cl_d)
        #time.sleep(t)
        #pwm.ChangeDutyCycle(sp_d)
    elif cmd == 'e':
        break

pwm.stop()
GPIO.cleanup()

