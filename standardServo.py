#Standard Servo
import time
import RPi.GPIO as GPIO

pin = 19

GPIO.setmode(GPIO.BCM)
GPIO.setup(pin, GPIO.OUT)
pwm = GPIO.PWM(pin, 100)
pwm.start( 7 )
#time.sleep( 5 )

#Servo Company: [range of motion] counterclockwise bound to clockwise bound
#EX1: [1-13]
#
lowerBound = 1
upperBound = 13


for pulse in range( 30,90, 5 ):
    pwm.ChangeFrequency( pulse)
    print( pulse )
    time.sleep( .5 )

pwm.stop()
GPIO.cleanup()

