
#Continuous Servo
import time
import RPi.GPIO as GPIO

pin = 13

GPIO.setmode(GPIO.BCM)
GPIO.setup(pin, GPIO.OUT)
pwm = GPIO.PWM(pin, 100)
pwm.start( 6 )
#Servo Company: [levels of speed] direction
#Parallax/Futaba Continuous: [6] clockwise, [7] zero, [8] counterclockwise
#Spring RC: [5,6] counterclockwise, [7] zero, [8,9] clockwise
for pulse in range( 30,70, 1 ):
    pwm.ChangeFrequency( pulse )
    print pulse
    time.sleep( .2 );

pwm.stop()
GPIO.cleanup()

