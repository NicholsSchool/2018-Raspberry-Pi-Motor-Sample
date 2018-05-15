from Adafruit_MotorHAT import Adafruit_MotorHAT, Adafruit_DCMotor
import atexit
import Adafruit_MCP3008
import time
import RPi.GPIO as GPIO
import thread
SCALE = 100
DEADZONE = { "A" : 20,
             "B" : 5,
             "C" : 50 }
ICPIN = { "A" : 0,
          "B" : 1,
          "C" : 2 }
MIN = { "A" : 0,
        "B" : 0,
        "C" : 50
      }
MAX = { "A" : 1023,
        "B" : 70,
        "C" : 650
      }

global control
control = ""
global motor
motor = ""
global terminate
terminate = False
#####################################################################
#Board Control
def boardControl():

        KEYPAD =  [
                  ["1","2","3","A"],
                  ["4","5","6","B"],
                  ["7","8","9","C"],
                  ["*","0","#","D"]
                  ]

        ROW = [21,20,16,12]
        COL = [7,8,25,24]
        global control
        global motor
        global terminate
        for j in range(4):
            GPIO.setup( COL[j], GPIO.OUT )
            GPIO.output( COL[j], 1 )

         
        for i in range(4):
            GPIO.setup( ROW[i], GPIO.IN, pull_up_down = GPIO.PUD_UP )
        try:
            
            while(True):
                for j in range(4):
                    GPIO.output(COL[j],0)
                    for i in range(4):    
                        if GPIO.input( ROW[i] ) == 0:
                            if KEYPAD[i][j]=="A" or KEYPAD[i][j]=="B" or KEYPAD[i][j]=="C" or KEYPAD[i][j]=="D":
                                control = KEYPAD[i][j]
                            elif control != "" and (KEYPAD[i][j]=="1" or KEYPAD[i][j]=="2" or KEYPAD[i][j]=="3" or KEYPAD[i][j]=="4" or KEYPAD[i][j]=="5" or KEYPAD[i][j]=="6"):
                                motor = KEYPAD[i][j]
                                if( motor != "1" and motor != "2" ):
                                    thread.start_new_thread( controlServo, (motor,) )
                                else:
                                    try:
                                        global pwm
                                        pwm.stop()
                                    except NameError:
                                        pass
                                    thread.start_new_thread( controlMotor, (motor,) )
                            elif KEYPAD[i][j]=="*":
                                terminate = True
                                return;
                            while( GPIO.input( ROW[i] ) == 0):
                                pass
                    GPIO.output(COL[j],1)
                time.sleep(.2)
        except KeyboardInterrupt:
            GPIO.cleanup()

######################################################################
#Analogue Joystick [1,223,1023]
#Flex Sensor [14,44,61]
#Haptic Servo/Potentiometer [62,630]
def controlServo(xmotor):
        global control
        global motor
        xcontrol = control
        med = mcp.read_adc(ICPIN[xcontrol])

        GPIO.setup(PIN[xmotor], GPIO.OUT)
        global pwm
        pwm = GPIO.PWM(PIN[xmotor], 100)
        pwm.start( 6 )
        try:
                while True:
                    if control != xcontrol or motor != xmotor or terminate:
                        return
                    value = mcp.read_adc(ICPIN[xcontrol])
                    if value < med - (med - MIN[xcontrol])/ DEADZONE[xcontrol]:
                        value = (int)((float)(value - med) / (med - MIN[xcontrol]) * SCALE)
                    elif value > med + (MAX[xcontrol] - med)/ DEADZONE[xcontrol]:
                        value = (int)((float)(value - med) / (MAX[xcontrol] - med) * SCALE)
                    else:
                        value = 0
                    MOTORS[xmotor](value)
                    time.sleep( .2 )
        except KeyError:
                print "Error"


def controlMotor(xmotor):
        global control
        global motor
        global terminate
        xcontrol = control
        med = mcp.read_adc(ICPIN[xcontrol])

        mh = Adafruit_MotorHAT(addr=0x60)
        global mym
        if xmotor == "1":
            mym = mh.getMotor(PIN[xmotor])
        elif xmotor == "2":
            mym = mh.getStepper(200, PIN[xmotor])
        else:
            return
        #Speed [0,255]
        mym.setSpeed(150)
        try:
                while True:
                    if control != xcontrol or motor != xmotor or terminate:
                        turnOffMotors()
                        return
                    value = mcp.read_adc(ICPIN[xcontrol])
                    if value < med - (med - MIN[xcontrol])/ DEADZONE[xcontrol]:
                        value = (int)((float)(value - med) / (med - MIN[xcontrol]) * SCALE)
                    elif value > med + (MAX[xcontrol] - med)/ DEADZONE[xcontrol]:
                        value = (int)((float)(value - med) / (MAX[xcontrol] - med) * SCALE)
                    else:
                        value = 0
                    MOTORS[xmotor](value)
                    time.sleep( .05 )
        except KeyError:
                print "Error"

#####################################################################
#DCMotor
def DCMotor(value):
    global mym
    if value > 0:
        mym.run(Adafruit_MotorHAT.FORWARD)
    if value < 0:
        mym.run(Adafruit_MotorHAT.BACKWARD)
    mym.setSpeed((int)((float)(abs(value)) / SCALE * 255))
#####################################################################
#Stepper Motor
def stepperMotor(value):
    global mym
    if value == 0:
        return
    if value > 0:
        direction = Adafruit_MotorHAT.FORWARD        
    if value < 0:
        direction = Adafruit_MotorHAT.BACKWARD
    value = (int)((abs((float)(value))) / SCALE * 255)
    mym.setSpeed(value)
    if value < 64:
        steptype = Adafruit_MotorHAT.MICROSTEP
    elif value < 128:
        steptype = Adafruit_MotorHAT.SINGLE
    elif value < 256:
        steptype = Adafruit_MotorHAT.DOUBLE
        
    mym.step(5, direction, steptype)

#####################################################################
#Big Servo
def bigServo(value):
    value = (int)((float)(value) / SCALE * RANGE["3"]) + MEAN["3"]
    #Servo Company: [range of motion] counterclockwise bound to clockwise bound
    #EX1: [37,47),[47],(47,57]
    global pwm

    pwm.ChangeFrequency( value )
######################################################################
#Standard Servo
def standardServo(value):
    value = (int)((float)(value) / SCALE * RANGE["4"]) + MEAN["4"]
    #Servo Company: [range of motion] counterclockwise bound to clockwise bound
    #EX1: [20,100]
    #
    pwm.ChangeFrequency( value )

######################################################################
#Continuous Servo
def continuousServo(value):
    value = (int)((float)(value) / SCALE * RANGE["5"]) + MEAN["5"]
    #Servo Company: [levels of speed] direction
    #Parallax/Futaba Continuous: [40,50) counterclockwise, [50] zero, (50,60] clockwise
    #Spring RC: [] counterclockwise, [] zero, [] clockwise
    pwm.ChangeFrequency( value )


#####################################################################
#Tiny Servo
def tinyServo(value):
    value = (int)((float)(value) / SCALE * RANGE["6"]) + MEAN["6"]
    #Servo Company: [range of motion] counterclockwise bound to clockwise bound
    #EX1: [20-80]
    #

    if value < 0:
        return
    pwm.ChangeFrequency( value )
    
#####################################################################


MOTORS = { "1" : DCMotor,
           "2" : stepperMotor,
           "3" : bigServo,
           "4" : standardServo,
           "5" : continuousServo,
           "6" : tinyServo }
PIN = { "1" : 4,
        "2" : 1,
        "3" : 26,
        "4" : 19,
        "5" : 13,
        "6" : 6 }

MEAN = {"3" : 47,
        "4" : 60,
        "5" : 50,
        "6" : 50 }
        
RANGE = {"3" : 10,
         "4" : 40,
         "5" : 10,
         "6" : 30 }
#Software SPI configuration:
CLK  = 23
MISO = 18
MOSI = 15
CS   = 14
mcp = Adafruit_MCP3008.MCP3008(clk=CLK, cs=CS, miso=MISO, mosi=MOSI)

def turnOffMotors():
    try:
        mh.getMotor(1).run(Adafruit_MotorHAT.RELEASE)
        mh.getMotor(2).run(Adafruit_MotorHAT.RELEASE)
        mh.getMotor(3).run(Adafruit_MotorHAT.RELEASE)
        mh.getMotor(4).run(Adafruit_MotorHAT.RELEASE)
    except NameError:
        pass

atexit.register(turnOffMotors)

boardControl()
