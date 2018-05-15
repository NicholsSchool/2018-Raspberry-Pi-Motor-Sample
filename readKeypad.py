import time
import RPi.GPIO as GPIO
import thread
GPIO.setmode(GPIO.BOARD)

KEYPAD =  [
          ["1","2","3","A"],
          ["4","5","6","B"],
          ["7","8","9","C"],
          ["*","0","#","D"]
          ]

ROW = [40,38,36,32]
COL = [26,24,22,18]

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
                    print KEYPAD[i][j]
                    while( GPIO.input( ROW[i] ) == 0):
                        pass
            GPIO.output(COL[j],1)
        time.sleep(.2)
except KeyboardInterrupt:
    GPIO.cleanup()
