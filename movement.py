import serial,time
from sys import version_info

PY2 = version_info[0] == 2   #Running Python 2.x?

#
#---------------------------
# Maestro Servo Controller
#---------------------------
#
# Support for the Pololu Maestro line of servo controllers
#
# Steven Jacobs -- Aug 2013
# https://github.com/FRC4564/Maestro/
#
# These functions provide access to many of the Maestro's capabilities using the
# Pololu serial protocol
#
class Controller:
    # When connected via USB, the Maestro creates two virtual serial ports
    # /dev/ttyACM0 for commands and /dev/ttyACM1 for communications.
    # Be sure the Maestro is configured for "USB Dual Port" serial mode.
    # "USB Chained Mode" may work as well, but hasn't been tested.
    #
    # Pololu protocol allows for multiple Maestros to be connected to a single
    # serial port. Each connected device is then indexed by number.
    # This device number defaults to 0x0C (or 12 in decimal), which this module
    # assumes.  If two or more controllers are connected to different serial
    # ports, or you are using a Windows OS, you can provide the tty port.  For
    # example, '/dev/ttyACM2' or for Windows, something like 'COM3'.
    def __init__(self,ttyStr='/dev/ttyACM0',device=0x0c):
        # Open the command port
        self.usb = serial.Serial(ttyStr)
        # Command lead-in and device number are sent for each Pololu serial command.
        self.PololuCmd = chr(0xaa) + chr(device)
        # Track target position for each servo. The function isMoving() will
        # use the Target vs Current servo position to determine if movement is
        # occuring.  Upto 24 servos on a Maestro, (0-23). Targets start at 0.
        self.Targets = [0] * 24
        # Servo minimum and maximum targets can be restricted to protect components.
        self.Mins = [0] * 24
        self.Maxs = [0] * 24
        
    # Cleanup by closing USB serial port
    def close(self):
        self.usb.close()

    # Send a Pololu command out the serial port
    def sendCmd(self, cmd):
        cmdStr = self.PololuCmd + cmd
        if PY2:
            self.usb.write(cmdStr)
        else:
            self.usb.write(bytes(cmdStr,'latin-1'))

    # Set channels min and max value range.  Use this as a safety to protect
    # from accidentally moving outside known safe parameters. A setting of 0
    # allows unrestricted movement.
    #
    # ***Note that the Maestro itself is configured to limit the range of servo travel
    # which has precedence over these values.  Use the Maestro Control Center to configure
    # ranges that are saved to the controller.  Use setRange for software controllable ranges.
    def setRange(self, chan, min, max):
        self.Mins[chan] = min
        self.Maxs[chan] = max

    # Return Minimum channel range value
    def getMin(self, chan):
        return self.Mins[chan]

    # Return Maximum channel range value
    def getMax(self, chan):
        return self.Maxs[chan]
        
    # Set channel to a specified target value.  Servo will begin moving based
    # on Speed and Acceleration parameters previously set.
    # Target values will be constrained within Min and Max range, if set.
    # For servos, target represents the pulse width in of quarter-microseconds
    # Servo center is at 1500 microseconds, or 6000 quarter-microseconds
    # Typcially valid servo range is 3000 to 9000 quarter-microseconds
    # If channel is configured for digital output, values < 6000 = Low ouput
    def setTarget(self, chan, target):
        # if Min is defined and Target is below, force to Min
        if self.Mins[chan] > 0 and target < self.Mins[chan]:
            target = self.Mins[chan]
        # if Max is defined and Target is above, force to Max
        if self.Maxs[chan] > 0 and target > self.Maxs[chan]:
            target = self.Maxs[chan]
        #    
        lsb = target & 0x7f #7 bits for least significant byte
        msb = (target >> 7) & 0x7f #shift 7 and take next 7 bits for msb
        cmd = chr(0x04) + chr(chan) + chr(lsb) + chr(msb)
        self.sendCmd(cmd)
        # Record Target value
        self.Targets[chan] = target
        
    # Set speed of channel
    # Speed is measured as 0.25microseconds/10milliseconds
    # For the standard 1ms pulse width change to move a servo between extremes, a speed
    # of 1 will take 1 minute, and a speed of 60 would take 1 second.
    # Speed of 0 is unrestricted.
    def setSpeed(self, chan, speed):
        lsb = speed & 0x7f #7 bits for least significant byte
        msb = (speed >> 7) & 0x7f #shift 7 and take next 7 bits for msb
        cmd = chr(0x07) + chr(chan) + chr(lsb) + chr(msb)
        self.sendCmd(cmd)

    # Set acceleration of channel
    # This provide soft starts and finishes when servo moves to target position.
    # Valid values are from 0 to 255. 0=unrestricted, 1 is slowest start.
    # A value of 1 will take the servo about 3s to move between 1ms to 2ms range.
    def setAccel(self, chan, accel):
        lsb = accel & 0x7f #7 bits for least significant byte
        msb = (accel >> 7) & 0x7f #shift 7 and take next 7 bits for msb
        cmd = chr(0x09) + chr(chan) + chr(lsb) + chr(msb)
        self.sendCmd(cmd)
    
    # Get the current position of the device on the specified channel
    # The result is returned in a measure of quarter-microseconds, which mirrors
    # the Target parameter of setTarget.
    # This is not reading the true servo position, but the last target position sent
    # to the servo. If the Speed is set to below the top speed of the servo, then
    # the position result will align well with the acutal servo position, assuming
    # it is not stalled or slowed.
    def getPosition(self, chan):
        cmd = chr(0x10) + chr(chan)
        self.sendCmd(cmd)
        lsb = ord(self.usb.read())
        msb = ord(self.usb.read())
        return (msb << 8) + lsb

    # Test to see if a servo has reached the set target position.  This only provides
    # useful results if the Speed parameter is set slower than the maximum speed of
    # the servo.  Servo range must be defined first using setRange. See setRange comment.
    #
    # ***Note if target position goes outside of Maestro's allowable range for the
    # channel, then the target can never be reached, so it will appear to always be
    # moving to the target.  
    def isMoving(self, chan):
        if self.Targets[chan] > 0:
            if self.getPosition(chan) != self.Targets[chan]:
                return True
        return False
    
    # Have all servo outputs reached their targets? This is useful only if Speed and/or
    # Acceleration have been set on one or more of the channels. Returns True or False.
    # Not available with Micro Maestro.
    def getMovingState(self):
        cmd = chr(0x13)
        self.sendCmd(cmd)
        if self.usb.read() == chr(0):
            return False
        else:
            return True

    # Run a Maestro Script subroutine in the currently active script. Scripts can
    # have multiple subroutines, which get numbered sequentially from 0 on up. Code your
    # Maestro subroutine to either infinitely loop, or just end (return is not valid).
    def runScriptSub(self, subNumber):
        cmd = chr(0x27) + chr(subNumber)
        # can pass a param with command 0x28
        # cmd = chr(0x28) + chr(subNumber) + chr(lsb) + chr(msb)
        self.sendCmd(cmd)

    # Stop the current Maestro Script
    def stopScript(self):
        cmd = chr(0x24)
        self.sendCmd(cmd)

    def rotateHeadRight(self):
        self.setRange(3,4000,8000)
        self.setSpeed(3,1000)
        if(self.getPosition(3)<8000):
            self.setTarget(3,self.getPosition(3)+1000)

    def rotateHeadLeft(self):
        self.setRange(3,4000,8000)
        self.setSpeed(3,1000)
        if(self.getPosition(3)>4000):
            self.setTarget(3,self.getPosition(3)-1000)

    def tiltHeadUp(self):
        self.setRange(4,4000,8000)
        self.setSpeed(4,1000)
        if(self.getPosition(4)<8000):
            self.setTarget(4,self.getPosition(4)+1000)

    def tiltHeadDown(self):
        self.setRange(4,4000,8000)
        self.setSpeed(4,1000)
        if(self.getPosition(4)>4000):
            self.setTarget(4,self.getPosition(4)-1000)

    def turnRight(self):
        self.setRange(2,5000,7000)
        self.setSpeed(2,1000)
        self.setTarget(2,7000)

    def turnLeft(self):
        self.setRange(2,5000,7000)
        self.setSpeed(2,1000)
        self.setTarget(2,5000)

    def moveForward(self):
        self.setRange(1,5000,7000)
        self.setSpeed(1,1000)
        self.setTarget(1,6700)

    def moveForwarder(self):
        self.setRange(1,5000,7000)
        self.setSpeed(1,1000)
        self.setTarget(1,6800)

    def moveForwardest(self):
        self.setRange(1,5000,7000)
        self.setSpeed(1,1000)
        self.setTarget(1,6900)

    def moveBackward(self):
        self.setRange(1,5000,7000)
        self.setSpeed(1,1000)
        self.setTarget(1,5400)

    def moveBackwarder(self):
        self.setRange(1,5000,7000)
        self.setSpeed(1,1000)
        self.setTarget(1,5300)

    def moveBackwardest(self):
        self.setRange(1,5000,7000)
        self.setSpeed(1,1000)
        self.setTarget(1,5200)

    def stopMotion(self):
        self.setRange(1,5000,7000)
        self.setSpeed(1,1000)
        self.setTarget(1,6000)

    def resetAll(self):
        self.setSpeed(3,1000)
        self.setTarget(3,6000)
        self.setSpeed(4,1000)
        self.setTarget(4,6000)
        self.setSpeed(0,1000)
        self.setTarget(0,6000)
        self.setSpeed(2,1000)
        self.setTarget(2,6000)
        self.setSpeed(1,1000)
        self.setTarget(1,6000)


m = Controller()
#m.resetAll()
while True:
    x = input()
    if(x == 'q'):
        m.moveForward()
        print("moving forward 1")
    elif(x == 'w'):
        m.moveForwarder()
        print("moving forward 2")
    elif(x == 'e'):
        m.moveForwardest()
        print("moving forward 3")
    elif(x == 'a'):
        m.turnLeft()
        print("turn left")
    elif(x == 'z'):
        m.moveBackward()
        print("moving backward 1")
    elif(x == 'x'):
        m.moveBackwarder()
        print("moving backward 2")
    elif(x == 'c'):
        m.moveBackwardest()
        print("moving backward 3")
    elif(x == 'd'):
        #m.stopMotion()
        m.turnRight()
        print("turn right")
    elif(x =='s'):
        m.stopMotion()
        print("stopping")


    elif(x == 'b'):
        m.setSpeed(0,1000)
        m.setTarget(0,3000)
        print("twist left")
    elif(x == 'n'):
        m.setSpeed(0,1000)
        m.setTarget(0,6000)
        print("twist center")
    elif(x == 'm'):
        m.setSpeed(0,1000)
        m.setTarget(0,9000)
        print("twist right")


    elif(x == 'i'):
        m.tiltHeadUp()
        print("tilting head up")
    elif(x == 'k'):
        m.tiltHeadDown()
        print("tilting head down")
    elif(x == 'j'):
        m.rotateHeadLeft()
        print("rotating head left")
    elif(x == 'l'):
        m.rotateHeadRight()
        print("rotating head right")

        
    elif(x == 'r'):
        m.resetAll()
        print("resetting conditions")
    else:
        print("invalid key")

''' 
        0 torso 3 9
        1 wheels backwards and forwards 5 7
        2 wheels left right 5 7 
        3 head turning 4 8
        4 head tilting 4 8'''