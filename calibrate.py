from pybricks.hubs import PrimeHub
from pybricks.pupdevices import Motor, ColorSensor, UltrasonicSensor, ForceSensor
from pybricks.parameters import Button, Color, Direction, Port, Side, Stop
from pybricks.robotics import DriveBase
from pybricks.tools import wait, StopWatch
from pupremote_hub import PUPRemoteHub

# Initialize hardware
hub = PrimeHub()
claw = Motor(Port.B, Direction.CLOCKWISE)
slide = Motor(Port.F, Direction.CLOCKWISE)
stopper = Motor(Port.E, Direction.COUNTERCLOCKWISE)
lmotor = Motor(Port.D, Direction.COUNTERCLOCKWISE)
rmotor = Motor(Port.C,Direction.CLOCKWISE)
pr = PUPRemoteHub(Port.A)
pr.add_channel('lines', 'bbB')
pr.add_command('calib', from_hub_fmt='b')
w = StopWatch()

db = DriveBase(lmotor, rmotor, 62.4, 160)


pr.call('calib',1) # Start calibration.
db.settings(straight_speed=100)
for i in range(8):
    db.straight(50)
    db.straight(-50)        
pr.call('calib',0) # Stop calibration.
