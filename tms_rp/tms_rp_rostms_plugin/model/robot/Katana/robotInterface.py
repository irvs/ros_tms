#testfunction
def multiply(a,b):
    print "Will compute", a, "times", b
    c = 0
    for i in range(0, a):
        c = c + b
    return c

#robot go to home position
def gotoHomePosition():
	print "gotoHomePosition"
	print commands.getoutput("manip home")
	return


#robot go to home position not implemented
def gotoTargetPosition(j0,j1,j2,j3,j4,j0,j5,j6,j7):
	print "gotoHomePosition"
#	print commands.getoutput("manip home")
               pring commands.getoutput("manip abs_jmove 0.0 0.0 0.0 90.0 0.0 90.0 0.0")
	return

def doServoOn():
              print "doServoOn"
              print commands.getoutput("manip start")
              print commands.getoutput("manip serv_on")

def doServoOff():
              print "doServoOff"
              print commands.getoutput("manip serv_off")
              print commands.getoutput("manip end")

import commands
