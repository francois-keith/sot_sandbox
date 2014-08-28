#!/usr/bin/env python
import roslib
import rospy
import actionlib

from sensor_msgs.msg import Joy
from geometry_msgs.msg import Vector3
from dynamic_graph_actionlib.msg import *
from dynamic_graph_bridge.srv import RunCommand

## Common part

""" convert a vector3 to a string """
def vector3ToStr(vec):
  st = "(%f, %f, %f)" % (vec.x, vec.y, vec.z)
  return st;

""" run an inscruction """
def runCommand(proxy, instruction):
  rospy.logdebug ("run instruction: " + instruction)
  result = proxy (instruction)
  rospy.loginfo ("stdout: " + result.stdout)
#  rospy.loginfo ("stderr: " + result.stderr)


""" convert a vector3 into a command """
def convertVelocityToCommands(proxy, vec):
  # push the task in the solver
  instruction = "robot.pg.velocitydes.value ="+vector3ToStr(vec)
  runCommand(proxy, instruction)


"""
VelocityPublisher listens to the joystick signals and convert it into
a command sent to the sot through run_command.
"""
class VelocityPublisher:

  run_command = None
  vel = Vector3(0,0,0)
  maxvel = Vector3(0.3,    0.25,  0.25)
  minvel = Vector3(-0.28, -0.25, -0.25)

  def __init__(self):
    # Create the node
    rospy.init_node('velPublisher', anonymous=True)

    # Wait for the run_command service to be started
    rospy.loginfo("\n " + rospy.get_name() + " waiting for run_command")
    rospy.wait_for_service ('run_command')
    rospy.loginfo(rospy.get_name() + "run_command obtained")
    self.run_command = rospy.ServiceProxy ('run_command', RunCommand)

    # Subscribe to the constraint publisher
    rospy.Subscriber("/joy", Joy, self.callback)
    rospy.spin()

  def callback(self, data):
    prevVel = Vector3(self.vel.x, self.vel.y, self.vel.z)
    # select button: display manuel
    if data.buttons[8] == 1:
      message = "\n *Controller for the pg wia the joystick (optimized for Microsoft paddle) *\n"
      message = message + "down/up   : decrease/increase the frontal velocity\n"
      message = message + "left/right: straff left/right (lateral velocity)\n"
      message = message + "L   /R    : turn left/right\n"
      message = message + "A: set frontal velocity to 0\n"
      message = message + "B: set lateral velocity to 0\n"
      message = message + "C: set rotation velocity to 0\n"
      message = message + "Start: set velocity to 0\n"

      rospy.loginfo(message)

    # start button: pause => immediate stop 
    if data.buttons[9] == 1:
      self.vel.x = self.vel.y = self.vel.z = 0
    elif data.buttons[0] == 1:
      self.vel.x = 0
    elif data.buttons[1] == 1:
      self.vel.y = 0
    elif data.buttons[2] == 1:
      self.vel.z = 0
    else :
      self.vel.x = self.vel.x + data.axes[1] * 0.02
      self.vel.y = self.vel.y + data.axes[0] * 0.02
      self.vel.z = self.vel.z + (data.buttons[6] - data.buttons[7]) * 0.02

      # Stay in the bounds...
      self.vel.x = min (self.maxvel.x, max( self.vel.x , self.minvel.x))
      self.vel.y = min (self.maxvel.y, max( self.vel.y , self.minvel.y))
      self.vel.z = min (self.maxvel.z, max( self.vel.z , self.minvel.z))

    # convert the constrain and the send the corresponding command
    if not (prevVel == self.vel):
      convertVelocityToCommands(self.run_command, self.vel)

# Start the listener
if __name__ == '__main__':
  pgvel = VelocityPublisher()

