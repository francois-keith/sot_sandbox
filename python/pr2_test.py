#This simple test allows to check the joints of the robot one by one.
# 

# 1. Init robot, ros binding, solver
from dynamic_graph.sot.pr2.pr2_tasks import *
from dynamic_graph.sot.core.meta_tasks_kine import gotoNd
from dynamic_graph import plug


# Binds with ROS. assert that roscore is running.
from dynamic_graph.ros import *
ros = Ros(robot)

# Create a simple kinematic solver.
from dynamic_graph.sot.dyninv import SolverKine

# load the initialization prototype for the pr2
from dynamic_graph.sot.pr2.pr2_tasks import *
solver = initialize(robot, SolverKine)


#### Build the stack of tasks
taskBase = Pr2BaseTask(robot)
gotoNd(taskBase, (0,0,0,0,0,0), '100011')
solver.push(taskBase.task)


# 3. Init Tasks
initPostureTask(robot)
solver.push(robot.tasks['robot_task_position'])

# set Zero to all the joints.
def setZero(robot):
  robot.features['featurePosition'].posture.value = (0,)* len(robot.halfSitting)

# set Zero to all the joints.
def setInitial(robot):
  robot.features['featurePosition'].posture.value =robot.halfSitting

# allows to change only one joint and to test 
def testJoint(robot, index, angle):
  currentState = robot.dynamic.position.value
  #state(index) = angle
  posture = currentState[0:index] + (angle,) + currentState[index+1: len(robot.halfSitting)]
  robot.features['featurePosition'].posture.value = posture

# Set position
def setRootPosition(robot, root):
  homo = root[0:2] + (0,0,0,) + (root[2],)
  print homo
  gotoNd(taskBase, homo, '100011')

def help():
  print 'setZero(robot):    set Zero to all the joints.'
  print 'setInitial(robot): reset to the initial position'
  print 'testJoint(robot, index, angle): set the joint(index) to the value angle'
  print 'setRootPosition(robot, root):   defines the root position (X,Y,theta)'

