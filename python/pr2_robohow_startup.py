# execfile('/home/fkeith/software/georg/install/lib/python2.7/site-packages/dynamic_graph/sot/robohow/hrp4_startup.py')

from dynamic_graph import plug
from dynamic_graph.sot.core import *
from dynamic_graph.sot.dynamics import *
from dynamic_graph.sot.core.meta_task_6d import toFlags
from dynamic_graph.sot.core.meta_task_visual_point import MetaTaskVisualPoint
from dynamic_graph.sot.core.matrix_util import matrixToTuple
from dynamic_graph.sot.core.meta_tasks_kine import MetaTaskKine6d
from dynamic_graph.sot.dyninv import TaskInequality, TaskJointLimits
from dynamic_graph.sot.robohow.tools import *
from dynamic_graph.sot.robohow.gripper import Gripper
from dynamic_graph.sot.robohow.cylinder_pouring import CylinderPouring
from dynamic_graph.sot.expression_graph.types import BaseElement
from dynamic_graph.sot.expression_graph.types import *
from dynamic_graph.sot.expression_graph.expression_graph import *
from dynamic_graph.sot.expression_graph.functions import *
from dynamic_graph.sot.robohow.superviser import *


# Binds with ROS. assert that roscore is running.
from dynamic_graph.ros import *
ros = Ros(robot)

# Create a simple kinematic solver.
from dynamic_graph.sot.dyninv import SolverKine

# load the initialization prototype for the pr2
from dynamic_graph.sot.pr2.pr2_tasks import *
solver = initialize(robot, SolverKine)

# allows the publication of the velocity in the JointState message
plug(solver.jointLimitator.control, ros.rosJointState.velocity)

# Creates the superviser, that will handle the stack of task update
superviser = Superviser(robot, solver, ros.rosPublish)

# Additional frames.
robot.frames['l_gripper'] = robot.frames['leftGripper']
robot.frames['r_gripper'] = robot.frames['rightGripper']
robot.expressions={}

taskBase = Pr2BaseTask(robot)
gotoNd(taskBase, (0,0,0,0,0,0), '100011')
#solver.push(taskBase.task)

#taskChest = Pr2ChestTask(robot)
#taskChest.feature.selec.value = '000100'
#taskChest.featureDes.position.value  = array([[1.0,0.0,0.0,0.0],[0,1,0.0,0.0],[0.0,0.,1.0,0.0],[0.0,0.0,0.85,1.0]])


#TODO: time hard coded!
taskJL = Pr2JointLimitsTask(robot,0.005)
(taskWeight, featureWeight) = Pr2Weight(robot)


### Initialization of the Universe

# define the properties of a bottle containing liquid
#TODO: externalize
def createBottle():
  cp = CylinderPouring('cp')
  cp.setHeight(0.19)
  cp.setRadius(0.0325)
  cp.setVolume(0.5)
  return cp


# Import the cup and bottle tf from ros
# TODO: use a method.
ros.rosSubscribe.add('matrixHomoStamped', 'bottle', 'bottle')
BaseElement.frames['bottle'] = ros.rosSubscribe.signal('bottle')

ros.rosSubscribe.add('matrixHomoStamped', 'cup', 'cup')
BaseElement.frames['cup'] = ros.rosSubscribe.signal('cup')

# TODO...
def estimateBottleFrameInHand(robot):
  # consider the difference of position between the bottle and the hand.
  # starting from that, define the robot (frame) corresponding to the top
  # of the bottle.
  bungFrame = OpPointModifier('bung')
  #TODO find the correct Z, position of the bung in the frame of the hand
  bungFrame.setTransformation(((1,0,0,0.0), (0,1,0,0.0), (0,0,1,0.10), (0,0,0,1)))
  plug(robot.frames['leftGripper'].position, bungFrame.positionIN)
  plug(robot.frames['leftGripper'].jacobian, bungFrame.jacobianIN)
#  frame.position.recompute(bungFrame.position.time + 1)
#  frame.jacobian.recompute(bungFrame.jacobian.time + 1)
  robot.frames['bung'] = bungFrame


def initChestTask(robot):
  name = "chest"
  dim = 1
  index = 18
  feature = FeatureGeneric('feature_'+name)
  featureDes = FeatureGeneric('featureDes_'+name)
  featureDes.errorIN.value=(0.2,)
  feature.setReference('featureDes_'+name)
  featureDes.errorIN.value = (0,) * dim;
  robot.features[name] = feature 
  robot.features['Des'+name] = feature 

  # create jacobian.
  jacobianGripper = eye(dim,robot.dimension) * 0;
  jacobianGripper[0][index] = 1;
  feature.jacobianIN.value = jacobianGripper

  # only selec some dofs
  selecFeatureChest = Selec_of_vector('selecfeature_'+name)
  selecFeatureChest.selec(index, index+dim)
  plug(robot.dynamic.position, selecFeatureChest.sin)
  plug(selecFeatureChest.sout, feature.errorIN)

  # 2\ Define the task. Associate to the task the position feature.
  task = Task('task_'+name)
  task.add('feature_'+name)
  task.controlGain.value = 1
  robot.tasks[name] = task



bottle = createBottle()
estimateBottleFrameInHand(robot)

# define the posture task.
def testJoint(robot, index, angle):
  currentState = robot.dynamic.position.value
  #state(index) = angle
#  posture = currentState[0:index] + (angle,) + currentState[index+1: len(robot.halfSitting)]
  posture = (0,)* 18  + \
    (0.31, 0, 1.05) +\
    (1.0650093105988152,     0.26376743371555295,     1.392565491097796,    -1.629946646305397,       0.524,     -0.9668414952685922,  -1.8614) +\
    (0,)* 6 +\
    (0.04,) +  \
    (0,) +\
    ( -1.265335905500992,     1.2666995326579538,     -1.9643297630604963,     -0.2625872772879775,     5.81991983730232,     -0.13242260444085052,     2.64) +\
    (0,)* 8
  robot.features['featurePosition'].posture.value = posture
  robot.features['featurePosition'].selec.value = '0'*18 + '1' + '0' + '1'*8 + '0'*6 + '10' + '1'*7+ '0'*8

#  robot.features['featurePosition'].posture.value = posture
#  robot.features['featurePosition'].selec.value = '0' * index + '1' + '0' * (len(robot.halfSitting) - index-1)

initPostureTask(robot)
testJoint(robot, 18, 0.2)

#initChestTask(robot)


#TODO: should be formulated using an expression graph task
taskLH = Pr2LeftHandTask(robot)
taskLH.feature.frame('desired')
taskLH.feature.selec.value = '000111'
plug(BaseElement.frames['bottle'], taskLH.featureDes.position)
robot.tasks['taskleft-wrist'] = taskLH.task

def displayError():
  l = solver.sot.getTaskList()
  list = l.split('|')
  for t in list:
    if t in robot.tasks:
      print t, robot.tasks[t].className, robot.tasks[t].error.value
  
#robot.tasks['robot_task_position'].controlGain.set(1,1, 0.1)
gainPosition = GainAdaptive('gainPosition') 
plug(robot.tasks['robot_task_position'].error,gainPosition.error)
plug(gainPosition.gain,robot.tasks['robot_task_position'].controlGain)


gainPosition.set(0.1,0.1,1e3)

robot.tasks['robot_task_position'].controlGain.value = 1
