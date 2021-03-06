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

from dynamic_graph.sot.application.velocity.precomputed_tasks import initialize
solver = initialize ( robot, SolverKine )

# load the initialization prototype for the pr2
#from dynamic_graph.sot.pr2.pr2_tasks import *
#solver = initialize(robot, SolverKine)

# allows the publication of the velocity in the JointState message
plug(solver.jointLimitator.control, ros.rosJointState.velocity)

# Creates the superviser, that will handle the stack of task update
superviser = Superviser(robot, solver, ros.rosPublish)

# Additional frames.
robot.frames['l_gripper'] = robot.frames['leftGripper']
robot.frames['r_gripper'] = robot.frames['rightGripper']
robot.expressions={}

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
ros.rosSubscribe.add('matrixHomoStamped', 'table', 'table')
BaseElement.frames['table'] = ros.rosSubscribe.signal('table')

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
  bungFrame.setTransformation(((1,0,0,0.0), (0,1,0,0.0), (0,0,1, 0.10), (0,0,0,1)))
  plug(robot.frames['rightGripper'].position, bungFrame.positionIN)
  plug(robot.frames['rightGripper'].jacobian, bungFrame.jacobianIN)
#  frame.position.recompute(bungFrame.position.time + 1)
#  frame.jacobian.recompute(bungFrame.jacobian.time + 1)
  robot.frames['bung'] = bungFrame


def estimateCupFrameInHand(robot):
  # consider the difference of position between the bottle and the hand.
  # starting from that, define the robot (frame) corresponding to the top
  # of the bottle.
  topcupFrame = OpPointModifier('topcup')
  #TODO find the correct Z, position of the topcup in the frame of the hand
  topcupFrame.setTransformation(((1,0,0,0.0), (0,1,0,0.0), (0,0,1,-0.10), (0,0,0,1)))
  plug(robot.frames['leftGripper'].position, topcupFrame.positionIN)
  plug(robot.frames['leftGripper'].jacobian, topcupFrame.jacobianIN)
  robot.frames['topcup'] = topcupFrame


def initPostureTask(robot):
  # --- TASK POSTURE --------------------------------------------------
  # set a default position for the joints. 
  robot.features['featurePosition'] = FeaturePosture('featurePosition')
  plug(robot.device.state,robot.features['featurePosition'].state)
  robotDim = len(robot.dynamic.velocity.value)
  robot.features['featurePosition'].posture.value = robot.halfSitting
  if robot.device.name == 'HRP2LAAS' or \
     robot.device.name == 'HRP2JRL':
    postureTaskDofs = [ False,False,False,False,False,False, \
                        False,False,False,False,False,False, \
                        True,True,True,True, \
                        True,True,True,True,True,True,True, \
                        True,True,True,True,True,True,True ]
  elif robot.device.name == 'HRP4LIRMM':
    # Right Leg, Left leg, chest, right arm, left arm
    postureTaskDofs = [False]*6 +  [False]*6 + [True]*4 + [True]*9 + [True]*9
  elif robot.device.name == 'ROMEO':
    # chest, left/right arms, left/right legs
    postureTaskDofs = [True]*5 + [True]*7 + [True]*7 + [False]*7 + [False]*7
  else:
    print "/!\\ walking.py: The robot " +robot.device.name+ " is unknown."
    print "  Default posture task froze all the dofs"
    postureTaskDofs=[True] * (robot.dimension-6)
  for dof,isEnabled in enumerate(postureTaskDofs):
    robot.features['featurePosition'].selectDof(dof+6,isEnabled)
  robot.tasks['robot_task_position']=Task('robot_task_position')
  robot.tasks['robot_task_position'].add('featurePosition')
  gainPosition = GainAdaptive('gainPosition')
  gainPosition.set(0.1,0.1,125e3)
  gainPosition.gain.value = 5
  plug(robot.tasks['robot_task_position'].error,gainPosition.error)
  plug(gainPosition.gain,robot.tasks['robot_task_position'].controlGain)



def initChestHeadTask(robot):
  # --- TASK POSTURE --------------------------------------------------
  # set a default position for the joints. 
  robot.features['featurechest_head'] = FeaturePosture('featurechest_head')
  plug(robot.device.state,robot.features['featurechest_head'].state)
  robotDim = len(robot.dynamic.velocity.value)
  robot.features['featurechest_head'].posture.value = robot.halfSitting
  if robot.device.name == 'HRP2LAAS' or \
     robot.device.name == 'HRP2JRL':
    postureTaskDofs = [ False,False,False,False,False,False, \
                        False,False,False,False,False,False, \
                        True,True,True,True, \
                        False,False,False,False,False,False,False, \
                        False,False,False,False,False,False,False ]
  elif robot.device.name == 'HRP4LIRMM':
    # Right Leg, Left leg, chest, right arm, left arm
    postureTaskDofs = [False]*6 +  [False]*6 + [True]*4 + [False]*9 + [False]*9
  elif robot.device.name == 'ROMEO':
    # chest, left/right arms, left/right legs
    postureTaskDofs = [False]*5 + [False]*7 + [True]*7 + [False]*7 + [False]*7
  else:
    print "/!\\ walking.py: The robot " +robot.device.name+ " is unknown."
    print "  Default posture task froze all the dofs"
    postureTaskDofs=[True] * (robot.dimension-6)
  for dof,isEnabled in enumerate(postureTaskDofs):
    robot.features['featurechest_head'].selectDof(dof+6,isEnabled)
  robot.tasks['chest_head']=Task('chest_head')
  robot.tasks['chest_head'].add('featurechest_head')
  gainchest_head = GainAdaptive('gainchest_head')
  gainchest_head.set(0.1,0.1,125e3)
  gainchest_head.gain.value = 5
  plug(robot.tasks['robot_task_position'].error,gainchest_head.error)
  plug(gainchest_head.gain,robot.tasks['robot_task_position'].controlGain)

initPostureTask(robot)


#########################################################

# --- JOINT LIMITS TASK
# set a default position for the joints. 
robot.features['feature_jl'] = FeaturePosture('feature_jl')
plug(robot.device.state,robot.features['feature_jl'].state)
robotDim = len(robot.dynamic.velocity.value)
robot.features['feature_jl'].posture.value = robot.halfSitting

# Right Leg, Left leg, chest+neck, right arm, left arm
postureTaskDofs = \
	  [False]*3 + [False]*3 + [False]*3 + [False]*3 \
	+ [True] *1 + [False]*3 \
	+ [False]*1 + [True] *1 + [False]*5 + [False]*2 \
	+ [False]*1 + [True] *1 + [False]*5 + [False]*2 

for dof,isEnabled in enumerate(postureTaskDofs):
	robot.features['feature_jl'].selectDof(dof+6,isEnabled)
  
taskJointLimit=TaskInequality('taskJL')
taskJointLimit.add('feature_jl')

# Limits on these joints: Shoulder Roll, Thumb, Fingers (Left and Right) 
#taskJointLimit.referenceInf.value = (r_hip_yaw, r_hip_roll, r_hip_pitch, l_hip_yaw, l_hip_roll, l_hip_pitch, -1.55 ,1.57, 1.57,-0.005, -1.57,-1.57) 
#taskJointLimit.referenceSup.value = (r_hip_yaw, r_hip_roll, r_hip_pitch, l_hip_yaw, l_hip_roll, l_hip_pitch,  0.005,1.57, 1.57, 1.55 , -1.57,-1.57) 
taskJointLimit.referenceInf.value = (0.00,-1.550,-0.005) 
taskJointLimit.referenceSup.value = (0.10, 0.005, 1.550) 
taskJointLimit.dt.value= 0.005
taskJointLimit.controlGain.value = 0.009

#########################################################


bottle = createBottle()
estimateBottleFrameInHand(robot)
estimateCupFrameInHand(robot)


#TODO: PLEASE !!
#TODO: should be formulated using an expression graph task
taskLH = MetaTaskKine6d('left-wrist',robot.dynamic,'left-wrist','left-wrist')
taskLH.feature.selec.value = '000111'
taskLH.feature.frame('desired')
plug(BaseElement.frames['cup'], taskLH.featureDes.position)
robot.tasks['taskleft-wrist'] = taskLH.task


#TODO: PLEASE !!
#TODO: should be formulated using an expression graph task
taskRH = MetaTaskKine6d('right-wrist',robot.dynamic,'right-wrist','right-wrist')
taskRH.feature.selec.value = '000111'
taskRH.feature.frame('desired')
plug(BaseElement.frames['bottle'], taskRH.featureDes.position)
robot.tasks['taskright-wrist'] = taskRH.task


r_gripper_opening = Gripper('r_gripper_opening', robot)
robot.tasks['r_gripper_opening'] = r_gripper_opening.task
robot.features['r_gripper_opening'] = r_gripper_opening.feature

# --- KEEP FIX the WAIST
taskWAIST=MetaTaskKine6d('waist',robot.dynamic,'waist','waist')
taskWAIST.gain.set(2,0.1,0.1)
taskWAIST.feature.frame('desired')
taskWAIST.featureDes.position.value = robot.waist.position.value

def close1():
  r_gripper_opening.featureDes.errorIN.value =(1,0)

def close2():
  r_gripper_opening.featureDes.errorIN.value =(1,0.47)


