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
  plug(robot.frames['rightGripper'].position, bungFrame.positionIN)
  plug(robot.frames['rightGripper'].jacobian, bungFrame.jacobianIN)
#  frame.position.recompute(bungFrame.position.time + 1)
#  frame.jacobian.recompute(bungFrame.jacobian.time + 1)
  robot.frames['bung'] = bungFrame



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


bottle = createBottle()
estimateBottleFrameInHand(robot)

#TODO: should be formulated using an expression graph task
taskRH = Pr2RightHandTask(robot)
taskRH.feature.frame('desired')
taskRH.feature.selec.value = '111111'
plug(BaseElement.frames['bottle'], taskRH.featureDes.position)
robot.tasks['taskright-wrist'] = taskRH.task

def displayError():
  l = solver.sot.getTaskList()
  list = l.split('|')
  for t in list:
    if t in robot.tasks:
      print t, robot.tasks[t].className, robot.tasks[t].error.value
  


















index = 0
def step():
  global index
  if index == 0:
    superviser.clear()
    superviser.push('taskcontact')
    superviser.push('taskbase')
    superviser.push('taskJL')
    superviser.push('weight')
    superviser.update()

  elif index == 1:
    print "task wrist"
    superviser.clear()
    superviser.push('taskcontact')
    superviser.push('taskbase')
    superviser.push('taskJL')
    setTaskGoal(robot, 'taskright-wrist', (), (), '', (1.000000, ))
    superviser.push('taskright-wrist')
    superviser.push('weight')
    superviser.update()

  elif index == 2:
    print "task wrist + pouting"
    superviser.clear()
    superviser.push('taskcontact')
    superviser.push('taskbase')
    superviser.push('taskJL')
    setTaskGoal(robot, 'taskright-wrist', (), (), '', (1.000000, ))
    superviser.push('taskright-wrist')
    createExpression(robot, VersorElement('bung_x', robot, 'bung', versor = (1.000000, 0.000000, 0.000000)))
    createExpression(robot, VersorElement('ground_z', robot, 'ground', versor = (0.000000, 0.000000, 1.000000)))
    createTask(robot,'angle_pouring', 'bung_x', 'ground_z', 'angle', lowerBound = (1.570796, ), upperBound = (1.570796, ))
    setTaskGoal(robot, 'angle_pouring', (1.570796, ), (1.570796, ), '', ())
    superviser.push('angle_pouring')
    superviser.push('weight')
    superviser.update()

  elif index == 3:
    print "pouring position"
    superviser.clear()
    superviser.push('taskcontact')
    superviser.push('taskbase')
    superviser.push('taskJL')
    createExpression(robot, VersorElement('bung_x', robot, 'bung', versor = (1.000000, 0.000000, 0.000000)))
    createExpression(robot, VersorElement('ground_z', robot, 'ground', versor = (0.000000, 0.000000, 1.000000)))
    createTask(robot,'angle_pouring', 'bung_x', 'ground_z', 'angle', lowerBound = (1.570796, ), upperBound = (1.570796, ))
    setTaskGoal(robot, 'angle_pouring', (1.570796, ), (1.570796, ), '', ())
    createExpression(robot, PointElement('bung', robot, 'bung', position = (0.000000, 0.000000, 0.000000)))
    createExpression(robot, PointElement('cup', robot, 'cup', position = (0.000000, 0.000000, 0.000000)))
    createTask(robot,'position_bung_Z', 'bung', 'cup', 'position', lowerBound = (0.000000, ), upperBound = (0.000000, ))
    setTaskGoal(robot, 'position_bung_Z', (0.000000, ), (0.000000, ), '100', ())
    superviser.push('position_bung_Z')
    createExpression(robot, PointElement('cup', robot, 'cup', position = (0.000000, 0.000000, 0.000000)))
    createExpression(robot, PointElement('bung', robot, 'bung', position = (0.000000, 0.000000, 0.000000)))
    createTask(robot,'position_bung_XY', 'cup', 'bung', 'position', lowerBound = (-0.025000, -0.025000, ), upperBound = (0.025000, 0.025000, ))
    setTaskGoal(robot, 'position_bung_XY', (-0.025000, -0.025000, ), (0.025000, 0.025000, ), '011', ())
    superviser.push('position_bung_XY')
    createExpression(robot, PlaneElement('ground_plane', robot, 'ground', normal = (0.000000, 0.000000, 1.000000),position = (0.000000, 0.000000, 0.000000)))
    createExpression(robot, VersorElement('r_gripper_y', robot, 'r_gripper', versor = (0.000000, 1.000000, 0.000000)))
    createTask(robot,'angle_gripperY_in_ground_plane', 'ground_plane', 'r_gripper_y', 'angle', lowerBound = (0.000000, ), upperBound = (0.000000, ))
    setTaskGoal(robot, 'angle_gripperY_in_ground_plane', (0.000000, ), (0.000000, ), '', ())
    superviser.push('angle_pouring')
 #   superviser.push('angle_gripperY_in_ground_plane')
    superviser.push('weight')
    superviser.update()

  elif index == 4:
    print "pouring a little"
    superviser.clear()
    superviser.push('taskcontact')
    superviser.push('taskbase')
    superviser.push('taskJL')
    createExpression(robot, VersorElement('bung_x', robot, 'bung', versor = (1.000000, 0.000000, 0.000000)))
    createExpression(robot, VersorElement('ground_z', robot, 'ground', versor = (0.000000, 0.000000, 1.000000)))
    createTask(robot,'angle_pouring', 'bung_x', 'ground_z', 'angle', lowerBound = (1.745329, ), upperBound = (1.745329, ))
    setTaskGoal(robot, 'angle_pouring', (2, ), (2, ), '', ())
    createExpression(robot, PointElement('bung', robot, 'bung', position = (0.000000, 0.000000, 0.000000)))
    createExpression(robot, PointElement('cup', robot, 'cup', position = (0.000000, 0.000000, 0.000000)))
    createTask(robot,'position_bung_Z', 'bung', 'cup', 'position', lowerBound = (0.000000, ), upperBound = (0.000000, ))
    setTaskGoal(robot, 'position_bung_Z', (0.000000, ), (0.000000, ), '100', ())
    superviser.push('position_bung_Z')
    createExpression(robot, PointElement('cup', robot, 'cup', position = (0.000000, 0.000000, 0.000000)))
    createExpression(robot, PointElement('bung', robot, 'bung', position = (0.000000, 0.000000, 0.000000)))
    createTask(robot,'position_bung_XY', 'cup', 'bung', 'position', lowerBound = (-0.025000, -0.025000, ), upperBound = (0.025000, 0.025000, ))
    setTaskGoal(robot, 'position_bung_XY', (-0.025000, -0.025000, ), (0.025000, 0.025000, ), '011', ())
    superviser.push('position_bung_XY')
    createExpression(robot, PlaneElement('ground_plane', robot, 'ground', normal = (0.000000, 0.000000, 1.000000),position = (0.000000, 0.000000, 0.000000)))
    createExpression(robot, VersorElement('r_gripper_y', robot, 'r_gripper', versor = (0.000000, 1.000000, 0.000000)))
    createTask(robot,'angle_gripperY_in_ground_plane', 'ground_plane', 'r_gripper_y', 'angle', lowerBound = (0.000000, ), upperBound = (0.000000, ))
    setTaskGoal(robot, 'angle_gripperY_in_ground_plane', (0.000000, ), (0.000000, ), '', ())
    superviser.push('angle_gripperY_in_ground_plane')
    superviser.push('angle_pouring')
    superviser.push('weight')
    superviser.update()

  elif index == 5:
    print "pouring MORE"
    superviser.clear()
    superviser.push('taskcontact')
    superviser.push('taskbase')
    superviser.push('taskJL')
    createExpression(robot, VersorElement('bung_x', robot, 'bung', versor = (1.000000, 0.000000, 0.000000)))
    createExpression(robot, VersorElement('ground_z', robot, 'ground', versor = (0.000000, 0.000000, 1.000000)))
    createTask(robot,'angle_pouring', 'bung_x', 'ground_z', 'angle', lowerBound = (2.007129, ), upperBound = (2.007129, ))
    setTaskGoal(robot, 'angle_pouring', (2.4, ), (2.4, ), '', ())
    createExpression(robot, PointElement('bung', robot, 'bung', position = (0.000000, 0.000000, 0.000000)))
    createExpression(robot, PointElement('cup', robot, 'cup', position = (0.000000, 0.000000, 0.000000)))
    createTask(robot,'position_bung_Z', 'bung', 'cup', 'position', lowerBound = (0.000000, ), upperBound = (0.000000, ))
    setTaskGoal(robot, 'position_bung_Z', (0.0750000, ), (0.07500000, ), '100', ())
    superviser.push('position_bung_Z')
    createExpression(robot, PointElement('cup', robot, 'cup', position = (0.000000, 0.000000, 0.000000)))
    createExpression(robot, PointElement('bung', robot, 'bung', position = (0.000000, 0.000000, 0.000000)))
    createTask(robot,'position_bung_XY', 'cup', 'bung', 'position', lowerBound = (-0.025000, -0.025000, ), upperBound = (0.025000, 0.025000, ))
    setTaskGoal(robot, 'position_bung_XY', (-0.025000, -0.025000, ), (0.025000, 0.025000, ), '011', ())
    superviser.push('position_bung_XY')
    createExpression(robot, PlaneElement('ground_plane', robot, 'ground', normal = (0.000000, 0.000000, 1.000000),position = (0.000000, 0.000000, 0.000000)))
    createExpression(robot, VersorElement('r_gripper_y', robot, 'r_gripper', versor = (0.000000, 1.000000, 0.000000)))
    createTask(robot,'angle_gripperY_in_ground_plane', 'ground_plane', 'r_gripper_y', 'angle', lowerBound = (0.000000, ), upperBound = (0.000000, ))
    setTaskGoal(robot, 'angle_gripperY_in_ground_plane', (0.000000, ), (0.000000, ), '', ())
    superviser.push('angle_pouring')
#    superviser.push('angle_gripperY_in_ground_plane')
    superviser.push('weight')
    superviser.update()


  elif index == 6:
    print "go away"

#    superviser.clear()
#    setTaskGoal(robot, 'angle_pouring', (1.483530, ), (1.483530, ), '', ())
#    superviser.push('taskcontact')
#    superviser.push('taskbase')
#    superviser.push('taskJL')
#    createExpression(robot, VersorElement('bung_x', robot, 'bung', versor = (1.000000, 0.000000, 0.000000)))
#    createExpression(robot, VersorElement('ground_z', robot, 'ground', versor = (0.000000, 0.000000, 1.000000)))
#    createTask(robot,'angle_pouring', 'bung_x', 'ground_z', 'angle', lowerBound = (1.483530, ), upperBound = (1.483530, ))
#    setTaskGoal(robot, 'angle_pouring', (1.483530, ), (1.483530, ), '', ())
#    superviser.push('angle_pouring')
#    createExpression(robot, PlaneElement('ground_plane', robot, 'ground', normal = (0.000000, 0.000000, 1.000000),position = (0.000000, 0.000000, 0.000000)))
#    createExpression(robot, VersorElement('r_gripper_y', robot, 'r_gripper', versor = (0.000000, 1.000000, 0.000000)))
#    createTask(robot,'angle_gripperY_in_ground_plane', 'ground_plane', 'r_gripper_y', 'angle', lowerBound = (0.000000, ), upperBound = (0.000000, ))
#    setTaskGoal(robot, 'angle_gripperY_in_ground_plane', (0.000000, ), (0.000000, ), '', ())
#    superviser.push('angle_gripperY_in_ground_plane')
#    superviser.push('weight')
#    superviser.update()


#  elif index == 7:
    superviser.clear()
    superviser.push('taskcontact')
    superviser.push('taskbase')
    superviser.push('taskJL')
    setTaskGoal(robot, 'taskright-wrist', (), (), '', (0.2, ))
    superviser.push('taskright-wrist')
    superviser.push('weight')
    superviser.update()

  elif index == 8:
    index = -1

  index = index + 1

