#!/usr/bin/env python

# Import ROS modules.
import roslib;

import rospy

from std_srvs.srv import Empty, EmptyResponse

def startDynamicGraphCb(self, req):
  return
#  if not self.manager:
#    return None
#  if self.manager.dynamicGraphStarted:
#    rospy.logerr("dynamic-graph already started")
#    return None

if __name__ == "__main__":
  rospy.init_node("pouet")
  startDynamicGraphSrv = rospy.Service(
			'start_dynamic_graph', Empty,
			lambda req: self.startDynamicGraphCb(req))
#		self.stopDynamicGraphSrv = rospy.Service(
#			'stop_dynamic_graph', Empty,
  rospy.spin()
