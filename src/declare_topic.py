#!/usr/bin/env python
# license removed for brevity
import rospy
import sys
from std_msgs.msg import Float64

"""

"""
def declare_topic():
    rospy.init_node('declare_topic', anonymous=True)
    names = rospy.get_param("names");
    nameList = names.split(' ')
    print "names are ", names

    pubs = []
    for name in nameList:
      print "name is ", name 
      pubs.append(rospy.Publisher(name, Float64))


    r = rospy.Rate(10) # 10hz
    numrun = 0
    while not rospy.is_shutdown() and numrun < 100:
      for p in pubs:
        p.publish(0.0)
      numrun = numrun +1
      r.sleep()

if __name__ == '__main__':
    try:
      declare_topic()
    except rospy.ROSInterruptException: pass
