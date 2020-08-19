#! /usr/bin/env python
"""Publishes joint trajectory to move robot to given pose"""

import rospy
from std_msgs.msg import Float64
from trajectory_msgs.msg import JointTrajectory
from trajectory_msgs.msg import JointTrajectoryPoint
from std_srvs.srv import Empty
import argparse
import time

def movePTUPosition (jointcmds,rname):
  module = ['pan','tilt']
  for m in range(2):
    topic_name = '/' + rname + '/' + 'ptu_' + module[m] + '_position_controller/command'
    pub = rospy.Publisher(topic_name, Float64, queue_size=10)
    rate = rospy.Rate(100)
    count = 0
    while (count < 10):
        pub.publish(jointcmds[m])
        count = count + 1
        rate.sleep()

def moveJointPosition (jointcmds,rname,prefix,nbJoints):
  for i in range(nbJoints):
    topic_name = '/' + rname + '/' + prefix + '_joint_' + str(i+1) + '_position_controller/command'
    pub = rospy.Publisher(topic_name, Float64, queue_size=10)
    rate = rospy.Rate(100)
    count = 0
    while (count < 10):
      pub.publish(jointcmds[i])
      count = count + 1
      rate.sleep()

def moveFingersPosition (jointcmds,rname,prefix,nbJoints):
  for i in range(nbJoints):
    topic_name = '/' + rname + '/' + prefix + '_finger_' + str(i+1) + '_position_controller/command'
    pub = rospy.Publisher(topic_name, Float64, queue_size=10)
    rate = rospy.Rate(100)
    count = 0
    while (count < 10):
      pub.publish(jointcmds[i])
      count = count + 1
      rate.sleep()

def moveJointTrajectory (jointcmds,rname,prefix,nbJoints):
  topic_name = '/' + rname + '/' + prefix + '_effort_joint_trajectory_controller/command'
  pub = rospy.Publisher(topic_name, JointTrajectory, queue_size=1)
  jointCmd = JointTrajectory()  
  point = JointTrajectoryPoint()
  jointCmd.header.stamp = rospy.Time.now() + rospy.Duration.from_sec(0.0);  
  point.time_from_start = rospy.Duration.from_sec(5.0)
  for i in range(0, nbJoints):
    jointCmd.joint_names.append(prefix +'_joint_'+str(i+1))
    point.positions.append(jointcmds[i])
    point.velocities.append(0)
    point.accelerations.append(0)
    point.effort.append(0) 
  jointCmd.points.append(point)
  rate = rospy.Rate(100)
  count = 0
  while (count < 50):
    pub.publish(jointCmd)
    count = count + 1
    rate.sleep()     

def moveFingersTrajectory (jointcmds,rname,prefix,nbJoints):
  topic_name = '/' + rname + '/' + prefix + '_effort_finger_trajectory_controller/command'
  pub = rospy.Publisher(topic_name, JointTrajectory, queue_size=1)  
  jointCmd = JointTrajectory()  
  point = JointTrajectoryPoint()
  jointCmd.header.stamp = rospy.Time.now() + rospy.Duration.from_sec(0.0);  
  point.time_from_start = rospy.Duration.from_sec(5.0)
  for i in range(0, nbJoints):
    jointCmd.joint_names.append(prefix +'_joint_finger_'+str(i+1))
    point.positions.append(jointcmds[i])
    point.velocities.append(0)
    point.accelerations.append(0)
    point.effort.append(0) 
  jointCmd.points.append(point)
  rate = rospy.Rate(100)
  count = 0
  while (count < 500):
    pub.publish(jointCmd)
    count = count + 1
    rate.sleep()     

if __name__ == '__main__':
  try:    
    rospy.init_node('move_robot_using_trajectory_msg')		
    #allow gazebo to launch
    time.sleep(5)

    # Unpause the physics
    rospy.wait_for_service('/gazebo/unpause_physics')
    unpause_gazebo = rospy.ServiceProxy('/gazebo/unpause_physics', Empty)
    resp = unpause_gazebo()

    # left arm
    moveJointPosition ([1.3,1.07,4.2,-1.6,0.0,0.0],'robot','left',6)
    moveFingersPosition ([1,1,1],'robot','left',3)

	# right arm
    moveJointPosition ([-1.3,5.21,2.08,1.6,0.0,0.0],'robot','right',6)
    moveFingersPosition ([1,1,1],'robot','right',3)

    # FLIR PTU
    movePTUPosition ([0.0,0.0],'robot')

  except rospy.ROSInterruptException:
    print "program interrupted before completion"
