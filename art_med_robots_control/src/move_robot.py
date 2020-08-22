#! /usr/bin/env python
"""Publishes joint trajectory to move robot to given pose"""

import rospy
from std_msgs.msg import Float64
from trajectory_msgs.msg import JointTrajectory
from trajectory_msgs.msg import JointTrajectoryPoint
from std_srvs.srv import Empty
import argparse
import time
import actionlib
from control_msgs.msg import *
from trajectory_msgs.msg import *

def movePTUPosition (jointcmds,rname):
  module = ['pan','tilt']
  for m in range(2):
    topic_name = 'ptu_' + module[m] + '_position_controller/command'
    pub = rospy.Publisher(topic_name, Float64, queue_size=10)
    rate = rospy.Rate(100)
    count = 0
    while (count < 50):
        pub.publish(jointcmds[m])
        count = count + 1
        rate.sleep()


def moveTrajectory (jointcmds,controller_ns,joint_ns):
    try:
      client = actionlib.SimpleActionClient(controller_ns+'/follow_joint_trajectory',
                                              FollowJointTrajectoryAction)
      client.wait_for_server()
    except KeyboardInterrupt: 
        rospy.signal_shutdown("KeyboardInterrupt")
        raise    

    g = FollowJointTrajectoryGoal()
    g.trajectory = JointTrajectory()  
    g.trajectory.header.stamp = rospy.Time.now() + rospy.Duration.from_sec(0.0);  

    point = JointTrajectoryPoint()  
    point.time_from_start = rospy.Duration.from_sec(3.0)
    for i in range(len(jointcmds)):
        g.trajectory.joint_names.append(joint_ns+str(i+1))
        point.positions.append(jointcmds[i])
        point.velocities.append(0)
        point.accelerations.append(0)
        point.effort.append(0)
        
    g.trajectory.points.append(point)
    client.send_goal(g)
    try:
        client.wait_for_result()
    except KeyboardInterrupt:
        client.cancel_goal()
        raise
    rospy.loginfo("Done! ")
        


    
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
    moveTrajectory([1.0,1.57,4.71,-1.0,0.0,0.0],
                     'left_position_joint_trajectory_controller',
                     'left_joint_')
    moveTrajectory([0,0,0],
                     'left_effort_finger_trajectory_controller',
                     'left_joint_finger_')
    
	# right arm
    moveTrajectory([-1.0,4.71,1.57,1.0,0.0,0.0],
                     'right_position_joint_trajectory_controller',
                     'right_joint_')
    moveTrajectory([0,0,0],
                     'right_effort_finger_trajectory_controller',
                     'right_joint_finger_')
    
    # FLIR PTU
    ## movePTUPosition ([0.0,0.0],'robot')

  except rospy.ROSInterruptException:
    print "program interrupted before completion"
