#!/usr/bin/env python
import numpy as np
import matplotlib.pyplot as pl
import dynamic_graph as dg
import dynamic_graph.sotcollision as sc
import time
import datetime as dt
import rospy
import csv
import tf

# SolverKine solver and relavant tasks
from dynamic_graph.sot.core import *
from dynamic_graph.sot.dyninv import SolverKine
from dynamic_graph.sot.core.meta_tasks_kine import MetaTaskKine6d
from dynamic_graph.sot.dyninv import TaskInequality, TaskJointLimits

# robot
from dynamic_graph.sot.dynamics.mobile_robot import Ur
from dynamic_graph.sot.core import RobotSimu, FeaturePosition, Task, SOT, FeatureGeneric
from dynamic_graph import plug, writeGraph
from dynamic_graph.ros import Ros
# visualization
from visualization_msgs.msg import MarkerArray,Marker


class RobotCollisionModel:
    def __init__(self):
        # Marker array for link collision model
        self.linkcollisionmodel = MarkerArray()
        # Define marker properties for link 
        self.rcm_pub = rospy.Publisher('collision_model', MarkerArray)
        self.r = rospy.Rate(100) 
        self.nolinks = 5
        self.dt = 0
        self.buildurrobot()

    def buildurrobot(self):
        # collision components
        self.a = sc.SotCollision("sc")

        # create links for collision check
        self.a.createcollisionlink("wall1",(0.08,0.01,0,0,0.0,0.0,0,0))
        self.a.createcollisionlink("wall2",(0.08,0.04,0,0,0,0,0,0))
        self.a.createcollisionlink("wall3",(0.09,0.15,0.2,0.0,0.0,0,0.0,0))
        self.a.createcollisionlink("platform1",(0.07,0.14,0.2,0.0,0.0,-3.1416,1.5706,-3.1416))
        self.a.createcollisionlink("platform2",(0.05,0.03,0.0,0.093,0,-3.14,0,-3.14))

        # plug joint position and joint jacobian to collision checker entity
        self.a.wall1.value = ((1,0,0,0),(0,1,0,0),(0,0,1,0),(0,0,0,1))
        self.a.wall2.value = ((1,0,0,0),(0,1,0,0),(0,0,1,0),(0,0,0,1))
        self.a.wall3.value = ((1,0,0,0),(0,1,0,0),(0,0,1,0),(0,0,0,1))
        self.a.platform1.value = ((1,0,0,0),(0,1,0,0),(0,0,1,0),(0,0,0,1))
        self.a.platform2.value = ((1,0,0,0),(0,1,0,0),(0,0,1,0),(0,0,0,1))

        self.a.Jwall1.value = 
        self.a.Jwall2.value = 
        self.a.Jwall3.value = 
        self.a.Jplatform1.value = 
        self.a.Jplatform2.value = 



    def publish(self):        
        self.robot.dynamic.Jwrist_2_joint.recompute(self.dt)
        self.dt = self.dt+1
        self.getstate()
        self.rcm_pub.publish(self.linkcollisionmodel)
        self.r.sleep()

    def getstate(self):
        state = rcm.a.getfclstate()
        #print state
        #a = self.robot.dynamic.wrist_2_joint.value
        for i in range(7):
            self.link = Marker() 
            self.link.type = visualization_msgs.Marker.CUBE
            #self.link.mesh_resource = "package://sot-collision/mesh/capsule/capsule.dae"
            self.link.header.frame_id = "/odom"
            self.link.header.stamp = rospy.Time.now()
            #self.link.ns = "basicshapes"
            self.link.id = i
            self.link.action = self.link.ADD
            #print state
            self.link.pose.position.x = state[i][0];
            self.link.pose.position.y = state[i][1];
            self.link.pose.position.z = state[i][2];
            self.link.pose.orientation.x = state[i][3]
            self.link.pose.orientation.y = state[i][4]
            self.link.pose.orientation.z = state[i][5]
            self.link.pose.orientation.w = state[i][6]
            self.link.scale.x = state[i][7];
            self.link.scale.y = state[i][7];
            self.link.scale.z = state[i][8];
            self.link.color.r = 0.0;
            self.link.color.g = 1.0;
            self.link.color.b = 0.0;
            self.link.color.a = 1.0;
            #pself.link.pose.orientation
            #self.link.lifetime =  rospy.Duration();
            self.linkcollisionmodel.markers.append(self.link)

if __name__ == '__main__' :
    rospy.init_node('collision_checker')
    rcm = RobotCollisionModel()
    i = 0
    dt = 0.01
    #while i < 1000: 
    #	rcm.publish()
