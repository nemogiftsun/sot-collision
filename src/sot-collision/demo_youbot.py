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
from dynamic_graph.sot.youbot.robot import youbot

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
        self.r = rospy.Rate(1) 
        self.nolinks = 5
        self.dt = 0
        self.buildurrobot()

    def buildurrobot(self):
        # collision components

        self.robot = youbot(name = 'robot', device=RobotSimu('youbot'))
        self.dimension = self.robot.dynamic.getDimension()
        plug(self.robot.device.state, self.robot.dynamic.position)



        self.a = sc.SotCollision("sc")
        # create links for collision check
        self.a.createcollisionlink("wall1","box","external",(2.04,0.015,0.3,0,0,0.0,0.0,0,0))
        self.a.createcollisionlink("wall2","box","external",(0.015,-2.44,0.3,0,0,0,0,0,0))
        self.a.createcollisionlink("wall3","box","external",(2.04,0.015,0.3,0,0,0.0,0.0,0,0))
        self.a.createcollisionlink("platform1","box","external",(0.5,0.8,0.11,0.0,0.0,0.0,0,0.0,0))
        self.a.createcollisionlink("platform2","box","external",(0.5,0.8,0.11,0.0,0.0,0.0,0,0.0,0))
        #self.a.createcollisionlink("base_link","box","internal",(0.64,0.4,0.11,0.0,0.0,0,0,0,0))
        self.a.createcollisionlink("base_link_1","box","internal",(0.1,0.4,0.11,0.26,0.0,0,0,0,0))
        self.a.createcollisionlink("base_link_2","box","internal",(0.1,0.4,0.11,-0.26,0.0,0,0,0,0))
        self.a.createcollisionlink("base_link_3","box","internal",(0.44,0.1,0.11,0.0,0.15,0,0,0,0))
        self.a.createcollisionlink("base_link_4","box","internal",(0.44,0.1,0.11,0.0,-0.15,0,0,0,0))

        # plug joint position and joint jacobian to collision checker entity
        self.a.wall1.value = ((1,0,0,0),(0,1,0,1.22),(0,0,1,0.12),(0,0,0,1))
        self.a.wall2.value = ((1,0,0,1.02),(0,1,0,0),(0,0,0,0.12),(0,0,0,1))
        self.a.wall3.value = ((1,0,0,0),(0,1,0,-1.22),(0,0,1,0.12),(0,0,0,1))
        
        self.a.platform1.value = ((1,0,0,0.77),(0,1,0,0.82),(0,0,1,0.025),(0,0,0,1))
        self.a.platform2.value = ((1,0,0,0.77),(0,1,0,-0.82),(0,0,1,0.025),(0,0,0,1))

        #plug(self.robot.dynamic.base_joint,self.a.base_link)
        plug(self.robot.dynamic.base_joint,self.a.base_link_1)
        plug(self.robot.dynamic.base_joint,self.a.base_link_2)
        plug(self.robot.dynamic.base_joint,self.a.base_link_3)
        plug(self.robot.dynamic.base_joint,self.a.base_link_4)

        
        self.a.Jwall1.value = ((1,0,0,0),(0,1,0,0),(0,0,1,0),(0,0,0,1))
        self.a.Jwall2.value = ((1,0,0,0),(0,1,0,0),(0,0,1,0),(0,0,0,1))
        self.a.Jwall3.value = ((1,0,0,0),(0,1,0,0),(0,0,1,0),(0,0,0,1))
        self.a.Jplatform1.value = ((1,0,0,0),(0,1,0,0),(0,0,1,0),(0,0,0,1))
        self.a.Jplatform2.value = ((1,0,0,0),(0,1,0,0),(0,0,1,0),(0,0,0,1))
        plug(self.robot.dynamic.Jbase_joint,self.a.Jbase_link_1)
        plug(self.robot.dynamic.Jbase_joint,self.a.Jbase_link_2)
        plug(self.robot.dynamic.Jbase_joint,self.a.Jbase_link_3)
        plug(self.robot.dynamic.Jbase_joint,self.a.Jbase_link_4)


        self.a.createcollisionpair("base_link_1","platform1")
        self.a.createcollisionpair("base_link_1","platform2")
        self.a.createcollisionpair("base_link_2","platform1")
        self.a.createcollisionpair("base_link_2","platform2")
        self.a.createcollisionpair("base_link_3","platform1")
        self.a.createcollisionpair("base_link_3","platform2")
        self.a.createcollisionpair("base_link_4","platform1")
        self.a.createcollisionpair("base_link_4","platform2")

        goal = ((1.,0,0,0.5),
            (0,1.,0,0),
            (0,0,1.,0),
            (0,0,0,1.),)
        self.task_waist_metakine=MetaTaskKine6d('task_waist_metakine',self.robot.dynamic,'base_joint','base_joint')
        #task_waist_metakine.opmodif = goal
        #task_waist_metakine.feature.frame('desired')
        self.task_waist_metakine.featureDes.position.value = goal
        self.task_waist_metakine.gain.setConstant(1) 
        
        self.solver = SolverKine('sot')
        self.solver.setSize (self.dimension)  
        #self.solver.push (self.task_waist_metakine.task.name)
        #self.solver.push (self.task_collision_avoidance.name)
        #self.solver.push (self.task_wrist_metakine.task.name)
        #self.solver.damping.value =3e-2
        plug (self.solver.control, self.robot.device.control)
        

    def publish(self):        
        #self.robot.dynamic.Jwrist_2_joint.recompute(self.dt)
        self.a.collisionDistance.recompute(self.dt)
        self.a.collisionJacobian.recompute(self.dt)
        #print self.a.collisionDistance.value
        self.dt = self.dt+1
        self.getstate()
        self.rcm_pub.publish(self.linkcollisionmodel)
        self.r.sleep()
        self.robot.device.increment(0.01)

    def getstate(self):
        state = rcm.a.getfclstate()
        #print state
        #a = self.robot.dynamic.wrist_2_joint.value
        for i in range(9):
            self.link = Marker() 
            self.link.type = Marker().CUBE
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
            self.link.scale.y = state[i][8];
            self.link.scale.z = state[i][9];
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
    while i < 1000: 
    	rcm.publish()
    #a = sc.SotCollision("sc")
    #a.DoExperimentCollisionCheck(1)
    #print a.getdistance()
    #while i < 1000: 
    #	rcm.publish()
