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
from dynamic_graph.sot.pr2.robot import Pr2

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
        self.nolinks = 2
        self.dt = 0
        self.buildurrobot()

    def buildurrobot(self):
        # collision components

        self.robot = Pr2(name = 'robot', device=RobotSimu('Pr2'))
        self.dimension = self.robot.dynamic.getDimension()
        plug(self.robot.device.state, self.robot.dynamic.position)
        self.ros = Ros(self.robot)

        self.a = sc.SotCollision("sc")
        # create links for collision check
        #self.a.createcollisionlink("wall1","box","external",(2.04,0.015,0.3,0,0,0.0,0.0,0,0))
        self.a.createcollisionlink("base_link","box","internal",(0.65,0.65,0.25,0.0,0.0,0.175,0,0,0))
        self.a.createcollisionlink("shoulder_right_link","box","internal",(0.20,0.23,0.65,0.0,0.0,-0.13,0,0,0))
        self.a.createcollisionlink("shoulder_left_link","box","internal",(0.20,0.23,0.65,0.0,0.0,-0.13,0,0,0))

        self.a.createcollisionlink("upper_left_arm","box","internal",(0.42,0.13,0.15,0.26,0.0,-0.0,0,0,0))
        self.a.createcollisionlink("upper_right_arm","box","internal",(0.42,0.13,0.15,0.26,0.0,-0.0,0,0,0))

        self.a.createcollisionlink("upper_left_forearm","box","internal",(0.37,0.09,0.09,0.28,0.0,-0.0,0,0,0))
        self.a.createcollisionlink("upper_right_forearm","box","internal",(0.37,0.09,0.09,0.28,0.0,-0.0,0,0,0))
        self.a.createcollisionlink("upper_right_wrist","box","internal",(0.1,0.09,0.05,0.1,0.0,-0.0,0,0,0))
        self.a.createcollisionlink("upper_left_wrist","box","internal",(0.1,0.09,0.05,0.1,0.0,-0.0,0,0,0))



        #self.a.createcollisionlink("platform1","box","external",(0.5,0.8,0.11,0.0,0.0,0.0,0,0.0,0))

        # plug joint position and joint jacobian to collision checker entity
        #self.a.wall1.value = ((1,0,0,0),(0,1,0,1.22),(0,0,1,0.12),(0,0,0,1))
 
        
        #self.a.platform1.value = ((1,0,0,12),(0,1,0,0.82),(0,0,1,0.025),(0,0,0,1))
        #self.a.platform2.value = ((1,0,0,0.77),(0,1,0,-0.82),(0,0,1,0.025),(0,0,0,1))
        #print self.robot.dynamic
        plug(self.robot.dynamic.base_joint,self.a.base_link)
        plug(self.robot.dynamic.r_shoulder_pan_joint,self.a.shoulder_right_link)
        plug(self.robot.dynamic.l_shoulder_pan_joint,self.a.shoulder_left_link)

        plug(self.robot.dynamic.l_upper_arm_roll_joint,self.a.upper_left_arm)
        plug(self.robot.dynamic.r_upper_arm_roll_joint,self.a.upper_right_arm)


        plug(self.robot.dynamic.l_forearm_roll_joint,self.a.upper_left_forearm)
        plug(self.robot.dynamic.r_forearm_roll_joint,self.a.upper_right_forearm)
        
        plug(self.robot.dynamic.r_wrist_roll_joint,self.a.upper_right_wrist)
        plug(self.robot.dynamic.l_wrist_roll_joint,self.a.upper_left_wrist)

        #plug(self.robot.dynamic.base_joint,self.a.base_link_2)
        #plug(self.robot.dynamic.base_joint,self.a.base_link_3)
        #plug(self.robot.dynamic.base_joint,self.a.base_link_4)
        

        #l_shoulder_pan_joint
        #l_shoulder_lift_joint
        #r_shoulder_pan_joint
        #r_shoulder_lift_joint
        
        #self.a.Jwall1.value = ((1,0,0,0),(0,1,0,0),(0,0,1,0),(0,0,0,1))
        #self.a.Jwall2.value = ((1,0,0,0),(0,1,0,0),(0,0,1,0),(0,0,0,1))
        #self.a.Jwall3.value = ((1,0,0,0),(0,1,0,0),(0,0,1,0),(0,0,0,1))
        #self.a.Jplatform1.value = ((1,0,0,0),(0,1,0,0),(0,0,1,0),(0,0,0,1))
        #self.a.Jplatform2.value = ((1,0,0,0),(0,1,0,0),(0,0,1,0),(0,0,0,1))
        plug(self.robot.dynamic.Jbase_joint,self.a.Jbase_link)
        plug(self.robot.dynamic.Jr_shoulder_pan_joint,self.a.Jshoulder_right_link)
        plug(self.robot.dynamic.Jl_shoulder_pan_joint,self.a.Jshoulder_left_link)
        
        plug(self.robot.dynamic.Jl_upper_arm_roll_joint,self.a.Jupper_left_arm)
        plug(self.robot.dynamic.Jr_upper_arm_roll_joint,self.a.Jupper_right_arm)


        plug(self.robot.dynamic.Jl_forearm_roll_joint,self.a.Jupper_left_forearm)
        plug(self.robot.dynamic.Jr_forearm_roll_joint,self.a.Jupper_right_forearm)

        plug(self.robot.dynamic.Jr_wrist_roll_joint,self.a.Jupper_right_wrist)
        plug(self.robot.dynamic.Jl_wrist_roll_joint,self.a.Jupper_left_wrist)
        
        #plug(self.robot.dynamic.Jbase_joint,self.a.Jbase_link_4)


        self.a.createcollisionpair("base_link","upper_left_arm")
        self.a.createcollisionpair("base_link","upper_right_arm")
        self.a.createcollisionpair("base_link","upper_left_forearm")
        self.a.createcollisionpair("base_link","upper_right_forearm")
        self.a.createcollisionpair("shoulder_left_link","upper_left_forearm")
        self.a.createcollisionpair("shoulder_right_link","upper_right_forearm")
        
        self.a.createcollisionpair("upper_left_forearm","upper_right_forearm")
        self.a.createcollisionpair("upper_left_arm","upper_right_arm")

        self.a.createcollisionpair("upper_left_arm","upper_right_forearm")
        self.a.createcollisionpair("upper_left_forearm","upper_right_arm")  
        
        self.a.createcollisionpair("upper_left_wrist","upper_right_wrist")  
        self.a.createcollisionpair("upper_left_forearm","upper_right_wrist")
        self.a.createcollisionpair("upper_left_arm","upper_right_wrist")  
        self.a.createcollisionpair("upper_right_wrist","upper_left_wrist")  
        self.a.createcollisionpair("upper_right_forearm","upper_left_wrist")
        self.a.createcollisionpair("upper_right_arm","upper_left_wrist")  


# joint limits
        self.robot.dynamic.upperJl.recompute(0)
        self.robot.dynamic.lowerJl.recompute(0)
        self.taskjl = TaskJointLimits('taskJL')
        plug(self.robot.dynamic.position,self.taskjl.position)
        self.taskjl.controlGain.value = 5
        self.taskjl.referenceInf.value = self.robot.dynamic.lowerJl.value
        #print self.robot.dynamic.lowerJl.value
        self.taskjl.referenceSup.value = self.robot.dynamic.upperJl.value
        #print self.robot.dynamic.upperJl.value
        self.taskjl.dt.value = 1



#torsotask
        self.task_torso_metakine=MetaTaskKine6d('task_torso_metakine',self.robot.dynamic,'torso_lift_joint','torso_lift_joint')
        goal_torso = ((1.,0,0,-0.05),(0,1.,0,-0.0),(0,0,1.,0.790675),(0,0,0,1.),)
        self.task_torso_metakine.feature.frame('desired')
#task_torso_metakine.feature.selec.value = '000100'#RzRyRxTzTyTx
        self.task_torso_metakine.gain.setConstant(10)
        self.task_torso_metakine.featureDes.position.value = goal_torso 

        goal = ((1.,0,0,0.0),
            (0,1.,0,0),
            (0,0,1.,0),
            (0,0,0,1.),)
        self.task_waist_metakine=MetaTaskKine6d('task_waist_metakine',self.robot.dynamic,'base_joint','base_joint')
        #task_waist_metakine.opmodif = goal
        #task_waist_metakine.feature.frame('desired')
        self.task_waist_metakine.featureDes.position.value = goal
        self.task_waist_metakine.gain.setConstant(1) 

        self.task_l_wrist_metakine=MetaTaskKine6d('task_l_wrist_metakine',self.robot.dynamic,'l_wrist_roll_joint','l_wrist_roll_joint')
        #self.goal_l_wrist = ((1.,0,0,0.748),(0,1.,0,-0.246),(0,0,1.,0.639),(0,0,0,1.),)
        self.goal_l_wrist = ((1.,0,0,0.649),(0,1.,0,-0.042),(0,0,1.,0.845),(0,0,0,1.),)
        #self.goal_l_wrist = ((1.,0,0,0.486),(0,1.,0,-0.251),(0,0,1.,0.826),(0,0,0,1.),)        
        self.task_l_wrist_metakine.feature.frame('desired')
        self.task_l_wrist_metakine.feature.selec.value = '000111'#RzRyRxTzTyTx
        self.task_l_wrist_metakine.gain.setConstant(6)
        self.task_l_wrist_metakine.featureDes.position.value = self.goal_l_wrist 

        self.task_r_wrist_metakine=MetaTaskKine6d('task_r_wrist_metakine',self.robot.dynamic,'r_wrist_roll_joint','r_wrist_roll_joint')
        #self.goal_l_wrist = ((1.,0,0,0.748),(0,1.,0,-0.246),(0,0,1.,0.639),(0,0,0,1.),)
        #self.goal_r_wrist = ((1.,0,0,0.455),(0,1.,0,-0.835),(0,0,1.,0.780),(0,0,0,1.),)    
        self.goal_r_wrist = ((1.,0,0,0.590),(0,1.,0,-0.188),(0,0,1.,1.107 ),(0,0,0,1.),)  
        self.task_r_wrist_metakine.feature.frame('desired')
        self.task_r_wrist_metakine.feature.selec.value = '000100'#RzRyRxTzTyTx
        self.task_r_wrist_metakine.gain.setConstant(6)
        self.task_r_wrist_metakine.featureDes.position.value = self.goal_r_wrist 
        
        

        self.task_collision_avoidance=TaskInequality('taskcollision')
        self.collision_feature = FeatureGeneric('collisionfeature')
        plug(self.a.collisionJacobian,self.collision_feature.jacobianIN)
        plug(self.a.collisionDistance,self.collision_feature.errorIN)
        self.task_collision_avoidance.add(self.collision_feature.name)
        self.task_collision_avoidance.referenceInf.value = (0.1,0.1,0.1,0.1,0.1,0.1,0.1,0.1,0.1,0.1,0.1,0.1,0.1,0.1,0.1,0.1)    
        self.task_collision_avoidance.referenceSup.value = (2e10,2e10,2e10,2e10,2e10,2e10,2e10,2e10,2e10,2e10,2e10,2e10,2e10,2e10,2e10,2e10)  
        self.task_collision_avoidance.dt.value=1
        self.task_collision_avoidance.controlGain.value=50.0

        
        self.solver = SolverKine('sot')
        self.solver.setSize (self.dimension)
        self.solver.damping.value =3e-2
        self.solver.push (self.taskjl.name) 
        self.solver.push (self.task_collision_avoidance.name)        
        self.solver.push (self.task_waist_metakine.task.name)
        self.solver.push (self.task_torso_metakine.task.name)
        self.solver.push (self.task_r_wrist_metakine.task.name)
        self.solver.push (self.task_l_wrist_metakine.task.name)
        #self.solver.push (self.task_collision_avoidance.name)
        #self.solver.push (self.task_wrist_metakine.task.name)
        #self.solver.damping.value =3e-2
        plug (self.solver.control, self.robot.device.control)
        
        
        

    def publish(self):        
        #self.robot.dynamic.Jwrist_2_joint.recompute(self.dt)
        #self.a.collisionDistance.recompute(self.dt)
        #print self.a.collisionDistance.value
        #self.a.collisionJacobian.recompute(self.dt)
        #print self.a.collisionDistance.value
        self.dt = self.dt+1
        #self.r.sleep()
        #print self.robot.dynamic.base_joint.value
        tstart = dt.datetime.now()
        self.robot.device.increment(0.01)
        self.getstate()
        self.rcm_pub.publish(self.linkcollisionmodel)
        tend = dt.datetime.now()
        print (tend-tstart).microseconds
        self.robot.dynamic.l_upper_arm_roll_joint.recompute(self.dt)
        #print self.robot.dynamic.l_upper_arm_roll_joint.value
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
