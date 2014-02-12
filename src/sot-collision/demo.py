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


'''
##collision_features = FeatureGeneric('collision_feature')
rpy="0.0 0.0 2.3561944875" xyz="0.0 0.0 0.003"
rpy="0.0 0.0 0.0" xyz="0.0 0.0 0.089159"
rpy="0.0 1.570796325 0.0" xyz="0.0 0.13585 0.0"
rpy="0.0 0.0 0.0" xyz="0.0 -0.1197 0.425"
rpy="0.0 1.570796325 0.0" xyz="0.0 0.0 0.39225"
rpy="0.0 0.0 0.0" xyz="0.0 0.093 0.0"
rpy="0.0 0.0 0.0" xyz="0.0 0.0 0.09465"
'''
# visualization markers
from visualization_msgs.msg import MarkerArray,Marker
'''
# define inverted pendulum
a = sc.SotCollision("sc")
a.DoCapsuleCollisionCheck(1)
    print a.getcheck()
    print a.getdistance()
    print a.gettime()  
    #t0 = dt.datetime.now()  
    #time.sleep(0.9)
    #t1 = dt.datetime.now()
    #print (t1-t0).microseconds
'''
#For frame [/collision_link_4]: Fixed Frame [/my_frame] does not exist

class RobotCollisionModel:
    def __init__(self):
        # Marker array for link collision model
        self.linkcollisionmodel = MarkerArray()
        # Define marker properties for link 

        self.rcm_pub = rospy.Publisher('collision_model', MarkerArray)
        self.r = rospy.Rate(100) 
        self.nolinks = 6
        self.dt = 0
        self.buildurrobot()
        

    def buildurrobot(self):
        self.robot = Ur('ur5', device=RobotSimu('ur5'))
        robot_dimension = self.robot.dynamic.getDimension() 
        self.robot.device.resize (robot_dimension)

        # Ros binding
        # roscore must be running
        ros = Ros(self.robot)

        # collision components
        self.a = sc.SotCollision("sc")

        #self.a.createlinkmodel(((0,0.08,0.01,0,0,0.0,0.0,0,0),(1,0.08,0.04,0,0,0,0,0,0),(2,0.09,0.15,0.2,0.0,0.0,0,1.57,0),(3,0.07,0.14,0.2,0.0,0.0,-3.1416,1.5706,-3.1416),(4,0.05,0.03,0.0,0.093,0,-3.14,0,-3.14),(5,0.057,0.02,0.0,0,-0.095,-3.14,0,-3.14),(6,0.04,0.01,0.0,0.065,0,1.57,0,0)))

        # create links for collision check
        self.a.createcollisionlink("base_link","capsule","internal",(0.08,0.01,0,0,0,0.0,0.0,0,0))
        self.a.createcollisionlink("shoulder_pan_link","capsule","internal",(0.08,0.04,0,0,0,0,0,0,0))
        self.a.createcollisionlink("shoulder_lift_link","capsule","internal",(0.09,0.15,0,0.2,0.0,0.0,0,1.57,0))
        self.a.createcollisionlink("elbow_link","capsule","internal",(0.07,0.14,0,0.2,0.0,0.0,-3.1416,1.5706,-3.1416))
        self.a.createcollisionlink("wrist_1_link","capsule","internal",(0.05,0.03,0,0.0,0.093,0,-3.14,0,-3.14))
        self.a.createcollisionlink("wrist_2_link","capsule","internal",(0.057,0.02,0,0.0,0,-0.095,-3.14,0,-3.14))
        self.a.createcollisionlink("wrist_3_link","capsule","internal",(0.04,0.01,0,0.0,0.065,0,1.57,0,0))

        # plug joint position and joint jacobian to collision checker entity
        plug (self.robot.dynamic.base_joint,self.a.base_link)
        plug (self.robot.dynamic.shoulder_pan_joint,self.a.shoulder_pan_link)
        plug ( self.robot.dynamic.shoulder_lift_joint,self.a.shoulder_lift_link)
        plug ( self.robot.dynamic.elbow_joint,self.a.elbow_link)
        plug ( self.robot.dynamic.wrist_1_joint,self.a.wrist_1_link)
        plug (self.robot.dynamic.wrist_2_joint, self.a.wrist_2_link)
        plug (self.robot.dynamic.wrist_3_joint, self.a.wrist_3_link)
        
        plug (self.robot.dynamic.Jbase_joint,self.a.Jbase_link)
        plug (self.robot.dynamic.Jshoulder_pan_joint,self.a.Jshoulder_pan_link)
        plug ( self.robot.dynamic.Jshoulder_lift_joint,self.a.Jshoulder_lift_link)
        plug ( self.robot.dynamic.Jelbow_joint,self.a.Jelbow_link)
        plug ( self.robot.dynamic.Jwrist_1_joint,self.a.Jwrist_1_link)
        plug (self.robot.dynamic.Jwrist_2_joint, self.a.Jwrist_2_link)
        plug (self.robot.dynamic.Jwrist_3_joint, self.a.Jwrist_3_link)
        

        # create collision pairs       
        # base link
        self.a.createcollisionpair("base_link","shoulder_lift_link")
        self.a.createcollisionpair("base_link","elbow_link")
        self.a.createcollisionpair("base_link","wrist_1_link")
        self.a.createcollisionpair("base_link","wrist_2_link")
        self.a.createcollisionpair("base_link","wrist_3_link")
        # shoulder pan link
        self.a.createcollisionpair("shoulder_pan_link","wrist_1_link")
        self.a.createcollisionpair("shoulder_pan_link","wrist_2_link")
        self.a.createcollisionpair("shoulder_pan_link","wrist_3_link")
        self.a.createcollisionpair("shoulder_pan_link","elbow_link")    
        # shoulder lift link
        self.a.createcollisionpair("shoulder_lift_link","wrist_1_link")
        self.a.createcollisionpair("shoulder_lift_link","wrist_2_link")
        self.a.createcollisionpair("shoulder_lift_link","wrist_3_link")
        # shoulder elbow link
        self.a.createcollisionpair("elbow_link","wrist_2_link")
        self.a.createcollisionpair("elbow_link","wrist_3_link")
        # shoulder wrist1 link
        self.a.createcollisionpair("wrist_1_link","wrist_3_link")


######### task description for old type of solver############
        # Create task for the waist
        robot_pose = ((1.,0,0,0),
            (0,1.,0,0),
            (0,0,1.,0),
            (0,0,0,1.),)	
        feature_base = FeaturePosition ('position_base', self.robot.dynamic.base_joint,self.robot.dynamic.Jbase_joint, robot_pose)
        task_base = Task ('waist_task')
        task_base.controlGain.value = 100.
        task_base.add (feature_base.name)

        # Create task for the wrist3
        I4 =   ((1.,0,0,0.321),
            (0,1.,0,0.109),
            (0,0,1.,0.848),
            (0,0,0,1.),)
        feature_wrist = FeaturePosition ('position_wrist', self.robot.dynamic.wrist_3_joint, self.robot.dynamic.Jwrist_3_joint, I4)
        task_wrist = Task ('wrist_task')
        task_wrist.controlGain.value = 1
        task_wrist.add (feature_wrist.name)

######### task description for new type of solver############
        # waist position task
        goal = ((1.,0,0,0),
            (0,1.,0,0),
            (0,0,1.,0),
            (0,0,0,1.),)
        self.task_waist_metakine=MetaTaskKine6d('task_waist_metakine',self.robot.dynamic,'base_joint','base_joint')
        #task_waist_metakine.opmodif = goal
        #task_waist_metakine.feature.frame('desired')
        self.task_waist_metakine.featureDes.position.value = goal
        self.task_waist_metakine.gain.setConstant(100) 
        # wrist_3 task
        goal=   ((1.,0,0,0.321),
            (0,1.,0,0.109),
            (0,0,1.,0.848),
            (0,0,0,1.),)
        self.task_wrist_metakine=MetaTaskKine6d('task_wrist_metakine',self.robot.dynamic,'wrist_3_joint','wrist_3_joint')
        #task_wrist_metakine.opmodif = goal
        #task_wrist_metakine.feature.frame('desired')
        self.task_wrist_metakine.featureDes.position.value = goal
        self.task_wrist_metakine.gain.setConstant(1) 

######### task description for collision task   ############
        
        self.task_collision_avoidance=TaskInequality('taskcollision')
        self.collision_feature = FeatureGeneric('collisionfeature')
        plug(self.a.collisionJacobian,self.collision_feature.jacobianIN)
        plug(self.a.collisionDistance,self.collision_feature.errorIN)

        #self.a.collisionDistance

        self.task_collision_avoidance.add(self.collision_feature.name)
        self.task_collision_avoidance.referenceInf.value = (0,0,0,0,0,0,0,0,0,0,0,0,-1,0,0)    # min
        self.task_collision_avoidance.referenceSup.value = (2e10,2e10,2e10,2e10,2e10,2e10,2e10,2e10,2e10,2e10,2e10,2e10,2e10,2e10,2e10)  # max
        self.task_collision_avoidance.dt.value=self.dt
        self.task_collision_avoidance.controlGain.value=0.01

        #task.featureDes.xy.value = (0,0)
        '''
        #task_wrist_metakine.opmodif = goal
        #task_wrist_metakine.feature.frame('desired')
        task_wrist_metakine.featureDes.position.value = goal
        task_wrist_metakine.gain.setConstant(1) 
        '''
 

######### solver###########################################################
        
        ##### solver old############
        '''
        self.solver = SOT ('solver')
        self.solver.setSize (robot_dimension)  
        self.solver.push (task_base.name)
        self.solver.push (task_wrist.name)
        plug (self.solver.control, self.robot.device.control)
        '''
        self.solver = SolverKine('sot')
        self.solver.setSize (robot_dimension)  
        self.solver.push (self.task_waist_metakine.task.name)
        self.solver.push (self.task_collision_avoidance.name)
        self.solver.push (self.task_wrist_metakine.task.name)
        self.solver.damping.value =3e-2
        plug (self.solver.control, self.robot.device.control)
        


    def publish(self):
        
        self.a.collisionJacobian.recompute(self.dt)
        self.a.collisionDistance.recompute(self.dt)
        print 'colljac'
        print self.a.collisionJacobian.value 
        print 'colldist'
        print self.a.collisionDistance.value
        self.robot.device.increment(0.01)
        
        '''     
        rcm.robot.device.set((8E-58,8E-58,8E-58,8E-58,8E-58,8E-58,8E-58,8E-58,8E-58,8E-58,,8E-58,8E-58))
        rcm.robot.device.increment(0.01)
        rcm.robot.dynamic.shoulder_pan_joint.recompute(dt)
        rcm.robot.dynamic.shoulder_lift_joint.recompute(dt)
        rcm.robot.dynamic.elbow_joint.recompute(dt)
        rcm.robot.dynamic.wrist_1_joint.recompute(dt)
        rcm.robot.dynamic.wrist_2_joint.recompute(dt)
        rcm.robot.dynamic.wrist_3_joint.recompute(dt)
        rcm.robot.dynamic.Jshoulder_pan_joint.recompute(dt)
        rcm.robot.dynamic.Jshoulder_lift_joint.recompute(dt)
        rcm.robot.dynamic.Jelbow_joint.recompute(dt)
        rcm.robot.dynamic.Jwrist_1_joint.recompute(dt)
        rcm.robot.dynamic.Jwrist_2_joint.recompute(dt)
        rcm.a.collisionJacobian.recompute(dt)
        print rcm.a.collisionJpair1.value
        dt = dt +1
        '''
        self.robot.dynamic.shoulder_pan_joint.recompute(self.dt)
        self.robot.dynamic.shoulder_lift_joint.recompute(self.dt)
        self.robot.dynamic.elbow_joint.recompute(self.dt)
        self.robot.dynamic.wrist_1_joint.recompute(self.dt)
        self.robot.dynamic.wrist_2_joint.recompute(self.dt)
        self.robot.dynamic.wrist_3_joint.recompute(self.dt)
        self.robot.dynamic.Jshoulder_pan_joint.recompute(self.dt)
        self.robot.dynamic.Jshoulder_lift_joint.recompute(self.dt)
        self.robot.dynamic.Jelbow_joint.recompute(self.dt)
        self.robot.dynamic.Jwrist_1_joint.recompute(self.dt)
        self.robot.dynamic.Jwrist_2_joint.recompute(self.dt)
        # recompute pair
        #self.a.collisionJacobian.recompute(self.dt)
        #self.a.imdpair1.recompute(self.dt)
        #print self.a.imdpair1.value
        #print 'im here'
        self.dt = self.dt+1
        #self.a.updatefclmodel(1)
        
        self.getstate()
        #print 'wrist_3'
        #print self.robot.dynamic.wrist_3_joint.value
        self.rcm_pub.publish(self.linkcollisionmodel)
        self.r.sleep()
        #self.linkcollisionmodel = MarkerArray()


# 0.07 0.07 0.05
#

    def getstate(self):
        state = rcm.a.getfclstate()
        #print state
        #a = self.robot.dynamic.wrist_2_joint.value
        for i in range(7):
            self.link = Marker() 
            self.link.type = self.link.MESH_RESOURCE # Cylinder
            self.link.mesh_resource = "package://sot-collision/mesh/capsule/capsule.dae"
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

# tf.transformations.quaternion_from_euler(0, 0, 30*math.pi/180),
#rcm = RobotCollisionModel()
if __name__ == '__main__' :
    rospy.init_node('collision_checker')
    rcm = RobotCollisionModel()
    i = 0
    dt = 0.01
    while i < 1000: 
    	rcm.publish()
       
'''

    rcm = RobotCollisionModel()
    while not rospy.is_shutdown():
        rcm.publish()

    runner = inc(rcm.robot)
    runner.once()
    [go,stop,next,n]=loopShortcuts(runner)
    
    a = sc.SotCollision("sc")
    a.DoCapsuleCollisionCheck(1)
    print a.getcheck()
    print a.getdistance()
    print a.gettime() 
  
    from dynamic_graph.sot.core.utils.thread_interruptible_loop import loopInThread,loopShortcuts
    @loopInThread

    def inc():
        rcm.robot.device.increment(dt)

    runner=inc()
    runner.once()
    [go,stop,next,n]=loopShortcuts(runner)
        while not rospy.is_shutdown():
            rcm.publish()
dt = 0.01
from dynamic_graph.sot.core.utils.thread_interruptible_loop import loopInThread,loopShortcuts
@loopInThread

def inc():
    robot.device.increment(0.01)

runner = inc()
runner.once()
[go,stop,next,n]=loopShortcuts(runner)
'''

'''
    def xyzrpy2tr(xyzrpy):
         tr= ((1.,0,0,0),
             (0,1.,0,0),
             (0,0,1.,0),
             (0,0,0,1.),)

        tx = xyzrpy[3] ty = xyzrpy[4] tz = xyzrpy[5]
        Rx = np.array([[1,0,0], [0, cos(tx), -sin(tx)], [0, sin(tx), cos(tx)]])
        Ry = np.array([[cos(ty), 0, -sin(ty)], [0, 1, 0], [sin(ty), 0, cos(ty)]])
        Rz = np.array([[cos(tz), -sin(tz), 0], [sin(tz), cos(tz), 0], [0,0,1]])
        R = Rx * Ry * Rz
        print R 
        # rotation
        tr[0][0]
        tr[0][0] 
        # translation
        tr[0][3] =  xyzrpy[1]
        tr[1][3] =  xyzrpy[1]
        tr[2][3] =  xyzrpy[2]
        q =tf.transformations.quaternion_from_euler(r,p,y)
        xyzq = [x,y,z,q]
        return tr

   def tr2xyzquat(tr):
        if abs(tr[2][2]) < 2.2204e-16 and abs(tr[1][2]) < 2.2204e-16:
            r = 0
            p = np.arctan2(tr[0][2], tr[2][2])
            y = np.arctan2(tr[1][0], tr[1][1])
        else:
            r = np.arctan2(-tr[1][2], tr[2][2])
            sr = sin(r)
            cr = cos(r)
            p = np.arctan2(-tr[0][2], cr * tr[2][2] - sr * tr[1][2])
            y = np.arctan2(-tr[0][1], tr[0][0])     
        x = tr[0][3]
        y = tr[1][3]
        z = tr[2][3] 
        q =tf.transformations.quaternion_from_euler(r,p,y)
        xyzq = [x,y,z,q]
        return xyzq

'''
