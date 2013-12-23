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
from dynamic_graph.sot.ur.mobile_robot import Ur
from dynamic_graph.sot.core import RobotSimu, FeaturePosition, Task, SOT
from dynamic_graph import plug, writeGraph
from dynamic_graph.ros import Ros


'''
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
        self.buildurrobot()

    def buildurrobot(self):
        self.robot = Ur('ur5', device=RobotSimu('ur5'))
        robot_dimension = self.robot.dynamic.getDimension() 
        self.robot.device.resize (robot_dimension)

        # 2. Ros binding
        # roscore must be running
        ros = Ros(self.robot)

        # Create task for the waist
        robot_pose = ((1.,0,0,0),
            (0,1.,0,0),
            (0,0,1.,0.089159),
            (0,0,0,1.),)	
        feature_waist = FeaturePosition ('position_waist', self.robot.dynamic.shoulder_pan_joint,self.robot.dynamic.Jshoulder_pan_joint, robot_pose)
        task_waist = Task ('waist_task')
        task_waist.controlGain.value = 100.
        task_waist.add (feature_waist.name)

       # Create task for the lift
        I4 =   ((1.,0,0,0.0),
            (0,1.,0,0.136),
            (0,0,1.,0.089),
            (0,0,0,1.),)
        feature_lift = FeaturePosition ('position_lift', self.robot.dynamic.shoulder_lift_joint, self.robot.dynamic.Jshoulder_lift_joint, I4)
        #feature_lift.selec.value = '000000'
        task_lift = Task ('lift_task')
        task_lift.controlGain.value = 0.
        task_lift.add (feature_lift.name)

       # Create task for the elbow
        I4 =   ((1.,0,0,0.321),
            (0,1.,0,0.109),
            (0,0,1.,0.848),
            (0,0,0,1.),)
        feature_elbow = FeaturePosition ('position_elbow', self.robot.dynamic.elbow_joint, self.robot.dynamic.Jelbow_joint, I4)
        #feature_elbow.selec.value = '000000'
        task_elbow = Task ('elbow_task')
        task_elbow.controlGain.value = 0.
        task_elbow.add (feature_elbow.name)


        # Create task for the wrist1
        I4 =   ((1.,0,0,0.321),
            (0,1.,0,0.109),
            (0,0,1.,0.848),
            (0,0,0,1.),)
        feature_wrist1 = FeaturePosition ('position_wrist1', self.robot.dynamic.wrist_1_joint, self.robot.dynamic.Jwrist_1_joint, I4)
        #feature_wrist1.selec.value = '000000'
        task_wrist1 = Task ('wrist1_task')
        task_wrist1.controlGain.value = 0.
        task_wrist1.add (feature_wrist1.name)

        # Create task for the wrist2
        I4 =   ((1.,0,0,0.321),
            (0,1.,0,0.109),
            (0,0,1.,0.848),
            (0,0,0,1.),)
        feature_wrist2 = FeaturePosition ('position_wrist2', self.robot.dynamic.wrist_2_joint, self.robot.dynamic.Jwrist_2_joint, I4)
        #feature_wrist2.selec.value = '000000'
        task_wrist2 = Task ('wrist2_task')
        task_wrist2.controlGain.value = 0.
        task_wrist2.add (feature_wrist2.name)


        # Create task for the wrist3
        I4 =   ((1.,0,0,0.321),
            (0,1.,0,0.109),
            (0,0,1.,0.848),
            (0,0,0,1.),)
        feature_wrist = FeaturePosition ('position_wrist', self.robot.dynamic.wrist_3_joint, self.robot.dynamic.Jwrist_3_joint, I4)
        task_wrist = Task ('wrist_task')
        task_wrist.controlGain.value = 0.5
        task_wrist.add (feature_wrist.name)

        # solver
        self.solver = SOT ('solver')
        self.solver.setSize (robot_dimension)
        self.solver.push (task_waist.name)
        self.solver.push (task_wrist.name)
        self.solver.push (task_elbow.name)
        self.solver.push (task_lift.name)
        self.solver.push (task_wrist1.name)
        self.solver.push (task_wrist2.name)
        plug (self.solver.control, self.robot.device.control)
        # sot collision
        self.a = sc.SotCollision("sc")
#(((0,0.08,0.01,0,0,0.0,0.0,0,0),(1,0.08,0.04,0,0,0,0,0,0),(2,0.09,0.17,0.2,0.0,0.0,0,1.57,0),(3,0.07,0.14,0.2,0.0,0.0,-3.1416,1.5706,-3.1416),(4,0.05,0.03,0.0,0.093,0,-3.14,0,-3.14),(5,0.057,0.02,0.0,0,-0.095,-3.14,0,-3.14),(6,0.04,0.01,0.0,0.065,0,1.57,0,0)))
        self.a.createlinkmodel(((0,0.08,0.01,0,0,0.0,0.0,0,0),(1,0.08,0.04,0,0,0,0,0,0),(2,0.09,0.15,0.2,0.0,0.0,0,1.57,0),(3,0.07,0.14,0.2,0.0,0.0,-3.1416,1.5706,-3.1416),(4,0.05,0.03,0.0,0.093,0,-3.14,0,-3.14),(5,0.057,0.02,0.0,0,-0.095,-3.14,0,-3.14),(6,0.04,0.01,0.0,0.065,0,1.57,0,0)))
        plug (self.robot.dynamic.base_joint,self.a.link_0)
        plug (self.robot.dynamic.shoulder_pan_joint,self.a.link_1)
        plug ( self.robot.dynamic.shoulder_lift_joint,self.a.link_2)
        plug ( self.robot.dynamic.elbow_joint,self.a.link_3)
        plug ( self.robot.dynamic.wrist_1_joint,self.a.link_4)
        plug (self.robot.dynamic.wrist_2_joint, self.a.link_5)
        plug (self.robot.dynamic.wrist_3_joint, self.a.link_6)
    def publish(self):
        self.robot.device.increment(0.01)
        self.a.updatefclmodel(1)
        self.getstate()
        self.rcm_pub.publish(self.linkcollisionmodel)
        print self.robot.dynamic.shoulder_lift_joint.value
        #self.r.sleep()
        #self.linkcollisionmodel = MarkerArray()


# 0.07 0.07 0.05
#

    def getstate(self):
        state = rcm.a.getfclstate()
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
    while not rospy.is_shutdown():
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
