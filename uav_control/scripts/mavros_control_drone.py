#from __future__ import print_function
import rospy
import tf
from geometry_msgs.msg import Pose, PoseStamped, Twist, Quaternion
from mavros_msgs.msg import State
from mavros_msgs.srv import CommandBool, SetMode, CommandTOL
from sensor_msgs.msg import NavSatFix
from std_msgs.msg import Float64MultiArray, UInt16MultiArray
from cv2 import imwrite
from numpy import zeros
from os import mkdir
from time import time, localtime
from sensor_msgs.msg import Image
from cv_bridge import CvBridge
from math import atan2
from os.path import exists

pi_2 = 3.141592654 / 2.0

class MavController_drone:
    def __init__(self):

        rospy.init_node("mav_control_node")
        rospy.Subscriber("/mavros/local_position/pose", PoseStamped, self.pose_callback)
        rospy.Subscriber("/mavros/state", State, self.state_callback)

        self.cmd_pos_pub = rospy.Publisher("/mavros/setpoint_position/local", PoseStamped, queue_size=100)
        self.cmd_vel_pub = rospy.Publisher("/mavros/setpoint_velocity/cmd_vel_unstamped", Twist, queue_size=100)

        self.mode_service = rospy.ServiceProxy('/mavros/set_mode', SetMode)
        self.arm_service = rospy.ServiceProxy('/mavros/cmd/arming', CommandBool)
        self.takeoff_service = rospy.ServiceProxy('/mavros/cmd/takeoff', CommandTOL)
        
        self.mode = "HOLD"
        self.save_meas = False
        self.org_alt = 0.0
        self.pose = Pose()
        self.timestamp = rospy.Time()
        self.pos_eps = 0.2
        self.ang_eps = 0.1
        self.f_mode = True

        self.rate = rospy.Rate(10)
        
        t = localtime( time() )
        self.time = "{}_{}_{}_{}_{}".format(t.tm_year, t.tm_mon, t.tm_mday, t.tm_hour, t.tm_min)
        self.file_name = '../result/' + self.time
        if not exists(self.file_name):
            mkdir(self.file_name)
        '''self.img_num = 1
        self.bridge = CvBridge()
        self.cv_image = zeros([720, 1280, 3])'''

    def save_msg(self, filename, data, write_type='a'):
        with open(filename, write_type) as f:
            for d in data:
              f.write('{}\n'.format(d))
                  
    def pose_callback(self, msg):
        self.timestamp = msg.header.stamp
        self.pose = msg.pose
        if self.save_meas:
            data = [msg.header.stamp, msg.pose.position.x, \
                                      msg.pose.position.y, \
                                      msg.pose.position.z, \
                                      msg.pose.orientation.x, \
                                      msg.pose.orientation.y, \
                                      msg.pose.orientation.z, \
                                      msg.pose.orientation.w ]
            self.save_msg(self.file_name + '/mavros_local.txt', data)
    
    def state_callback(self, data):
        if not self.mode == data.mode:
            rospy.loginfo("Change mode to {}".format(data.mode))
        self.mode = data.mode
    
    def arm(self):
        mode_resp = self.mode_service(custom_mode="4")
        rospy.loginfo("Arm")
        return self.arm_service(True)

    def disarm(self):
        resp = self.mode_service(custom_mode="9")
        rospy.loginfo("Disarm")
        return self.arm_service(False)
    
    def takeoff(self, height=1.2):
        self.org_alt = self.pose.position.z
        takeoff_resp = self.takeoff_service(altitude=height)
        rospy.loginfo("Take Off")
        return takeoff_resp
    
    def land(self):
        resp = self.mode_service(custom_mode="9")
        rospy.sleep(5)
        self.disarm()

    def is_terminal(self, x, y, z, yaw):
        eular = tf.transformations.euler_from_quaternion((self.pose.orientation.x, self.pose.orientation.y, self.pose.orientation.z, self.pose.orientation.w))
        rospy.loginfo("( {:5.2f}, {:5.2f}, {:5.2f}, {:5.2f} ) | ( {:5.2f}, {:5.2f}, {:5.2f}, {:5.2f} )".format(x, y, z, yaw, self.pose.position.x, self.pose.position.y, self.pose.position.z, eular[2]))
        return abs(x - self.pose.position.x) < self.pos_eps and \
               abs(y - self.pose.position.y) < self.pos_eps and \
               abs(z - self.pose.position.z) < self.pos_eps and \
               ( abs(yaw - eular[2]) < self.ang_eps or abs( abs(yaw - eular[2]) - 6.28 ) < self.ang_eps )

    def goto_xyz_yaw(self, x, y, z, yaw):
        rospy.loginfo("Go to x = {:5.2f} ; y = {:5.2f} ; z = {:5.2f} ; yaw = {:5.2f}".format(x, y, z, yaw))
        pose = Pose()
        pose.position.x = x
        pose.position.y = y
        pose.position.z = z + self.org_alt

        quat = tf.transformations.quaternion_from_euler(0.0, 0.0, yaw)
        pose.orientation.x = quat[0]
        pose.orientation.y = quat[1]
        pose.orientation.z = quat[2]
        pose.orientation.w = quat[3]
        
        pose_stamped = PoseStamped()
        pose_stamped.header.stamp = self.timestamp
        pose_stamped.pose = pose
        
        '''for _ in range(40):
            self.cmd_pos_pub.publish(pose_stamped)
            self.rate.sleep()'''
        
        while not self.is_terminal(x, y, z, yaw):
            self.cmd_pos_pub.publish(pose_stamped)
            self.rate.sleep()
    
    def goto_terminal(self, x, y, z, yaw):
        # Turn to terminal
        theta = atan2( (y - self.pose.position.y), (x - self.pose.position.x) )
        self.goto_xyz_yaw(self.pose.position.x, self.pose.position.y, self.pose.position.z, theta)
        
        # Go Forward
        self.goto_xyz_yaw(x, y, z, theta)
        
        # Turn to yaw
        self.goto_xyz_yaw(x, y, z, yaw)
        
        # Hover and Save image
        # rospy.sleep(8)
        #self.save_img()
        rospy.sleep(3)
        rospy.loginfo('====================')
    
    def set_vel(self, vx, vy, vz, avx=0, avy=0, avz=0):
        """
        Send comand velocities. Must be in GUIDED mode. Assumes angular
        velocities are zero by default.
        """
        cmd_vel = Twist()

        cmd_vel.linear.x = vx
        cmd_vel.linear.y = vy
        cmd_vel.linear.z = vz

        cmd_vel.angular.x = avx
        cmd_vel.angular.y = avy
        cmd_vel.angular.z = avz
        
        print(cmd_vel)
        self.cmd_vel_pub.publish(cmd_vel)

        
        

