import rospy
from mavros_control_drone import *
from time import time, localtime

if __name__=='__main__':

    drone_ctr = MavController_drone()
    pi_ = 3.141592654
    alt = 1.2
    
    # Arm
    drone_ctr.save_meas = True
    drone_ctr.arm()
    rospy.sleep(5)
    drone_ctr.takeoff(alt)
    rospy.sleep(5)
    
    # way points [x, y, z, yaw]
    way_points = [[1.0,  0.0, alt,  0.0], \
                  [1.0,  1.0, alt,  0.0], \
                  [0.0,  1.0, alt,  0.0], \
                  [0.0,  0.0, alt,  0.0]]
    
    for wp in way_points:
        # 1 Turn heading and go
        '''drone_ctr.goto_terminal(wp[0], wp[1], wp[2], wp[3])
        '''# 2 Directly go
        drone_ctr.goto_xyz_yaw(wp[0], wp[1], wp[2], wp[3])
        rospy.sleep(5)
        print("( {:5.2f} , {:5.2f} , {:5.2f} )".format(drone_ctr.pose.position.x, drone_ctr.pose.position.y, drone_ctr.pose.position.z - drone_ctr.org_alt))
    
    # Disarm
    drone_ctr.land()
    rospy.sleep(5)
    drone_ctr.save_meas = False

