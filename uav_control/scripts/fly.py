import rospy
from mavros_control_drone import *
from time import time, localtime

from uav_control.srv import Coord,CoordResponse
import rospy
import time

drone_ctr = MavController_drone()
pi_ = 3.141592654
alt = 1.0
flying = False

def fly_to_subgoal(req):
    global drone_ctr, pi_, alt, flying
    
    if not flying:
        # Arm
        drone_ctr.save_meas = True
        drone_ctr.arm()
        rospy.sleep(5)
        drone_ctr.takeoff(alt)
        rospy.sleep(5)
        flying = True

    if (req.x == -1) and (req.y == -1) and (req.z == -1) and (req.yaw == -1):
        # Disarm
        drone_ctr.land()
        rospy.sleep(5)
        drone_ctr.save_meas = False
        flying = False
    else:
        print("Heading to SG ({}, {}, {}, {})".format(req.x, req.y, req.z, req.yaw))
        # 1 Turn heading and go
        drone_ctr.goto_terminal(req.x, req.y, req.z, req.yaw)
        
        # 2 Directly go
        #drone_ctr.goto_xyz_yaw(req.x, req.y, req.z, req.yaw)
        rospy.sleep(5)
        print("( {:5.2f} , {:5.2f} , {:5.2f} )".format(drone_ctr.pose.position.x, drone_ctr.pose.position.y, drone_ctr.pose.position.z - drone_ctr.org_alt))

    # print("Returning [{}, {}, {}, {}]".format(req.x, req.y, req.z, req.yaw))
    return CoordResponse("OK")

def uav_flight_server():
    #rospy.init_node('flight_server')
    s = rospy.Service('/flight_srv', Coord, fly_to_subgoal)
    print("Ready to fly.")
    rospy.spin()

if __name__ == "__main__":
    uav_flight_server()
