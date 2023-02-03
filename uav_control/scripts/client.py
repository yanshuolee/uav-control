import rospy
from uav_control.srv import Coord,CoordRequest
import time

def uav_flight_client(x, y, z, yaw):
    rospy.init_node('flight_client')
    rospy.wait_for_service('/flight_srv')
    try:
        flight_c = rospy.ServiceProxy('/flight_srv', Coord)
        resp1 = flight_c(x, y, z, yaw)
        return resp1.result
    except rospy.ServiceException as e:
        print("Service call failed: %s"%e)

if __name__ == "__main__":
    x, y, z, yaw = 1.0, 2.0, 3.0, 0.0
    for i in range(10):
        x, y, z, yaw = x+i, y+i, z+i, yaw+i
        print("Requesting {}, {}, {}, {}".format(x, y, z, yaw))
        print("Response:", uav_flight_client(x, y, z, yaw))
