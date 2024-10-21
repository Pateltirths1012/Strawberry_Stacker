from gazebo_msgs.msg import*
import gazebo_ros_link_attacher
import rospy
from geometry_msgs.msg import*
from mavros_msgs.msg import*
from mavros_msgs.srv import*
from std_msgs.msg import*
from sensor_msgs.msg import*
from gazebo_ros_link_attacher.srv import*
from rospy import exceptions
from rospy.exceptions import ROSException
import cv2 as cv
import numpy as np
import cv2.aruco as aruco
from cv_bridge import CvBridge, CvBridgeError
from sensor_msgs.msg import Image

class pick_n_place_aruco:

    def __init__(self):
        rospy.init_node('pick_n_place_aruco',anonymous = True)

    def setArm(self):
        rospy.wait_for_service('mavros/cmd/arming')
        try:
            armService = rospy.ServiceProxy('mavros/cmd/arming', mavros_msgs.srv.CommandBool) 
            armService(True)
        except rospy.ServiceException as e:
            print ("Service arming call failed: %s"%e)
    
    def setDisarm(self):
        rospy.wait_for_service('mavros/cmd/arming')
        try:
            armService = rospy.ServiceProxy('mavros/cmd/arming', mavros_msgs.srv.CommandBool)
            armService(False)
        except rospy.ServiceException as e:
            print ("Service arming call failed: %s"%e)
        
    def offboard_set_mode(self):
        rospy.wait_for_service('mavros/set_mode')
        try:
            offset = rospy.ServiceProxy('mavros/set_mode',mavros_msgs.srv.SetMode)
            offset(5,"OFFBOARD")
        except rospy.ServiceException as e:
            print("Servive offboard call failed: %s"%e)

    def land_mode(self):
        rospy.wait_for_service('mavros/set_mode')
        try:
            offset = rospy.ServiceProxy('mavros/set_mode',mavros_msgs.srv.SetMode)
            offset(0,"AUTO.LAND")
        except rospy.ServiceException as e:
            print("Servive land call failed: %s"%e)

    def grip_activate(Self,grip):
        rospy.wait_for_service('activate_gripper')
        try:
            catch = rospy.ServiceProxy('activate_gripper',gazebo_ros_link_attacher.srv.Gripper)
            catch(grip)
        except rospy.ServiceException as e:
            print("Service grip activate call failed: %s"%e)

class stateMonitor:
    def __init__(self):
        self.state = State()
        self.bridge = CvBridge()
    
    def stateCb(self, msg):
        self.state = msg
    
    def local_position_cb(self, data):
        self.local_position = data
    
    def gripper_cb(self, data): 
        self.gripper_check = data
    
    def image_callback(self,data):
        self.img = self.bridge.imgmsg_to_cv2(data, "bgr8")

def main():
    stateMt = stateMonitor()
    ofb_ctl = pick_n_place_aruco()

    local_pos_pub = rospy.Publisher('mavros/setpoint_position/local', PoseStamped, queue_size=10)
    local_vel_pub = rospy.Publisher('mavros/setpoint_velocity/cmd_vel', Twist, queue_size=10)

    rate = rospy.Rate(20.0)
    
    pos = PoseStamped()
    pos.pose.position.x = 0
    pos.pose.position.y = 0
    pos.pose.position.z = 3

    vel = Twist()
    vel.linear.x = 1
    vel.linear.y = 1
    vel.linear.z = 1

    rospy.Subscriber("/mavros/state",State, stateMt.stateCb)
    rospy.Subscriber("mavros/local_position/pose",PoseStamped, stateMt.local_position_cb)
    rospy.Subscriber("gripper_check",String, stateMt.gripper_cb)
    rospy.Subscriber("eDrone/camera/image_raw",Image,stateMt.image_callback)

    for i in range(100):
        local_pos_pub.publish(pos)
        rate.sleep()

    while not stateMt.state.armed:
        ofb_ctl.setArm()
        rate.sleep()
    print("Armed!!")

    while not stateMt.state.mode=="OFFBOARD":
        ofb_ctl.offboard_set_mode()
        rate.sleep()
    print ("OFFBOARD mode activated")
    
    objectCatched = False #flag variable for object catching
    delivered = False #flag variable for box delivery
    detected = False #flag variable for object detection
    count = 0 #counter variable for object corner count to add some delay
    
    while not rospy.is_shutdown():
        #algorithm for aruco detection
        gray = cv.cvtColor(stateMt.img, cv.COLOR_BGR2GRAY)
        aruco_dict = aruco.Dictionary_get(aruco.DICT_5X5_250)
        parameters = aruco.DetectorParameters_create()
        corners,ids,_ = aruco.detectMarkers(gray, aruco_dict, parameters = parameters)

        #if the drone reaches 3m then fly to 9m in x direction
        if (detected == False) and (delivered == False) and (3 - (stateMt.local_position.pose.position.z) < 0.5):
            pos.pose.position.x = 9
            pos.pose.position.y = 0
            pos.pose.position.z = 3
            local_pos_pub.publish(pos)
            local_vel_pub.publish(vel)
        
        #if aruco marker is detected then count is incremented by 1
        if (delivered == False) and (len(corners) != 0):
            detected = True
            count += 1
        #if the aruco marker is detected 10 times then land on the marker
        if (count >= 10) and (delivered == False):
            ofb_ctl.land_mode()
            
            if(stateMt.gripper_check.data == "True"):
                ofb_ctl.grip_activate(True)
                objectCatched = True
            
        if (objectCatched == True):
            ofb_ctl.offboard_set_mode()
            pos.pose.position.x = stateMt.local_position.pose.position.x
            pos.pose.position.y = 0
            pos.pose.position.z = 3
            local_pos_pub.publish(pos)
            local_vel_pub.publish(vel)
        
        if (objectCatched == True) and (3 - (stateMt.local_position.pose.position.z) < 0.5):
            pos.pose.position.x = 9
            pos.pose.position.y = 0
            pos.pose.position.z = 3
            local_pos_pub.publish(pos)
            local_vel_pub.publish(vel)
        
        if (objectCatched == True) and (9 - (stateMt.local_position.pose.position.x) < 0.5):
            ofb_ctl.land_mode()

            if (stateMt.local_position.pose.position.z < 0.2):
                ofb_ctl.grip_activate(False)
                objectCatched = False
                detected = False
                delivered = True
        
        if (delivered == True) and (stateMt.state.mode=="AUTO.LAND") and (stateMt.local_position.pose.position.x > 1):
            ofb_ctl.offboard_set_mode()
            pos.pose.position.x = 9
            pos.pose.position.y = 0
            pos.pose.position.z = 3
            local_pos_pub.publish(pos)
            local_vel_pub.publish(vel)
            
            
        if (delivered == True) and (9 - (stateMt.local_position.pose.position.x) < 1) and (3 - (stateMt.local_position.pose.position.z) < 1):
            ofb_ctl.offboard_set_mode()
            pos.pose.position.x = 0
            pos.pose.position.y = 0
            pos.pose.position.z = 3
            local_pos_pub.publish(pos)
            local_vel_pub.publish(vel)
            
        
        if (delivered == True) and (stateMt.local_position.pose.position.x < 0.5) and (3 - (stateMt.local_position.pose.position.z) < 0.5):
            ofb_ctl.land_mode()
            

        rate.sleep()
    
if __name__ == '__main__':
    try:
        main()
    except rospy.ROSInterruptException:
        pass