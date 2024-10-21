from gazebo_msgs.msg import*
from gazebo_msgs.srv import*
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
from sensor_msgs.msg import *
import math


class multidrone:

    def __init__(self):
        rospy.init_node('multidrone',anonymous = True)
    
    #services for drone 1 from here
    def edrone0_setArm(self):
        rospy.wait_for_service('edrone0/mavros/cmd/arming')
        try:
            armService = rospy.ServiceProxy('edrone0/mavros/cmd/arming', mavros_msgs.srv.CommandBool) 
            armService(True)
        except rospy.ServiceException as e:
            print ("Service arming call failed: %s"%e)
    
    def edrone0_offboard_set_mode(self):
        rospy.wait_for_service('edrone0/mavros/set_mode')
        try:
            offset = rospy.ServiceProxy('edrone0/mavros/set_mode',mavros_msgs.srv.SetMode)
            offset(5,"OFFBOARD")
        except rospy.ServiceException as e:
            print("Servive offboard call failed: %s"%e)
    
    def edrone0_land_mode(self):
        rospy.wait_for_service('edrone0/mavros/set_mode')
        try:
            offset = rospy.ServiceProxy('edrone0/mavros/set_mode',mavros_msgs.srv.SetMode)
            offset(0,"AUTO.LAND")
        except rospy.ServiceException as e:
            print("Servive land call failed: %s"%e)

    def edrone0_grip_activate(Self,grip):
        rospy.wait_for_service('edrone0/activate_gripper')
        try:
            catch = rospy.ServiceProxy('edrone0/activate_gripper',gazebo_ros_link_attacher.srv.Gripper)
            catch(grip)
        except rospy.ServiceException as e:
            print("Service grip activate call failed: %s"%e)

    #services for drone2 from here
    def edrone1_setArm(self):
        rospy.wait_for_service('edrone1/mavros/cmd/arming')
        try:
            armService = rospy.ServiceProxy('edrone1/mavros/cmd/arming', mavros_msgs.srv.CommandBool) 
            armService(True)
        except rospy.ServiceException as e:
            print ("Service arming call failed: %s"%e)
    
    def edrone1_offboard_set_mode(self):
        rospy.wait_for_service('edrone1/mavros/set_mode')
        try:
            offset = rospy.ServiceProxy('edrone1/mavros/set_mode',mavros_msgs.srv.SetMode)
            offset(5,"OFFBOARD")
        except rospy.ServiceException as e:
            print("Servive offboard call failed: %s"%e)
    
    def edrone1_land_mode(self):
        rospy.wait_for_service('edrone1/mavros/set_mode')
        try:
            offset = rospy.ServiceProxy('edrone1/mavros/set_mode',mavros_msgs.srv.SetMode)
            offset(0,"AUTO.LAND")
        except rospy.ServiceException as e:
            print("Servive land call failed: %s"%e)

    def edrone1_grip_activate(Self,grip):
        rospy.wait_for_service('edrone1/activate_gripper')
        try:
            catch = rospy.ServiceProxy('edrone1/activate_gripper',gazebo_ros_link_attacher.srv.Gripper)
            catch(grip)
        except rospy.ServiceException as e:
            print("Service grip activate call failed: %s"%e)
    
    
class stateMonitor:
    def __init__(self):
        self.state1 = State()
        self.state2 = State()
        self.bridge = CvBridge()
        
    def edrone0_stateCb(self, msg):
        self.state1 = msg
    
    def edrone0_local_position_cb(self, data):
        self.local_position0 = data
    
    def edrone0_gripper_cb(self, data): 
        self.gripper_check1 = data
    
    def edrone0_image_callback(self,data):
        self.img1 = self.bridge.imgmsg_to_cv2(data, "bgr8")

    def edrone0_camera_info(self,data):
        self.camera0 = data
    
    def edrone1_stateCb(self, msg):
        self.state2 = msg
    
    def edrone1_local_position_cb(self, data):
        self.local_position1 = data
    
    def edrone1_gripper_cb(self, data): 
        self.gripper_check2 = data
    
    def edrone1_image_callback(self,data):
        self.img2 = self.bridge.imgmsg_to_cv2(data, "bgr8")
    
    def edrone1_camera_info(self,data):
        self.camera1 = data
    


def main():
    stateMt = stateMonitor()
    ofb_ctl = multidrone()

    edrone0_local_pos_pub = rospy.Publisher('edrone0/mavros/setpoint_position/local',PoseStamped, queue_size=10)
    edrone1_local_pos_pub = rospy.Publisher('edrone1/mavros/setpoint_position/local',PoseStamped, queue_size=10)
    edrone0_local_vel_pub = rospy.Publisher('edrone0/mavros/setpoint_velocity/cmd_vel',Twist,queue_size=10)
    edrone1_local_vel_pub = rospy.Publisher('edrone1/mavros/setpoint_velocity/cmd_vel',Twist,queue_size=10)

    rate = rospy.Rate(20.0)

    pos0 = PoseStamped()
    pos0.pose.position.x = 0
    pos0.pose.position.y = 0
    pos0.pose.position.z = 3

    pos1 = PoseStamped()
    pos1.pose.position.x = 0
    pos1.pose.position.y = 0
    pos1.pose.position.z = 3

    vel = Twist()
    vel.linear.x = 1
    vel.linear.y = 1
    vel.linear.z = 1

    #subscribers for drone 0
    rospy.Subscriber("edrone0/mavros/state",State, stateMt.edrone0_stateCb)
    rospy.Subscriber("edrone0/mavros/local_position/pose",PoseStamped, stateMt.edrone0_local_position_cb)
    rospy.Subscriber("edrone0/gripper_check",String, stateMt.edrone0_gripper_cb)
    rospy.Subscriber("edrone0/camera/image_raw",Image,stateMt.edrone0_image_callback)
    rospy.Subscriber("edrone0/camera/camera_info",CameraInfo,stateMt.edrone0_camera_info)
    #subscribers for drone 1
    rospy.Subscriber("edrone1/mavros/state",State, stateMt.edrone1_stateCb)
    rospy.Subscriber("edrone1/mavros/local_position/pose",PoseStamped, stateMt.edrone1_local_position_cb)
    rospy.Subscriber("edrone1/gripper_check",String, stateMt.edrone1_gripper_cb)
    rospy.Subscriber("edrone1/camera/image_raw",Image,stateMt.edrone1_image_callback)
    rospy.Subscriber("edrone1/camera/camera_info",CameraInfo,stateMt.edrone1_camera_info)

    for i in range(100):
        edrone0_local_pos_pub.publish(pos0)
        edrone1_local_pos_pub.publish(pos1)
        rate.sleep()
    
    while (stateMt.state1.mode != "OFFBOARD") and (stateMt.state2.mode != "OFFBOARD"):
        ofb_ctl.edrone0_offboard_set_mode()
        ofb_ctl.edrone1_offboard_set_mode()
        rate.sleep()
    print("edrones OFFBOARD mode activated")

    while (stateMt.state1.armed != True) and (stateMt.state2.armed != True):
        ofb_ctl.edrone0_setArm()
        ofb_ctl.edrone1_setArm()
        rate.sleep()
    print("edrones Armed!")

    box1_catched = False
    box2_catched = False
    box3_catched = False
    box4_catched = False

    box1_delivered = False
    box2_delivered = False
    box3_delivered = False
    box4_delivered = False

    while not rospy.is_shutdown():
        #current positions of edrone0
        d0_x = stateMt.local_position0.pose.position.x
        d0_y = stateMt.local_position0.pose.position.y
        d0_z = stateMt.local_position0.pose.position.z

        #current positions of edrone1
        d1_x = stateMt.local_position1.pose.position.x
        d1_y = stateMt.local_position1.pose.position.y
        d1_z = stateMt.local_position1.pose.position.z

        if (box2_delivered == False) and (int(d0_x) == 0) and (int(d0_y) == 0):
            edrone0_local_pos_pub.publish(pos0)

        if (box4_delivered == False) and (int(d1_x) == 0) and (int(d1_y) == 0):
            edrone1_local_pos_pub.publish(pos1)

        #code for image processing
        aruco_dict = aruco.Dictionary_get(aruco.DICT_5X5_250)
        parameters = aruco.DetectorParameters_create()
        mtx = np.array(stateMt.camera0.K)
        mtx = mtx.reshape(3,3)
        dst = np.array(stateMt.camera0.D)
        corners1,ids1,p1 = aruco.detectMarkers(stateMt.img1, aruco_dict, parameters = parameters, cameraMatrix = mtx, distCoeff = dst)
        corners2,ids2,p2 = aruco.detectMarkers(stateMt.img2, aruco_dict, parameters = parameters, cameraMatrix = mtx, distCoeff = dst)
        
        d0_x = stateMt.local_position0.pose.position.x
        d0_y = stateMt.local_position0.pose.position.y
        d0_z = stateMt.local_position0.pose.position.z

        d1_x = stateMt.local_position1.pose.position.x
        d1_y = stateMt.local_position1.pose.position.y
        d1_z = stateMt.local_position1.pose.position.z
        '''algorithm for drone 1 starts here'''
        if (box1_delivered == False) and (box2_delivered == False):

            if (3 - (d0_z) < 0.5):
                pos0.pose.position.x = 0
                pos0.pose.position.y = 16
                pos0.pose.position.z = 3
                edrone0_local_pos_pub.publish(pos0)
            
            if (16 - (d0_y) < 0.5):
                pos0.pose.position.x = 10
                pos0.pose.position.y = 16
                pos0.pose.position.z = 3
                edrone0_local_pos_pub.publish(pos0)

            # algorithm for picking up box1
            if (len(corners1) != 0):   
                rvecs,tvecs,_ = aruco.estimatePoseSingleMarkers(corners1,0.05,mtx,dst)
                yaw = -1 * math.atan2(tvecs[0][0][0], tvecs[0][0][2])
                angle = np.rad2deg(yaw)
                pos0.pose.position.x = d0_x - 0.4
                pos0.pose.position.y = 16
                pos0.pose.position.z = 3
                edrone0_local_pos_pub.publish(pos0)

                if (abs(angle) < 1):
                    ofb_ctl.edrone0_land_mode()

            if (stateMt.gripper_check1.data == "True") and (d0_z < 1):
                ofb_ctl.edrone0_grip_activate(True)
                box1_catched = True 

            if (box1_catched == True):
                ofb_ctl.edrone0_offboard_set_mode()
                pos0.pose.position.x = d0_x            
                pos0.pose.position.y = 16
                pos0.pose.position.z = 3
                edrone0_local_pos_pub.publish(pos0)  
            
            if (box1_catched == True) and (3 - (d0_z) < 0.5):
                pos0.pose.position.x = 15
                pos0.pose.position.y = 16
                pos0.pose.position.z = 3
                edrone0_local_pos_pub.publish(pos0)
            
            if (box1_catched  == True) and (15 - (d0_x) < 0.5):
                pos0.pose.position.x = 15
                pos0.pose.position.y = -7.5
                pos0.pose.position.z = 3
                edrone0_local_pos_pub.publish(pos0)
            #algorithm for delivering box 1
            if (box1_catched  == True) and (abs(-7.5 - (d0_y)) < 1):
                pos0.pose.position.x = 15
                pos0.pose.position.y = d0_y + 0.4
                pos0.pose.position.z = 3
                edrone0_local_pos_pub.publish(pos0)

                if (d0_y > -7.5):
                    ofb_ctl.edrone0_land_mode()
                    ofb_ctl.edrone0_land_mode()
                if (d0_z < 2):
                    ofb_ctl.edrone0_grip_activate(False)
                    stateMt.gripper_check1.data = "False"
                    box1_catched = False
                    box1_delivered = True

        if (box1_delivered == True) and (box2_delivered == False):    

            if (stateMt.state1.mode == "AUTO.LAND") and (d0_y < -1):
                ofb_ctl.edrone0_offboard_set_mode()
                pos0.pose.position.x = d0_x
                pos0.pose.position.y = d0_y
                pos0.pose.position.z = 3
                edrone0_local_pos_pub.publish(pos0)
            
            if (3 - (d0_z) < 0.5):
                pos0.pose.position.x = 15
                pos0.pose.position.y = 23.9
                pos0.pose.position.z = 3
                edrone0_local_pos_pub.publish(pos0)

            if (24 - (d0_y) < 0.5):
                pos0.pose.position.x = 23
                pos0.pose.position.y = 23.9
                pos0.pose.position.z = 3
                edrone0_local_pos_pub.publish(pos0)
            
            if (len(corners1) != 0):
                rvecs,tvecs,_ = aruco.estimatePoseSingleMarkers(corners1,0.05,mtx,dst)
                yaw = -1 * math.atan2(tvecs[0][0][0], tvecs[0][0][2])
                angle = np.rad2deg(yaw)
                pos0.pose.position.x = d0_x - 0.4 
                pos0.pose.position.y = 23.9
                pos0.pose.position.z = 3
                edrone0_local_pos_pub.publish(pos0)

                if abs(angle) < 1:
                    ofb_ctl.edrone0_land_mode()

            if (stateMt.gripper_check1.data == "True") and (d0_z < 1):
                ofb_ctl.edrone0_grip_activate(True)
                box2_catched = True 
                
            if (box2_catched == True):
                ofb_ctl.edrone0_offboard_set_mode()
                pos0.pose.position.x = d0_x            
                pos0.pose.position.y = 23.9
                pos0.pose.position.z = 5
                edrone0_local_pos_pub.publish(pos0)
            
            if (box2_catched == True) and (5 - (d0_z) < 0.5):
                pos0.pose.position.x = 15
                pos0.pose.position.y = -6.5
                pos0.pose.position.z = 5
                edrone0_local_pos_pub.publish(pos0)
            
            if (box2_catched == True) and (abs(-6.5 - (d0_y)) < 1):
                pos0.pose.position.x = 15
                pos0.pose.position.y = d0_y + 0.4
                pos0.pose.position.z = 5
                edrone0_local_pos_pub.publish(pos0)

                if (d0_y) > -6.5:
                    ofb_ctl.edrone0_land_mode()
                    ofb_ctl.edrone0_land_mode()
                if (d0_z < 2):
                    ofb_ctl.edrone0_grip_activate(False)
                    stateMt.gripper_check1.data = "False"
                    box2_catched = False
                    box2_delivered = True
        
        if (box2_delivered == True):

            if (stateMt.state1.mode == "AUTO.LAND") and (d0_y < -1):
                ofb_ctl.edrone0_offboard_set_mode()
                ofb_ctl.edrone0_grip_activate(False)
                pos0.pose.position.x = d0_x
                pos0.pose.position.y = d0_y
                pos0.pose.position.z = 5
                edrone0_local_pos_pub.publish(pos0)
            
            if (5 - (d0_z) < 0.5):
                pos0.pose.position.x = 0
                pos0.pose.position.y = 0
                pos0.pose.position.z = 5
                edrone0_local_pos_pub.publish(pos0)

            if (d0_x < 0) and (5 - (d0_z) < 0.5):
                ofb_ctl.edrone0_land_mode()

        '''algorithm for drone 2 starts here'''
        if (box3_delivered == False) and (box4_delivered == False):

            if (3 - (d1_z) < 0.5):
                pos1.pose.position.x = 0
                pos1.pose.position.y = -12
                pos1.pose.position.z = 3
                edrone1_local_pos_pub.publish(pos1)

            if (abs(-12 - (d1_y)) < 0.5):
                pos1.pose.position.x = 17
                pos1.pose.position.y = -12
                pos1.pose.position.z = 3
                edrone1_local_pos_pub.publish(pos1)

            if (len(corners2) != 0):
                rvecs,tvecs,_ = aruco.estimatePoseSingleMarkers(corners2,0.05,mtx,dst)
                yaw = -1 * math.atan2(tvecs[0][0][0], tvecs[0][0][2])
                angle = np.rad2deg(yaw)
                pos1.pose.position.x =  d1_x - 0.4
                pos1.pose.position.y = -12
                pos1.pose.position.z = 3
                edrone1_local_pos_pub.publish(pos1)
               
                if (abs(angle) < 1):
                    ofb_ctl.edrone1_land_mode()

            if (stateMt.gripper_check2.data == "True") and (d1_z < 1):
                ofb_ctl.edrone1_grip_activate(True)
                box3_catched = True

            if (box3_catched == True):
                ofb_ctl.edrone1_offboard_set_mode()
                pos1.pose.position.x = d1_x
                pos1.pose.position.y = -12
                pos1.pose.position.z = 5
                edrone1_local_pos_pub.publish(pos1)
        
            if (box3_catched == True) and (5 - (d1_z) < 1):
                pos1.pose.position.x = 57
                pos1.pose.position.y = -12
                pos1.pose.position.z = 5
                edrone1_local_pos_pub.publish(pos1)
            
            if (box3_catched == True) and (57 - (d1_x) < 1):
                pos1.pose.position.x = 57
                pos1.pose.position.y = 4.5
                pos1.pose.position.z = 5
                edrone1_local_pos_pub.publish(pos1)
            
            if (box3_catched == True) and (4.5 - (d1_y) < 1):
                pos1.pose.position.x = 57
                pos1.pose.position.y = d1_y - 0.4
                pos1.pose.position.z = 5
                edrone1_local_pos_pub.publish(pos1)

                if (d1_y < 4.5):
                    ofb_ctl.edrone1_land_mode()
                    ofb_ctl.edrone1_land_mode()
                if (d1_z < 2):
                    ofb_ctl.edrone1_grip_activate(False)
                    stateMt.gripper_check2.data = "False"
                    box3_catched = False
                    box3_delivered = True

        if (box3_delivered == True) and (box4_delivered == False):

            if (stateMt.state2.mode == "AUTO.LAND") and (d1_y > 0): 
                ofb_ctl.edrone1_offboard_set_mode()
                ofb_ctl.edrone1_grip_activate(False)
                pos1.pose.position.x = d1_x
                pos1.pose.position.y = d1_y
                pos1.pose.position.z = 3
                edrone1_local_pos_pub.publish(pos1)
            
            if (3 - (d1_z) < 0.5): 
                pos1.pose.position.x = 58
                pos1.pose.position.y = -32
                pos1.pose.position.z = 3
                edrone1_local_pos_pub.publish(pos1)

            if (abs(-32 - (d1_y)) < 1):
                pos1.pose.position.x = 3
                pos1.pose.position.y = -32
                pos1.pose.position.z = 3
                edrone1_local_pos_pub.publish(pos1)
            
            if (len(corners2)!=0):
                rvecs,tvecs,_ = aruco.estimatePoseSingleMarkers(corners2,0.05,mtx,dst)
                yaw = -1 * math.atan2(tvecs[0][0][0], tvecs[0][0][2])
                angle = np.rad2deg(yaw)
                pos1.pose.position.x =  d1_x + 0.4
                pos1.pose.position.y = -32
                pos1.pose.position.z = 3
                edrone1_local_pos_pub.publish(pos1)

                if (abs(angle) < 1):
                    ofb_ctl.edrone1_land_mode()

            if (stateMt.gripper_check2.data == "True") and (d1_z < 1):
                ofb_ctl.edrone1_grip_activate(True)
                box4_catched = True

            if (box4_catched == True):
                ofb_ctl.edrone1_offboard_set_mode()
                pos1.pose.position.x = d1_x
                pos1.pose.position.y = -32
                pos1.pose.position.z = 3
                edrone1_local_pos_pub.publish(pos1)
            
            if (box4_catched == True) and (5 - (d1_z) < 0.5):
                pos1.pose.position.x = 58
                pos1.pose.position.y = -32
                pos1.pose.position.z = 3
                edrone1_local_pos_pub.publish(pos1)
            
            if (box4_catched == True) and (58 - (d1_x) < 0.5):
                pos1.pose.position.x = 58
                pos1.pose.position.y = 5.5
                pos1.pose.position.z = 3
                edrone1_local_pos_pub.publish(pos1)

            if (box4_catched == True) and (5.5 - (d1_y) < 1):
                pos1.pose.position.x = 58
                pos1.pose.position.y = d1_y - 0.4
                pos1.pose.position.z = 3
                edrone1_local_pos_pub.publish(pos1)

                if (d1_y < 5.5):
                    ofb_ctl.edrone1_land_mode()
                    ofb_ctl.edrone1_land_mode()
                if (d1_z < 2):
                    ofb_ctl.edrone1_grip_activate(False)
                    stateMt.gripper_check2.data = "False"
                    box4_catched = False
                    box4_delivered = True

        if (box4_delivered == True):

            if (stateMt.state2.mode == "AUTO.LAND") and (d1_y > 1):
                ofb_ctl.edrone1_offboard_set_mode()
                ofb_ctl.edrone1_grip_activate(False)
                pos1.pose.position.x = d1_x
                pos1.pose.position.y = d1_y
                pos1.pose.position.z = 5
                edrone1_local_pos_pub.publish(pos1)
            
            if (5 - (d1_z) < 0.5):
                pos1.pose.position.x = 0
                pos1.pose.position.y = 0
                pos1.pose.position.z = 3
                edrone1_local_pos_pub.publish(pos1)

            if (d1_x < 1) and (3 - (d1_z) < 0.5):
                ofb_ctl.edrone1_land_mode()

        rate.sleep()


if __name__ == '__main__':
    try:
        main()
    except rospy.ROSInterruptException:
        pass
    