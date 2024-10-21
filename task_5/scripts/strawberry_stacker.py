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
from sensor_msgs.msg import*
import math

class strawberry_stacker:

    def __init__(self):
        rospy.init_node("strawberry_stacker",anonymous = True)
        
    #services for drone 0 from here
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
            print("Service edrone0 grip activate call failed: %s"%e)

    #services for drone 1 from here
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
            print("Service edrone1 grip activate call failed: %s"%e)
    
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
    
    def spawn_cb(self,data):
        self.row = data

def main():
    stateMt = stateMonitor()
    ofb_ctl = strawberry_stacker()

    edrone0_local_pos_pub = rospy.Publisher('edrone0/mavros/setpoint_position/local',PoseStamped, queue_size=10)
    edrone1_local_pos_pub = rospy.Publisher('edrone1/mavros/setpoint_position/local',PoseStamped, queue_size=10)
    edrone0_local_vel_pub = rospy.Publisher('edrone0/mavros/setpoint_velocity/cmd_vel',Twist,queue_size=10)
    edrone1_local_vel_pub = rospy.Publisher('edrone1/mavros/setpoint_velocity/cmd_vel',Twist,queue_size=10)

    rate = rospy.Rate(20.0)

    pos0 = PoseStamped()
    pos0.pose.position.x = -1
    pos0.pose.position.y = 0
    pos0.pose.position.z = 3

    pos1 = PoseStamped()
    pos1.pose.position.x = -1
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

    #subscriber for spawn info
    rospy.Subscriber("spawn_info",UInt8,stateMt.spawn_cb)
    box_pos = []
    
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

    rows = [] #list for storing the row numbers published by spawn_info topic

    #flag variables for boxes delivered by edrone0
    box1_catched = False
    box2_catched = False
    box3_catched = False
    box4_catched = False
    box5_catched = False
    box6_catched = False

    box1_delivered = False
    box2_delivered = False
    box3_delivered = False
    box4_delivered = False
    box5_delivered = False
    box6_delivered = False

    #flag variables for boxes delivered by edrone1
    box7_catched = False
    box8_catched = False
    box9_catched = False
    box10_catched = False
    box11_catched = False
    box12_catched = False

    box7_delivered = False
    box8_delivered = False
    box9_delivered = False
    box10_delivered = False
    box11_delivered = False
    box12_delivered = False

    while not rospy.is_shutdown():
        #current positions of edrone0
        d0_x = stateMt.local_position0.pose.position.x
        d0_y = stateMt.local_position0.pose.position.y
        d0_z = stateMt.local_position0.pose.position.z

        #current positions of edrone1
        d1_x = stateMt.local_position1.pose.position.x
        d1_y = stateMt.local_position1.pose.position.y
        d1_z = stateMt.local_position1.pose.position.z
        #publishing the initial positions
        if (box6_delivered == False) and (int(d0_x) == 0) and (int(d0_y) == 0):
            edrone0_local_pos_pub.publish(pos0)

        if (box12_delivered == False) and (int(d1_x) == 0) and (int(d1_y) == 0):
            edrone1_local_pos_pub.publish(pos1)

        box_pos.append(stateMt.row.data) #continuously storing the row-numbers

        if len(box_pos)!=0:
            for i in box_pos:
                if i not in rows:
                    rows.append(i) #only storing the unique row-numbers to the list

        #code for image processing
        aruco_dict = aruco.Dictionary_get(aruco.DICT_5X5_250)
        parameters = aruco.DetectorParameters_create()
        mtx = np.array(stateMt.camera0.K)
        mtx = mtx.reshape(3,3)
        dst = np.array(stateMt.camera0.D)
        corners1,ids1,p1 = aruco.detectMarkers(stateMt.img1, aruco_dict, parameters = parameters, cameraMatrix = mtx, distCoeff = dst)
        corners2,ids2,p2 = aruco.detectMarkers(stateMt.img2, aruco_dict, parameters = parameters, cameraMatrix = mtx, distCoeff = dst)

        if len(rows) >= 2:
            if (box1_delivered == False):
                y1 = (rows[0]-1) * 4
                if (3 - (d0_z) < 0.5):
                    pos0.pose.position.x = -1
                    pos0.pose.position.y = y1
                    pos0.pose.position.z = 3
                    edrone0_local_pos_pub.publish(pos0)

                if (y1 - d0_y < 0.5):
                    pos0.pose.position.x = 4
                    pos0.pose.position.y = y1
                    pos0.pose.position.z = 3
                    edrone0_local_pos_pub.publish(pos0)
                
                if len(corners1)!=0 and (ids1[0][0] == 2):
                    rvecs,tvecs,_ = aruco.estimatePoseSingleMarkers(corners1,0.05,mtx,dst)
                    yaw = -1 * math.atan2(tvecs[0][0][0], tvecs[0][0][2])
                    angle = np.rad2deg(yaw)
                    pos0.pose.position.x = d0_x - 0.2
                    pos0.pose.position.y = y1
                    pos0.pose.position.z = 3
                    edrone0_local_pos_pub.publish(pos0)

                    if (0 < abs(angle) < 0.5):
                        ofb_ctl.edrone0_land_mode()
                    
                if (stateMt.gripper_check1.data == "True"):
                    ofb_ctl.edrone0_grip_activate(True)
                    box1_catched = True 
                
                if (box1_catched == True):
                    
                    if (stateMt.state1.mode == "AUTO.LAND") and (d0_y > -1):
                        ofb_ctl.edrone0_offboard_set_mode()
                        pos0.pose.position.x = d0_x            
                        pos0.pose.position.y = y1
                        pos0.pose.position.z = 3
                        edrone0_local_pos_pub.publish(pos0)
                
                    if (3 - d0_z) < 0.5:
                        pos0.pose.position.x = 15.5
                        pos0.pose.position.y = y1
                        pos0.pose.position.z = 3
                        edrone0_local_pos_pub.publish(pos0)

                    if (15 - d0_x) < 0.5:
                        pos0.pose.position.x = 15.5
                        pos0.pose.position.y = -6.5
                        pos0.pose.position.z = 3
                        edrone0_local_pos_pub.publish(pos0)
                    
                    if (abs(-6.5 - d0_y) < 0.2):
                        pos0.pose.position.x = 15.5
                        pos0.pose.position.y = -6.5
                        pos0.pose.position.z = 1
                        edrone0_local_pos_pub.publish(pos0)

                    if (d0_y < -1) and (d0_z < 2.3):
                        ofb_ctl.edrone0_land_mode()
                        ofb_ctl.edrone0_grip_activate(False)
                        stateMt.gripper_check1.data = "False"
                        box1_catched = False
                        box1_delivered = True
                            
            if (box1_delivered == True) and (box2_delivered == False):
                y1 = (rows[2]-1)*4
                y5 = y1 - 4
                if (stateMt.state1.mode == "AUTO.LAND") and (d0_y < -1):
                    ofb_ctl.edrone0_offboard_set_mode()
                    pos0.pose.position.x = d0_x
                    pos0.pose.position.y = d0_y
                    pos0.pose.position.z = 3
                    edrone0_local_pos_pub.publish(pos0)
        
                if (3 - (d0_z) < 0.5):
                    pos0.pose.position.x = 15
                    pos0.pose.position.y = y1
                    pos0.pose.position.z = 3
                    edrone0_local_pos_pub.publish(pos0)
                
                if (y1 - (d0_y) < 0.5):
                    pos0.pose.position.x = 1
                    pos0.pose.position.y = y1
                    pos0.pose.position.z = 3
                    edrone0_local_pos_pub.publish(pos0)
                
                if abs(1-d0_x)<0.5:
                    pos0.pose.position.x = 20
                    pos0.pose.position.y = y1
                    pos0.pose.position.z = 3
                    edrone0_local_pos_pub.publish(pos0)
                
                if (20 - d0_x) < 0.5:
                    pos0.pose.position.x = 20
                    pos0.pose.position.y = y5
                    pos0.pose.position.z = 3
                    edrone0_local_pos_pub.publish(pos0)
                
                if abs(y5 - d0_y)<0.5:
                    pos0.pose.position.x = 1
                    pos0.pose.position.y = y5
                    pos0.pose.position.z = 3
                    edrone0_local_pos_pub.publish(pos0)

                if len(corners1)!=0 and (ids1[0][0] == 2):
                    rvecs,tvecs,_ = aruco.estimatePoseSingleMarkers(corners1,0.05,mtx,dst)
                    yaw = -1 * math.atan2(tvecs[0][0][0], tvecs[0][0][2])
                    angle = np.rad2deg(yaw)
                    if (pos0.pose.position.x == 1) and (pos0.pose.position.y == y1):
                        pos0.pose.position.x = d0_x + 0.4
                        pos0.pose.position.y = y1
                        pos0.pose.position.z = 3
                        edrone0_local_pos_pub.publish(pos0)
                    
                    elif (pos0.pose.position.x == 20) and (pos0.pose.position.y == y1):
                        pos0.pose.position.x = d0_x - 0.4
                        pos0.pose.position.y = y1
                        pos0.pose.position.z = 3
                        edrone0_local_pos_pub.publish(pos0)
                    
                    elif (pos0.pose.position.x == 1) and (pos0.pose.position.y == y5):
                        pos0.pose.position.x = d0_x + 0.4
                        pos0.pose.position.y = y5
                        pos0.pose.position.z = 3
                        edrone0_local_pos_pub.publish(pos0)
                    
                    elif (pos0.pose.position.x == 20) and (pos0.pose.position.y == y5):
                        pos0.pose.position.x = d0_x - 0.4
                        pos0.pose.position.y = y5
                        pos0.pose.position.z = 3
                        edrone0_local_pos_pub.publish(pos0)

                    if (0 < abs(angle) < 0.5):
                        ofb_ctl.edrone0_land_mode()
                
                if (stateMt.gripper_check1.data == "True"):
                    ofb_ctl.edrone0_grip_activate(True)
                    box2_catched = True 
                
                if (box2_catched == True):
                    if (stateMt.state1.mode == "AUTO.LAND") and (d0_y > -1):
                        ofb_ctl.edrone0_offboard_set_mode()
                        pos0.pose.position.x = d0_x            
                        pos0.pose.position.y = d0_y
                        pos0.pose.position.z = 3
                        edrone0_local_pos_pub.publish(pos0)
                
                    if (3 - (d0_z) < 0.5):
                        pos0.pose.position.x = 15
                        pos0.pose.position.y = y5
                        pos0.pose.position.z = 3
                        edrone0_local_pos_pub.publish(pos0)
                    
                    if abs(15 - (d0_x)) < 0.5:
                        pos0.pose.position.x = 15
                        pos0.pose.position.y = -7.5
                        pos0.pose.position.z = 3
                        edrone0_local_pos_pub.publish(pos0)
                    
                    if (abs(-7.5 - d0_y) < 0.2):
                        pos0.pose.position.x = 15
                        pos0.pose.position.y = -7.5
                        pos0.pose.position.z = 1
                        edrone0_local_pos_pub.publish(pos0)
                       
                    if (d0_y < -1) and (d0_z < 2.3):
                        ofb_ctl.edrone0_land_mode()
                        ofb_ctl.edrone0_grip_activate(False)
                        stateMt.gripper_check1.data = "False"
                        box2_catched = False
                        box2_delivered = True
                
            if (box2_delivered == True) and (box3_delivered == False):
                y1 = (rows[4]-1)*4
                y5 = y1 - 4
                if (stateMt.state1.mode == "AUTO.LAND") and (d0_y < -1):
                    ofb_ctl.edrone0_offboard_set_mode()
                    ofb_ctl.edrone0_grip_activate(False)
                    pos0.pose.position.x = d0_x
                    pos0.pose.position.y = d0_y
                    pos0.pose.position.z = 3
                    edrone0_local_pos_pub.publish(pos0)
                
                if (3-(d0_z)<0.5):
                    pos0.pose.position.x = 15
                    pos0.pose.position.y = y1
                    pos0.pose.position.z = 3
                    edrone0_local_pos_pub.publish(pos0)
                
                if (y1-(d0_y)<0.5):
                    pos0.pose.position.x = 1
                    pos0.pose.position.y = y1
                    pos0.pose.position.z = 3
                    edrone0_local_pos_pub.publish(pos0)
                
                if abs(1-d0_x)<0.5:
                    pos0.pose.position.x = 20
                    pos0.pose.position.y = y1
                    pos0.pose.position.z = 3
                    edrone0_local_pos_pub.publish(pos0)
                
                if (20 - d0_x) < 0.5:
                    pos0.pose.position.x = 20
                    pos0.pose.position.y = y5
                    pos0.pose.position.z = 3
                    edrone0_local_pos_pub.publish(pos0)
                
                if abs(y5 - d0_y)<0.5:
                    pos0.pose.position.x = 1
                    pos0.pose.position.y = y5
                    pos0.pose.position.z = 3
                    edrone0_local_pos_pub.publish(pos0)
                
                if len(corners1)!=0 and (ids1[0][0] == 2):
                    rvecs,tvecs,_ = aruco.estimatePoseSingleMarkers(corners1,0.05,mtx,dst)
                    yaw = -1 * math.atan2(tvecs[0][0][0], tvecs[0][0][2])
                    angle = np.rad2deg(yaw)
                    if (pos0.pose.position.x == 1) and (pos0.pose.position.y == y1):
                        pos0.pose.position.x = d0_x + 0.4
                        pos0.pose.position.y = y1
                        pos0.pose.position.z = 3
                        edrone0_local_pos_pub.publish(pos0)
                    
                    elif (pos0.pose.position.x == 20) and (pos0.pose.position.y == y1):
                        pos0.pose.position.x = d0_x - 0.4
                        pos0.pose.position.y = y1
                        pos0.pose.position.z = 3
                        edrone0_local_pos_pub.publish(pos0)
                    
                    elif (pos0.pose.position.x == 1) and (pos0.pose.position.y == y5):
                        pos0.pose.position.x = d0_x - 0.4
                        pos0.pose.position.y = y5
                        pos0.pose.position.z = 3
                        edrone0_local_pos_pub.publish(pos0)
                    
                    elif (pos0.pose.position.x == 20) and (pos0.pose.position.y == y5):
                        pos0.pose.position.x = d0_x - 0.4
                        pos0.pose.position.y = y5
                        pos0.pose.position.z = 3
                        edrone0_local_pos_pub.publish(pos0)

                    if (0 < abs(angle) < 0.5):
                        ofb_ctl.edrone0_land_mode()
                
                if (stateMt.gripper_check1.data == "True"):
                    ofb_ctl.edrone0_grip_activate(True)
                    box3_catched = True 
                
                if (box3_catched == True):
                    
                    if (stateMt.state1.mode == "AUTO.LAND") and (d0_y > -1):
                        ofb_ctl.edrone0_offboard_set_mode()
                        pos0.pose.position.x = d0_x
                        pos0.pose.position.y = d0_y
                        pos0.pose.position.z = 3
                        edrone0_local_pos_pub.publish(pos0)

                    if (3 - (d0_z) < 0.5):
                        pos0.pose.position.x = 16
                        pos0.pose.position.y = y5
                        pos0.pose.position.z = 3
                        edrone0_local_pos_pub.publish(pos0)
                    
                    if abs(16 - (d0_x)) < 0.5:
                        pos0.pose.position.x = 16
                        pos0.pose.position.y = -6.5
                        pos0.pose.position.z = 3
                        edrone0_local_pos_pub.publish(pos0)
                    
                    if (abs(-6.5 - d0_y) < 0.2):
                        pos0.pose.position.x = 15
                        pos0.pose.position.y = -6.5
                        pos0.pose.position.z = 1
                        edrone0_local_pos_pub.publish(pos0)
                        
                    if (d0_y < -1) and (d0_z < 2.3):
                        ofb_ctl.edrone0_land_mode()
                        ofb_ctl.edrone0_grip_activate(False)
                        stateMt.gripper_check1.data = "False"
                        box3_catched = False
                        box3_delivered = True
                            

            if (box3_delivered == True) and (box4_delivered == False):
                y1 = (rows[6]-1)*4
                y5 = y1 - 4
                if (stateMt.state1.mode == "AUTO.LAND") and (d0_y < -1):
                    ofb_ctl.edrone0_offboard_set_mode()
                    ofb_ctl.edrone0_grip_activate(False)
                    pos0.pose.position.x = d0_x
                    pos0.pose.position.y = d0_y
                    pos0.pose.position.z = 3
                    edrone0_local_pos_pub.publish(pos0)
                
                if (3-(d0_z)<0.5):
                    pos0.pose.position.x = 15
                    pos0.pose.position.y = y1
                    pos0.pose.position.z = 3
                    edrone0_local_pos_pub.publish(pos0)
                
                if (y1 - (d0_y) < 0.5):
                    pos0.pose.position.x = 1
                    pos0.pose.position.y = y1
                    pos0.pose.position.z = 3
                    edrone0_local_pos_pub.publish(pos0)
                
                if abs(1-d0_x)<0.5:
                    pos0.pose.position.x = 20
                    pos0.pose.position.y = y1
                    pos0.pose.position.z = 3
                    edrone0_local_pos_pub.publish(pos0)
                
                if (20 - d0_x) < 0.5:
                    pos0.pose.position.x = 20
                    pos0.pose.position.y = y5
                    pos0.pose.position.z = 3
                    edrone0_local_pos_pub.publish(pos0)
                
                if abs(y5 - d0_y)<0.5:
                    pos0.pose.position.x = 1
                    pos0.pose.position.y = y5
                    pos0.pose.position.z = 3
                    edrone0_local_pos_pub.publish(pos0)
                
                if len(corners1)!=0 and (ids1[0][0] == 2):
                    rvecs,tvecs,_ = aruco.estimatePoseSingleMarkers(corners1,0.05,mtx,dst)
                    yaw = -1 * math.atan2(tvecs[0][0][0], tvecs[0][0][2])
                    angle = np.rad2deg(yaw)
                    if (pos0.pose.position.x == 1) and (pos0.pose.position.y == y1):
                        pos0.pose.position.x = d0_x + 0.4
                        pos0.pose.position.y = y1
                        pos0.pose.position.z = 3
                        edrone0_local_pos_pub.publish(pos0)
                    
                    elif (pos0.pose.position.x == 20) and (pos0.pose.position.y == y1):
                        pos0.pose.position.x = d0_x - 0.4
                        pos0.pose.position.y = y1
                        pos0.pose.position.z = 3
                        edrone0_local_pos_pub.publish(pos0)
                    
                    elif (pos0.pose.position.x == 1) and (pos0.pose.position.y == y5):
                        pos0.pose.position.x = d0_x + 0.4
                        pos0.pose.position.y = y5
                        pos0.pose.position.z = 3
                        edrone0_local_pos_pub.publish(pos0)
                    
                    elif (pos0.pose.position.x == 20) and (pos0.pose.position.y == y5):
                        pos0.pose.position.x = d0_x - 0.4
                        pos0.pose.position.y = y5
                        pos0.pose.position.z = 3
                        edrone0_local_pos_pub.publish(pos0)

                    if (0 < abs(angle) < 0.5):
                        ofb_ctl.edrone0_land_mode()
                
                if (stateMt.gripper_check1.data == "True"):
                    ofb_ctl.edrone0_grip_activate(True)
                    box4_catched = True
                
                if (box4_catched == True):
                    
                    if (stateMt.state1.mode == "AUTO.LAND") and (d0_y > -1):
                        ofb_ctl.edrone0_offboard_set_mode()
                        pos0.pose.position.x = d0_x
                        pos0.pose.position.y = d0_y
                        pos0.pose.position.z = 3
                        edrone0_local_pos_pub.publish(pos0)
                
                    if (3-(d0_z) < 0.5):
                        pos0.pose.position.x = 16
                        pos0.pose.position.y = y5
                        pos0.pose.position.z = 3
                        edrone0_local_pos_pub.publish(pos0)
                        
                    if abs(16 - d0_x) < 0.5:
                        pos0.pose.position.x = 16
                        pos0.pose.position.y = -7.5
                        pos0.pose.position.z = 3
                        edrone0_local_pos_pub.publish(pos0)
                    
                    if (abs(-7.5 - d0_y) < 0.2):
                        pos0.pose.position.x = 15
                        pos0.pose.position.y = -7.5
                        pos0.pose.position.z = 1
                        edrone0_local_pos_pub.publish(pos0)
                
                    if (d0_y < -1) and (d0_z < 2.3):
                        ofb_ctl.edrone0_land_mode()
                        ofb_ctl.edrone0_grip_activate(False)
                        stateMt.gripper_check1.data = "False"
                        box4_catched = False
                        box4_delivered = True
              
            if (box4_delivered == True) and (box5_delivered == False):
                y1 = 0
                if (stateMt.state1.mode == "AUTO.LAND") and (d0_y < -1):
                    ofb_ctl.edrone0_offboard_set_mode()
                    ofb_ctl.edrone0_grip_activate(False)
                    pos0.pose.position.x = d0_x
                    pos0.pose.position.y = d0_y
                    pos0.pose.position.z = 3
                    edrone0_local_pos_pub.publish(pos0)
                
                if (3-(d0_z) < 0.5):
                    pos0.pose.position.x = 17
                    pos0.pose.position.y = y1
                    pos0.pose.position.z = 3
                    edrone0_local_pos_pub.publish(pos0)
                
                if (d0_y > 0):
                    pos0.pose.position.x = 1
                    pos0.pose.position.y = y1
                    pos0.pose.position.z = 3
                    edrone0_local_pos_pub.publish(pos0)
                
                if (d0_x < 0):
                    pos0.pose.position.x = 20
                    pos0.pose.position.y = y1
                    pos0.pose.position.z = 3
                    edrone0_local_pos_pub.publish(pos0)
                
                if (20 - d0_x) < 0.5:
                    pos0.pose.position.x = 20
                    pos0.pose.position.y = y5
                    pos0.pose.position.z = 3
                    edrone0_local_pos_pub.publish(pos0)
                
                if abs(y5 - d0_y)<0.5:
                    pos0.pose.position.x = 1
                    pos0.pose.position.y = y5
                    pos0.pose.position.z = 3
                    edrone0_local_pos_pub.publish(pos0)
                
                if len(corners1)!=0 and (ids1[0][0] == 2):
                    rvecs,tvecs,_ = aruco.estimatePoseSingleMarkers(corners1,0.05,mtx,dst)
                    yaw = -1 * math.atan2(tvecs[0][0][0], tvecs[0][0][2])
                    angle = np.rad2deg(yaw)
                    if (pos0.pose.position.x == 1):
                        pos0.pose.position.x = d0_x + 0.4
                        pos0.pose.position.y = d0_y
                        pos0.pose.position.z = 3
                        edrone0_local_pos_pub.publish(pos0)
                    
                    elif (pos0.pose.position.x == 20):
                        pos0.pose.position.x = d0_x - 0.4
                        pos0.pose.position.y = d0_y
                        pos0.pose.position.z = 3
                        edrone0_local_pos_pub.publish(pos0)

                    if (0 < abs(angle) < 0.5):
                        ofb_ctl.edrone0_land_mode()
                
                if (stateMt.gripper_check1.data == "True"):
                    ofb_ctl.edrone0_grip_activate(True)
                    box5_catched = True
                
                if (box5_catched == True):
                    
                    if (stateMt.state1.mode == "AUTO.LAND") and (d0_y > -1):
                        ofb_ctl.edrone0_offboard_set_mode()
                        pos0.pose.position.x = d0_x
                        pos0.pose.position.y = d0_y
                        pos0.pose.position.z = 3
                        edrone0_local_pos_pub.publish(pos0)
                
                    if (3 - (d0_z) < 0.5):
                        pos0.pose.position.x = 17
                        pos0.pose.position.y = d0_y
                        pos0.pose.position.z = 3
                        edrone0_local_pos_pub.publish(pos0)
                    
                    if abs(17 - d0_x) < 1:
                        pos0.pose.position.x = 17
                        pos0.pose.position.y = -6.5
                        pos0.pose.position.z = 3
                        edrone0_local_pos_pub.publish(pos0)
                    
                    if (abs(-6.5 - d0_y) < 0.2):
                        pos0.pose.position.x = 15
                        pos0.pose.position.y = -6.5
                        pos0.pose.position.z = 3
                        edrone0_local_pos_pub.publish(pos0)
                 
                    if (d0_y < -1) and (d0_z < 2.3):
                        ofb_ctl.edrone0_land_mode()
                        ofb_ctl.edrone0_grip_activate(False)
                        stateMt.gripper_check1.data = "False"
                        box5_catched = False
                        box5_delivered = True
                    
            if (box5_delivered == True) and (box6_delivered == False):
                y1 = (rows[8]-1)*4
                if (stateMt.state1.mode == "AUTO.LAND") and (d0_y < -1):
                    ofb_ctl.edrone0_offboard_set_mode()
                    ofb_ctl.edrone0_grip_activate(False)
                    pos0.pose.position.x = d0_x
                    pos0.pose.position.y = d0_y
                    pos0.pose.position.z = 3
                    edrone0_local_pos_pub.publish(pos0)

                if (3 - (d0_z) < 0.5):
                    pos0.pose.position.x = 17
                    pos0.pose.position.y = y1
                    pos0.pose.position.z = 3
                    edrone0_local_pos_pub.publish(pos0)
                
                if (y1 - (d0_y) < 0.5):
                    pos0.pose.position.x = 1
                    pos0.pose.position.y = y1
                    pos0.pose.position.z = 3
                    edrone0_local_pos_pub.publish(pos0)
                
                if (d0_x < 0):
                    pos0.pose.position.x = 20
                    pos0.pose.position.y = y1
                    pos0.pose.position.z = 3
                    edrone0_local_pos_pub.publish(pos0)
                
                if (20 - d0_x) < 0.5:
                    pos0.pose.position.x = 20
                    pos0.pose.position.y = y5
                    pos0.pose.position.z = 3
                    edrone0_local_pos_pub.publish(pos0)
                
                if abs(y5 - d0_y)<0.5:
                    pos0.pose.position.x = 1
                    pos0.pose.position.y = y5
                    pos0.pose.position.z = 3
                    edrone0_local_pos_pub.publish(pos0)
                                
                if len(corners1)!=0 and (ids1[0][0] == 2):
                    rvecs,tvecs,_ = aruco.estimatePoseSingleMarkers(corners1,0.05,mtx,dst)
                    yaw = -1 * math.atan2(tvecs[0][0][0], tvecs[0][0][2])
                    angle = np.rad2deg(yaw)
                    if (pos0.pose.position.x == 1):
                        pos0.pose.position.x = d0_x + 0.4
                        pos0.pose.position.y = d0_y
                        pos0.pose.position.z = 3
                        edrone0_local_pos_pub.publish(pos0)
                    
                    elif (pos0.pose.position.x == 20):
                        pos0.pose.position.x = d0_x - 0.4
                        pos0.pose.position.y = d0_y
                        pos0.pose.position.z = 3
                        edrone0_local_pos_pub.publish(pos0)

                    if (0 < abs(angle) < 0.5):
                        ofb_ctl.edrone0_land_mode()
                
                if (stateMt.gripper_check1.data == "True"):
                    ofb_ctl.edrone0_grip_activate(True)
                    box6_catched = True
                
                if (box6_catched == True):
                    if (stateMt.state1.mode == "AUTO.LAND") and (d0_y > -1):
                        ofb_ctl.edrone0_offboard_set_mode()
                        pos0.pose.position.x = d0_x
                        pos0.pose.position.y = d0_y
                        pos0.pose.position.z = 3
                        edrone0_local_pos_pub.publish(pos0)
                
                    if (3 - (d0_z) < 0.5):
                        pos0.pose.position.x = 17
                        pos0.pose.position.y = d0_y
                        pos0.pose.position.z = 3
                        edrone0_local_pos_pub.publish(pos0)
                    
                    if (abs(17 - d0_x) < 0.5):
                        pos0.pose.position.x = 17
                        pos0.pose.position.y = -7.5
                        pos0.pose.position.z = 3
                        edrone0_local_pos_pub.publish(pos0)

                    if (abs(-7.5 - d0_y) < 0.2):
                        pos0.pose.position.x = 15
                        pos0.pose.position.y = -7.5
                        pos0.pose.position.z = 1
                        edrone0_local_pos_pub.publish(pos0)
    
                    if (d0_y < -1) and (d0_z < 2.3):
                        ofb_ctl.edrone0_land_mode()
                        ofb_ctl.edrone0_grip_activate(False)
                        stateMt.gripper_check1.data = "False"
                        box6_catched = False
                        box6_delivered = True
                            
            if (box6_delivered == True):
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
                
                if (d0_x < 1) and (5 - (d0_z) < 0.5):
                    ofb_ctl.edrone0_land_mode()
                
        # '''algorithm for drone 2 starts here'''
    
            if (box7_delivered == False):
                y2 = ((rows[1]-1)*4) - 60
                if (3 - (d1_z) < 0.5):
                    pos1.pose.position.x = -1
                    pos1.pose.position.y = y2
                    pos1.pose.position.z = 3
                    edrone1_local_pos_pub.publish(pos1)
                
                if abs(y2 - (d1_y)) < 0.5:
                    pos1.pose.position.x = 5
                    pos1.pose.position.y = y2
                    pos1.pose.position.z = 3
                    edrone1_local_pos_pub.publish(pos1)

                if (len(corners2) != 0) and (ids2[0][0] == 1):
                    rvecs,tvecs,_ = aruco.estimatePoseSingleMarkers(corners2,0.05,mtx,dst)
                    yaw = -1 * math.atan2(tvecs[0][0][0], tvecs[0][0][2])
                    angle = np.rad2deg(yaw)
                    pos1.pose.position.x =  d1_x - 0.2
                    pos1.pose.position.y = y2
                    pos1.pose.position.z = 3
                    edrone1_local_pos_pub.publish(pos1)

                    if (0 < abs(angle) < 0.5):
                        ofb_ctl.edrone1_land_mode()

                if (stateMt.gripper_check2.data == "True"):
                    ofb_ctl.edrone1_grip_activate(True)
                    box7_catched = True
                
                if (box7_catched == True):
                    
                    if (stateMt.state2.mode == "AUTO.LAND") and (d1_y < 0):
                        ofb_ctl.edrone1_offboard_set_mode()
                        pos1.pose.position.x = d1_x
                        pos1.pose.position.y = y2
                        pos1.pose.position.z = 3
                        edrone1_local_pos_pub.publish(pos1)
                
                    if (3-(d1_z) < 0.5):
                        pos1.pose.position.x = 58
                        pos1.pose.position.y = y2
                        pos1.pose.position.z = 3
                        edrone1_local_pos_pub.publish(pos1)
                    
                    if (58 - (d1_x) < 0.5):
                        pos1.pose.position.x = 58
                        pos1.pose.position.y = 4.5
                        pos1.pose.position.z = 3
                        edrone1_local_pos_pub.publish(pos1)

                    if abs(4.5 - d1_y) < 0.2:
                        pos1.pose.position.x = 58
                        pos1.pose.position.y = 4.5
                        pos1.pose.position.z = 1
                        edrone1_local_pos_pub.publish(pos1)
                        
                    if  (d1_y > 0) and (d1_z < 2.3):
                        ofb_ctl.edrone1_land_mode()
                        ofb_ctl.edrone1_grip_activate(False)
                        stateMt.gripper_check2.data = "False"
                        box7_catched = False
                        box7_delivered = True
                            
                
            if (box7_delivered == True) and (box8_delivered == False):
                y2 = ((rows[3]-1)*4) - 60
                y3 = y2-4
                y4 = y3-4
                if (stateMt.state2.mode == "AUTO.LAND") and (d1_y > 0): 
                    ofb_ctl.edrone1_offboard_set_mode()
                    ofb_ctl.edrone1_grip_activate(False)
                    pos1.pose.position.x = d1_x
                    pos1.pose.position.y = d1_y
                    pos1.pose.position.z = 3
                    edrone1_local_pos_pub.publish(pos1)
                
                if (3 - (d1_z) < 0.5):
                    pos1.pose.position.x = 57.5
                    pos1.pose.position.y = y2
                    pos1.pose.position.z = 3
                    edrone1_local_pos_pub.publish(pos1)
                
                if abs(y2 - (d1_y)) < 0.5:
                    pos1.pose.position.x = 2
                    pos1.pose.position.y = y2
                    pos1.pose.position.z = 3
                    edrone1_local_pos_pub.publish(pos1)
                
                if abs(1-d1_x)<0.5:
                    pos1.pose.position.x = 0
                    pos1.pose.position.y = y3
                    pos1.pose.position.z = 3
                    edrone1_local_pos_pub.publish(pos1)

                if abs(y3 - d1_y)<0.5:
                    pos1.pose.position.x = 20
                    pos1.pose.position.y = y3
                    pos1.pose.position.z = 3
                    edrone1_local_pos_pub.publish(pos1)
                
                if (20 - d0_x) < 0.5 and (pos1.pose.position.y == y3):
                    pos1.pose.position.x = 20
                    pos1.pose.position.y = y4
                    pos1.pose.position.z = 3
                    edrone1_local_pos_pub.publish(pos1)
                
                if abs(y4 - d0_y) < 0.5:
                    pos1.pose.position.x = 2
                    pos1.pose.position.y = y4
                    pos1.pose.position.z = 3
                    edrone1_local_pos_pub.publish(pos1)
                
                if (len(corners2) != 0) and (ids2[0][0] == 1):
                    rvecs,tvecs,_ = aruco.estimatePoseSingleMarkers(corners2,0.05,mtx,dst)
                    yaw = -1 * math.atan2(tvecs[0][0][0], tvecs[0][0][2])
                    angle = np.rad2deg(yaw)
                    if (pos1.pose.position.x == 2) and (pos1.pose.position.y == y2):
                        pos1.pose.position.x = d1_x + 0.4
                        pos1.pose.position.y = y2
                        pos1.pose.position.z = 3
                        edrone1_local_pos_pub.publish(pos1)
                    
                    elif (pos1.pose.position.x == 20) and (pos1.pose.position.y == y2):
                        pos1.pose.position.x = d1_x - 0.4
                        pos1.pose.position.y = y2
                        pos1.pose.position.z = 3
                        edrone1_local_pos_pub.publish(pos1)
                    
                    elif (pos1.pose.position.x == 2) and (pos1.pose.position.y == y3):
                        pos1.pose.position.x = d1_x + 0.4
                        pos1.pose.position.y = y3
                        pos1.pose.position.z = 3
                        edrone1_local_pos_pub.publish(pos1)
                    
                    elif (pos1.pose.position.x == 20) and (pos1.pose.position.y == y3):
                        pos1.pose.position.x = d1_x - 0.4
                        pos1.pose.position.y = y3
                        pos1.pose.position.z = 3
                        edrone1_local_pos_pub.publish(pos1)
                    
                    elif (pos1.pose.position.x == 2) and (pos1.pose.position.y == y4):
                        pos1.pose.position.x = d1_x + 0.4
                        pos1.pose.position.y = y4
                        pos1.pose.position.z = 3
                        edrone1_local_pos_pub.publish(pos1)

                    if (0 < abs(angle) < 0.5):
                        ofb_ctl.edrone1_land_mode()
                
                if (stateMt.gripper_check2.data == "True"):
                    ofb_ctl.edrone1_grip_activate(True)
                    box8_catched = True

                if (box8_catched == True):
                    if (stateMt.state2.mode == "AUTO.LAND") and (d1_y < 0):
                        ofb_ctl.edrone1_offboard_set_mode()
                        pos1.pose.position.x = d1_x
                        pos1.pose.position.y = d1_y
                        pos1.pose.position.z = 3
                        edrone1_local_pos_pub.publish(pos1)
                
                    if (3-(d1_z) < 0.5):
                        pos1.pose.position.x = 58
                        pos1.pose.position.y = y2
                        pos1.pose.position.z = 3
                        edrone1_local_pos_pub.publish(pos1)

                    if (57.5 - (d1_x) < 0.5):
                        pos1.pose.position.x = 58
                        pos1.pose.position.y = 5.5
                        pos1.pose.position.z = 3
                        edrone1_local_pos_pub.publish(pos1)
                    
                    if abs(5.5 - d1_y) < 0.2:
                        pos1.pose.position.x = 58
                        pos1.pose.position.y = 5.5
                        pos1.pose.position.z = 1
                        edrone1_local_pos_pub.publish(pos1)

                    if (d1_y > 0) and (d1_z < 2.3):
                        ofb_ctl.edrone1_land_mode()
                        ofb_ctl.edrone1_grip_activate(False)
                        stateMt.gripper_check2.data = "False"
                        box8_catched = False
                        box8_delivered = True
                            
                    
            if (box8_delivered == True) and (box9_delivered == False):
                y2 = ((rows[5]-1)*4)-60
                y3 = y2 - 4
                y4 = y3 - 4
                if (stateMt.state2.mode == "AUTO.LAND") and (d1_y > 0): 
                    ofb_ctl.edrone1_offboard_set_mode()
                    ofb_ctl.edrone1_grip_activate(False)
                    pos1.pose.position.x = d1_x
                    pos1.pose.position.y = d1_y
                    pos1.pose.position.z = 4
                    edrone1_local_pos_pub.publish(pos1)

                if (4 - (d1_z) < 0.5):
                    pos1.pose.position.x = 58
                    pos1.pose.position.y = y2
                    pos1.pose.position.z = 3
                    edrone1_local_pos_pub.publish(pos1)

                if abs(y2 - (d1_y)) < 0.5:
                    pos1.pose.position.x = 2
                    pos1.pose.position.y = y2
                    pos1.pose.position.z = 3
                    edrone1_local_pos_pub.publish(pos1)
                
                if abs(1-d1_x)<0.5:
                    pos1.pose.position.x = 0
                    pos1.pose.position.y = y3
                    pos1.pose.position.z = 3
                    edrone1_local_pos_pub.publish(pos1)

                if abs(y3 - d1_y)<0.5:
                    pos1.pose.position.x = 20
                    pos1.pose.position.y = y3
                    pos1.pose.position.z = 3
                    edrone1_local_pos_pub.publish(pos1)
                
                if (20 - d0_x) < 0.5 and (pos1.pose.position.y == y3):
                    pos1.pose.position.x = 20
                    pos1.pose.position.y = y4
                    pos1.pose.position.z = 3
                    edrone1_local_pos_pub.publish(pos1)
                
                if abs(y4 - d0_y) < 0.5:
                    pos1.pose.position.x = 2
                    pos1.pose.position.y = y4
                    pos1.pose.position.z = 3
                    edrone1_local_pos_pub.publish(pos1)
                
                if (len(corners2) != 0) and (ids2[0][0] == 1):
                    rvecs,tvecs,_ = aruco.estimatePoseSingleMarkers(corners2,0.05,mtx,dst)
                    yaw = -1 * math.atan2(tvecs[0][0][0], tvecs[0][0][2])
                    angle = np.rad2deg(yaw)
                    if (pos1.pose.position.x == 2) and (pos1.pose.position.y == y2):
                        pos1.pose.position.x = d1_x + 0.4
                        pos1.pose.position.y = y2
                        pos1.pose.position.z = 3
                        edrone1_local_pos_pub.publish(pos1)
                    
                    elif (pos1.pose.position.x == 20) and (pos1.pose.position.y == y2):
                        pos1.pose.position.x = d1_x - 0.4
                        pos1.pose.position.y = y2
                        pos1.pose.position.z = 3
                        edrone1_local_pos_pub.publish(pos1)
                    
                    elif (pos1.pose.position.x == 2) and (pos1.pose.position.y == y3):
                        pos1.pose.position.x = d1_x + 0.4
                        pos1.pose.position.y = y3
                        pos1.pose.position.z = 3
                        edrone1_local_pos_pub.publish(pos1)
                    
                    elif (pos1.pose.position.x == 20) and (pos1.pose.position.y == y3):
                        pos1.pose.position.x = d1_x - 0.4
                        pos1.pose.position.y = y3
                        pos1.pose.position.z = 3
                        edrone1_local_pos_pub.publish(pos1)
                    
                    elif (pos1.pose.position.x == 2) and (pos1.pose.position.y == y4):
                        pos1.pose.position.x = d1_x + 0.4
                        pos1.pose.position.y = y4
                        pos1.pose.position.z = 3
                        edrone1_local_pos_pub.publish(pos1)

                    if (0 < abs(angle) < 0.5):
                        ofb_ctl.edrone1_land_mode()

                if (stateMt.gripper_check2.data == "True"):
                    ofb_ctl.edrone1_grip_activate(True)
                    box9_catched = True
                
                if (box9_catched == True):
                    
                    if (stateMt.state2.mode == "AUTO.LAND") and (d1_y < 0):
                        ofb_ctl.edrone1_offboard_set_mode()
                        pos1.pose.position.x = d1_x
                        pos1.pose.position.y = d1_y
                        pos1.pose.position.z = 3
                        edrone1_local_pos_pub.publish(pos1)
                
                    if (3-(d1_z) < 0.5):
                        pos1.pose.position.x = 59
                        pos1.pose.position.y = y2
                        pos1.pose.position.z = 3
                        edrone1_local_pos_pub.publish(pos1)
                    
                    if (59 - (d1_x) < 0.5):
                        pos1.pose.position.x = 59
                        pos1.pose.position.y = 4.5
                        pos1.pose.position.z = 3
                        edrone1_local_pos_pub.publish(pos1)

                    if abs(4.5 - d1_y) < 0.2:
                        pos1.pose.position.x = 58
                        pos1.pose.position.y = 4.5
                        pos1.pose.position.z = 1
                        edrone1_local_pos_pub.publish(pos1)
                   
                    if (d1_y > 0) and (d1_z < 2.3):
                        ofb_ctl.edrone1_land_mode()
                        ofb_ctl.edrone1_grip_activate(False)
                        stateMt.gripper_check2.data = "False"
                        box9_catched = False
                        box9_delivered = True
                            
            if (box9_delivered == True) and (box10_delivered == False):
                y2 = ((rows[7]-1)*4) - 60
                y3 = y2 - 4
                y4 = y3 - 4
                if (stateMt.state2.mode == "AUTO.LAND") and (d1_y > 1): 
                    ofb_ctl.edrone1_offboard_set_mode()
                    ofb_ctl.edrone1_grip_activate(False)
                    pos1.pose.position.x = d1_x
                    pos1.pose.position.y = d1_y
                    pos1.pose.position.z = 5
                    edrone1_local_pos_pub.publish(pos1)
                
                if (5 - (d1_z) < 0.5):
                    pos1.pose.position.x = 59
                    pos1.pose.position.y = y2
                    pos1.pose.position.z = 3
                    edrone1_local_pos_pub.publish(pos1)
                
                if abs(y2 - (d1_y)) < 0.5:
                    pos1.pose.position.x = 2
                    pos1.pose.position.y = y2
                    pos1.pose.position.z = 3
                    edrone1_local_pos_pub.publish(pos1)
                
                if (d1_x < 0):
                    pos1.pose.position.x = 0
                    pos1.pose.position.y = y3
                    pos1.pose.position.z = 3
                    edrone1_local_pos_pub.publish(pos1)

                if abs(y3 - d1_y)<0.5:
                    pos1.pose.position.x = 20
                    pos1.pose.position.y = y3
                    pos1.pose.position.z = 3
                    edrone1_local_pos_pub.publish(pos1)
                
                if (20 - d0_x) < 0.5 and (pos1.pose.position.y == y3):
                    pos1.pose.position.x = 20
                    pos1.pose.position.y = y4
                    pos1.pose.position.z = 3
                    edrone1_local_pos_pub.publish(pos1)
                
                if abs((y3-4) - d0_y) < 0.5:
                    pos1.pose.position.x = 2
                    pos1.pose.position.y = y4
                    pos1.pose.position.z = 3
                    edrone1_local_pos_pub.publish(pos1)
                
                if (len(corners2) != 0) and (ids2[0][0] == 1):
                    rvecs,tvecs,_ = aruco.estimatePoseSingleMarkers(corners2,0.05,mtx,dst)
                    yaw = -1 * math.atan2(tvecs[0][0][0], tvecs[0][0][2])
                    angle = np.rad2deg(yaw)
                    if (pos1.pose.position.x == 2) and (pos1.pose.position.y == y2):
                        pos1.pose.position.x = d1_x + 0.4
                        pos1.pose.position.y = y2
                        pos1.pose.position.z = 3
                        edrone1_local_pos_pub.publish(pos1)
                    
                    elif (pos1.pose.position.x == 20) and (pos1.pose.position.y == y2):
                        pos1.pose.position.x = d1_x - 0.4
                        pos1.pose.position.y = y2
                        pos1.pose.position.z = 3
                        edrone1_local_pos_pub.publish(pos1)
                    
                    elif (pos1.pose.position.x == 2) and (pos1.pose.position.y == y3):
                        pos1.pose.position.x = d1_x + 0.4
                        pos1.pose.position.y = y3
                        pos1.pose.position.z = 3
                        edrone1_local_pos_pub.publish(pos1)
                    
                    elif (pos1.pose.position.x == 20) and (pos1.pose.position.y == y3):
                        pos1.pose.position.x = d1_x - 0.4
                        pos1.pose.position.y = y3
                        pos1.pose.position.z = 3
                        edrone1_local_pos_pub.publish(pos1)
                    
                    elif (pos1.pose.position.x == 2) and (pos1.pose.position.y == y4):
                        pos1.pose.position.x = d1_x + 0.4
                        pos1.pose.position.y = y4
                        pos1.pose.position.z = 3
                        edrone1_local_pos_pub.publish(pos1)

                    if (0 < abs(angle) < 0.5):
                        ofb_ctl.edrone1_land_mode()
           
                if (stateMt.gripper_check2.data == "True"):
                    ofb_ctl.edrone1_grip_activate(True)
                    box10_catched = True
                
                if (box10_catched == True):
                    
                    if (stateMt.state2.mode == "AUTO.LAND") and (d1_y < 0):
                        ofb_ctl.edrone1_offboard_set_mode()
                        pos1.pose.position.x = d1_x
                        pos1.pose.position.y = d1_y
                        pos1.pose.position.z = 3
                        edrone1_local_pos_pub.publish(pos1)
                
                    if (3-(d1_z) < 0.5):
                        pos1.pose.position.x = 59
                        pos1.pose.position.y = y2
                        pos1.pose.position.z = 3
                        edrone1_local_pos_pub.publish(pos1)
                    
                    if (59 - (d1_x) < 0.5):
                        pos1.pose.position.x = 59
                        pos1.pose.position.y = 5.5
                        pos1.pose.position.z = 3
                        edrone1_local_pos_pub.publish(pos1)

                    if abs(5.5 - d1_y) < 0.2:
                        pos1.pose.position.x = 58
                        pos1.pose.position.y = 5.5
                        pos1.pose.position.z = 1
                        edrone1_local_pos_pub.publish(pos1)

                    if (d1_y > 0) and (d1_z < 2):
                        ofb_ctl.edrone1_land_mode()
                        ofb_ctl.edrone1_grip_activate(False)
                        stateMt.gripper_check2.data = "False"
                        box10_catched = False
                        box10_delivered = True
                            
            
            if (box10_delivered == True) and (box11_delivered == False):
                y2 = ((rows[0]-1)*4) - 60
                y3 = y2 - 4
                y4 = y3 - 4
                if (stateMt.state2.mode == "AUTO.LAND") and (d1_y > 1): 
                    ofb_ctl.edrone1_offboard_set_mode()
                    ofb_ctl.edrone1_grip_activate(False)
                    pos1.pose.position.x = d1_x
                    pos1.pose.position.y = d1_y
                    pos1.pose.position.z = 5
                    edrone1_local_pos_pub.publish(pos1)
                
                if (5 - (d1_z) < 0.5):
                    pos1.pose.position.x = 59
                    pos1.pose.position.y = y2
                    pos1.pose.position.z = 3
                    edrone1_local_pos_pub.publish(pos1)
                
                if abs(y2 - (d1_y)) < 0.5:
                    pos1.pose.position.x = 2
                    pos1.pose.position.y = y2
                    pos1.pose.position.z = 3
                    edrone1_local_pos_pub.publish(pos1)
                
                if (d1_x < 0):
                    pos1.pose.position.x = 0
                    pos1.pose.position.y = y3
                    pos1.pose.position.z = 3
                    edrone1_local_pos_pub.publish(pos1)

                if abs(y3 - d1_y)<0.5:
                    pos1.pose.position.x = 20
                    pos1.pose.position.y = y3
                    pos1.pose.position.z = 3
                    edrone1_local_pos_pub.publish(pos1)
                
                if (20 - d0_x) < 0.5 and (pos1.pose.position.y == y3):
                    pos1.pose.position.x = 20
                    pos1.pose.position.y = y4
                    pos1.pose.position.z = 3
                    edrone1_local_pos_pub.publish(pos1)
                
                if abs(y4 - d0_y) < 0.5:
                    pos1.pose.position.x = 2
                    pos1.pose.position.y = y4
                    pos1.pose.position.z = 3
                    edrone1_local_pos_pub.publish(pos1)
                
                if (len(corners2) != 0) and (ids2[0][0] == 1):
                    rvecs,tvecs,_ = aruco.estimatePoseSingleMarkers(corners2,0.05,mtx,dst)
                    yaw = -1 * math.atan2(tvecs[0][0][0], tvecs[0][0][2])
                    angle = np.rad2deg(yaw)
                    if (pos1.pose.position.x == 2) and (pos1.pose.position.y == y2):
                        pos1.pose.position.x = d1_x + 0.4
                        pos1.pose.position.y = y2
                        pos1.pose.position.z = 3
                        edrone1_local_pos_pub.publish(pos1)
                    
                    elif (pos1.pose.position.x == 20) and (pos1.pose.position.y == y2):
                        pos1.pose.position.x = d1_x - 0.4
                        pos1.pose.position.y = y2
                        pos1.pose.position.z = 3
                        edrone1_local_pos_pub.publish(pos1)
                    
                    elif (pos1.pose.position.x == 2) and (pos1.pose.position.y == y3):
                        pos1.pose.position.x = d1_x + 0.4
                        pos1.pose.position.y = y3
                        pos1.pose.position.z = 3
                        edrone1_local_pos_pub.publish(pos1)
                    
                    elif (pos1.pose.position.x == 20) and (pos1.pose.position.y == y3):
                        pos1.pose.position.x = d1_x - 0.4
                        pos1.pose.position.y = y3
                        pos1.pose.position.z = 3
                        edrone1_local_pos_pub.publish(pos1)
                    
                    elif (pos1.pose.position.x == 2) and (pos1.pose.position.y == y4):
                        pos1.pose.position.x = d1_x + 0.4
                        pos1.pose.position.y = y4
                        pos1.pose.position.z = 3
                        edrone1_local_pos_pub.publish(pos1)

                    if (0 < abs(angle) < 0.5):
                        ofb_ctl.edrone1_land_mode()
                
                if (stateMt.gripper_check2.data == "True"):
                    ofb_ctl.edrone1_grip_activate(True)
                    box11_catched = True
                
                if (box11_catched == True):
                    
                    if (stateMt.state2.mode == "AUTO.LAND") and (d1_y < 0):
                        ofb_ctl.edrone1_offboard_set_mode()
                        pos1.pose.position.x = d1_x
                        pos1.pose.position.y = y2
                        pos1.pose.position.z = 3
                        edrone1_local_pos_pub.publish(pos1)
                
                    if (3-(d1_z) < 0.5):
                        pos1.pose.position.x = 60
                        pos1.pose.position.y = y2
                        pos1.pose.position.z = 3
                        edrone1_local_pos_pub.publish(pos1)
                    
                    if (60 - (d1_x) < 0.5):
                        pos1.pose.position.x = 60
                        pos1.pose.position.y = 4.5
                        pos1.pose.position.z = 3
                        edrone1_local_pos_pub.publish(pos1)

                    if abs(4.5 - d1_y) < 0.2:
                        pos1.pose.position.x = 58
                        pos1.pose.position.y = 4.5
                        pos1.pose.position.z = 1
                        edrone1_local_pos_pub.publish(pos1)

                    if (d1_y > 0) and (d1_z < 2):
                        ofb_ctl.edrone1_land_mode()
                        ofb_ctl.edrone1_grip_activate(False)
                        stateMt.gripper_check2.data = "False"
                        box11_catched = False
                        box11_delivered = True
                            
                
            if (box11_delivered) == True and (box12_delivered == False):
                y2 = ((rows[2]-1)*4) - 60
                y3 = y2 - 4
                y4 = y3 - 4
                if (stateMt.state2.mode == "AUTO.LAND") and (d1_y > 0): 
                    ofb_ctl.edrone1_offboard_set_mode()
                    ofb_ctl.edrone1_grip_activate(False)
                    pos1.pose.position.x = d1_x
                    pos1.pose.position.y = d1_y
                    pos1.pose.position.z = 5
                    edrone1_local_pos_pub.publish(pos1)
                
                if (5 - (d1_z) < 0.5):
                    pos1.pose.position.x = 60
                    pos1.pose.position.y = y2
                    pos1.pose.position.z = 3
                    edrone1_local_pos_pub.publish(pos1)
                
                if abs(y2 - (d1_y)) < 0.5:
                    pos1.pose.position.x = 1
                    pos1.pose.position.y = y2
                    pos1.pose.position.z = 3
                    edrone1_local_pos_pub.publish(pos1)
                
                if (d1_x < 0):
                    pos1.pose.position.x = 2
                    pos1.pose.position.y = y3
                    pos1.pose.position.z = 3
                    edrone1_local_pos_pub.publish(pos1)

                if abs(y3 - d1_y)<0.5:
                    pos1.pose.position.x = 20
                    pos1.pose.position.y = y3
                    pos1.pose.position.z = 3
                    edrone1_local_pos_pub.publish(pos1)
                
                if (20 - d0_x) < 0.5 and (pos1.pose.position.y == y3):
                    pos1.pose.position.x = 20
                    pos1.pose.position.y = y4
                    pos1.pose.position.z = 3
                    edrone1_local_pos_pub.publish(pos1)
                
                if abs(y4 - d0_y) < 0.5:
                    pos1.pose.position.x = 2
                    pos1.pose.position.y = y4
                    pos1.pose.position.z = 3
                    edrone1_local_pos_pub.publish(pos1)
                
                if (len(corners2) != 0) and (ids2[0][0] == 1):
                    rvecs,tvecs,_ = aruco.estimatePoseSingleMarkers(corners2,0.05,mtx,dst)
                    yaw = -1 * math.atan2(tvecs[0][0][0], tvecs[0][0][2])
                    angle = np.rad2deg(yaw)
                    if (pos1.pose.position.x == 2) and (pos1.pose.position.y == y2):
                        pos1.pose.position.x = d1_x + 0.4
                        pos1.pose.position.y = y2
                        pos1.pose.position.z = 3
                        edrone1_local_pos_pub.publish(pos1)
                    
                    elif (pos1.pose.position.x == 20) and (pos1.pose.position.y == y2):
                        pos1.pose.position.x = d1_x - 0.4
                        pos1.pose.position.y = y2
                        pos1.pose.position.z = 3
                        edrone1_local_pos_pub.publish(pos1)
                    
                    elif (pos1.pose.position.x == 2) and (pos1.pose.position.y == y3):
                        pos1.pose.position.x = d1_x + 0.4
                        pos1.pose.position.y = y3
                        pos1.pose.position.z = 3
                        edrone1_local_pos_pub.publish(pos1)
                    
                    elif (pos1.pose.position.x == 20) and (pos1.pose.position.y == y3):
                        pos1.pose.position.x = d1_x - 0.4
                        pos1.pose.position.y = y3
                        pos1.pose.position.z = 3
                        edrone1_local_pos_pub.publish(pos1)
                    
                    elif (pos1.pose.position.x == 2) and (pos1.pose.position.y == y4):
                        pos1.pose.position.x = d1_x + 0.4
                        pos1.pose.position.y = y4
                        pos1.pose.position.z = 3
                        edrone1_local_pos_pub.publish(pos1)

                    if (0 < abs(angle) < 0.5):
                        ofb_ctl.edrone1_land_mode()
                
                if (stateMt.gripper_check2.data == "True"):
                    ofb_ctl.edrone1_grip_activate(True)
                    box12_catched = True
                
                if (box12_catched == True):
                    
                    if (stateMt.state2.mode == "AUTO.LAND") and (d1_y < 0):
                        ofb_ctl.edrone1_offboard_set_mode()
                        pos1.pose.position.x = d1_x
                        pos1.pose.position.y = y2
                        pos1.pose.position.z = 3
                        edrone1_local_pos_pub.publish(pos1)
                
                    if (3-(d1_z) < 0.5):
                        pos1.pose.position.x = 60
                        pos1.pose.position.y = y2
                        pos1.pose.position.z = 3
                        edrone1_local_pos_pub.publish(pos1)
                    
                    if (60 - (d1_x) < 0.5):
                        pos1.pose.position.x = 60
                        pos1.pose.position.y = 5.5
                        pos1.pose.position.z = 3
                        edrone1_local_pos_pub.publish(pos1)
                    
                    if abs(5.5 - d1_y) < 0.2:
                        pos1.pose.position.x = 58
                        pos1.pose.position.y = 5.5
                        pos1.pose.position.z = 1
                        edrone1_local_pos_pub.publish(pos1)
              
                    if (d1_y > 0) and (d1_z < 2):
                        ofb_ctl.edrone1_land_mode()
                        ofb_ctl.edrone1_grip_activate(False)
                        stateMt.gripper_check2.data = "False"
                        box12_catched = False
                        box12_delivered = True
                            
                    
            if (box12_delivered == True):

                if (stateMt.state2.mode == "AUTO.LAND") and (d1_y > 0): 
                    ofb_ctl.edrone1_offboard_set_mode()
                    ofb_ctl.edrone1_grip_activate(False)
                    pos1.pose.position.x = d1_x
                    pos1.pose.position.y = d1_y
                    pos1.pose.position.z = 5
                    edrone1_local_pos_pub.publish(pos1)
                
                if (5 - d1_z < 0.5):
                    pos1.pose.position.x = 0
                    pos1.pose.position.y = 0
                    pos1.pose.position.z = 5
                    edrone1_local_pos_pub.publish(pos1)
                
                if (d1_x < 0) and (5 - (d1_z) < 0.5):
                    ofb_ctl.edrone1_land_mode()

        rate.sleep()
    
if __name__ == '__main__':
    try:
        main()
    except rospy.ROSInterruptException:
        pass