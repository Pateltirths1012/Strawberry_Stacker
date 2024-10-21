'''
Team ID :       2279
Author List     Tirth Patel
Filename:       bonus_configuration.py
Theme:          strawberry_stacker    
Functions:      [edrone0_setArm, edrone0_offboard_set_mode, edrone0_land_mode, edrone0_grip_activate, edrone1_setArm, edrone1_offboard_set_mode, 
                edrone1_land_mode, edrone1_grip_activate, main]
Global Variables:None 
'''

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
    '''
    function Name:  __init__
    Input:          None
    Output:         None
    Logic:          initializes a rosnode
    Example Call:   Called automatically by the Operating System
    '''

    def __init__(self):
        rospy.init_node("strawberry_stacker",anonymous = True)
        

    '''
    function Name:  edrone0_setArm
    Input:          None
    Output:         None
    Logic:          sets the mode from land to arming
    Example Call:   ofb_ctl.edrone0_setArm()
    '''

    def edrone0_setArm(self):
        rospy.wait_for_service('edrone0/mavros/cmd/arming')
        try:
            armService = rospy.ServiceProxy('edrone0/mavros/cmd/arming', mavros_msgs.srv.CommandBool) 
            armService(True)
        except rospy.ServiceException as e:
            print ("Service arming call failed: %s"%e)
    
    '''
    function Name:  edrone0_offboard_set_mode
    Input:          None
    Output:         None
    Logic:          sets the edrone0 mode to offboard 
    Example Call:   ofb_ctl.edrone0_offboard_set_mode()
    '''

    def edrone0_offboard_set_mode(self):
        rospy.wait_for_service('edrone0/mavros/set_mode')
        try:
            offset = rospy.ServiceProxy('edrone0/mavros/set_mode',mavros_msgs.srv.SetMode)
            offset(5,"OFFBOARD")
        except rospy.ServiceException as e:
            print("Servive offboard call failed: %s"%e)
    '''
    function Name:  edrone0_land_mode
    Input:          None
    Output:         None
    Logic:          sets the edrone0 mode to land
    Example Call:   ofb_ctl.edrone0_land_mode()
    '''

    def edrone0_land_mode(self):
        rospy.wait_for_service('edrone0/mavros/set_mode')
        try:
            offset = rospy.ServiceProxy('edrone0/mavros/set_mode',mavros_msgs.srv.SetMode)
            offset(0,"AUTO.LAND")
        except rospy.ServiceException as e:
            print("Servive land call failed: %s"%e)

    '''
    function Name:  edrone0_grip_activate
    Input:          boolean
    Output:         None
    Logic:          activates the grip if true and picks up the box and for false it drops the box for edrone0
    Example Call:   ofb_ctl.edrone0_grip_activate()
    '''

    def edrone0_grip_activate(Self,grip):
        rospy.wait_for_service('edrone0/activate_gripper')
        try:
            catch = rospy.ServiceProxy('edrone0/activate_gripper',gazebo_ros_link_attacher.srv.Gripper)
            catch(grip)
        except rospy.ServiceException as e:
            print("Service edrone0 grip activate call failed: %s"%e)

    '''
    function Name:  edrone1_setArm
    Input:          None
    Output:         arms drone1
    Logic:          sets the mode from land to arming
    Example Call:   ofb_ctl.edrone1_setArm()
    '''

    def edrone1_setArm(self):
        rospy.wait_for_service('edrone1/mavros/cmd/arming')
        try:
            armService = rospy.ServiceProxy('edrone1/mavros/cmd/arming', mavros_msgs.srv.CommandBool) 
            armService(True)
        except rospy.ServiceException as e:
            print ("Service arming call failed: %s"%e)
    
    '''
    function Name:  edrone1_offboard_set_mode
    Input:          None
    Output:         None
    Logic:          sets the edrone0 mode to offboard 
    Example Call:   ofb_ctl.edrone1_offboard_set_mode()
    '''
    
    def edrone1_offboard_set_mode(self):
        rospy.wait_for_service('edrone1/mavros/set_mode')
        try:
            offset = rospy.ServiceProxy('edrone1/mavros/set_mode',mavros_msgs.srv.SetMode)
            offset(5,"OFFBOARD")
        except rospy.ServiceException as e:
            print("Servive offboard call failed: %s"%e)
    
    '''
    function Name:  edrone1_land_mode
    Input:          None
    Output:         None
    Logic:          sets the edrone1 mode to land
    Example Call:   ofb_ctl.edrone1_land_mode()
    '''
    
    def edrone1_land_mode(self):
        rospy.wait_for_service('edrone1/mavros/set_mode')
        try:
            offset = rospy.ServiceProxy('edrone1/mavros/set_mode',mavros_msgs.srv.SetMode)
            offset(0,"AUTO.LAND")
        except rospy.ServiceException as e:
            print("Servive land call failed: %s"%e)
    
    '''
    function Name:  edrone1_grip_activate
    Input:          boolean
    Output:         None
    Logic:          activates the grip if true and picks up the box and for false it drops the box for edrone1
    Example Call:   ofb_ctl.edrone1_grip_activate(True/False)
    '''

    def edrone1_grip_activate(Self,grip):
        rospy.wait_for_service('edrone1/activate_gripper')
        try:
            catch = rospy.ServiceProxy('edrone1/activate_gripper',gazebo_ros_link_attacher.srv.Gripper)
            catch(grip)
        except rospy.ServiceException as e:
            print("Service edrone1 grip activate call failed: %s"%e)
    
class stateMonitor:
    def __init__(self):
        #state call back variable for drone0
        self.state1 = State()
        #state callback variable for drone1
        self.state2 = State()
        #bridge variable
        self.bridge = CvBridge()
    
    '''
    function Name:  edrone0_stateCb
    Input:          None
    Output:         None
    Logic:          publishes the current state of edrone0
    Example Call:   stateMt.edrone0_stateCb()
    '''

    def edrone0_stateCb(self, msg):
        self.state1 = msg
    
    '''
    function Name:  edrone0_local_position_cb
    Input:          None
    Output:         None
    Logic:          subscribes to the local position of edrone0
    Example Call:   stateMt.local_position0.pose
    '''

    def edrone0_local_position_cb(self, data):
        self.local_position0 = data
    
    '''
    function Name:  edrone0_gripper_cb
    Input:          None
    Output:         None
    Logic:          checks whether the object is pickable or not
    Example Call:   stateMt.gripper_check1.data
    '''

    def edrone0_gripper_cb(self, data): 
        self.gripper_check1 = data
    
    '''
    function Name:  edrone0_image_callback
    Input:          None
    Output:         None
    Logic:          publishes the binary data images to cv2 image format for further detection use from the camera of edrone0
    Example Call:   stateMt.edrone0_image_callback()
    '''

    def edrone0_image_callback(self,data):
        self.img1 = self.bridge.imgmsg_to_cv2(data, "bgr8")

    '''
    function Name:  edrone0_camera_info
    Input:          None
    Output:         None
    Logic:          publishes the camera info of the edrone0 (distortion, matrix etc)
    Example Call:   stateMt.edrone0_camera_info()
    '''

    def edrone0_camera_info(self,data):
        self.camera0 = data
    
    '''
    function Name:  edrone1_stateCb
    Input:          None
    Output:         None
    Logic:          publishes the current state of edrone1
    Example Call:   stateMt.edrone1_stateCb()
    '''

    def edrone1_stateCb(self, msg):
        self.state2 = msg
    
    '''
    function Name:  edrone1_local_position_cb
    Input:          None
    Output:         None
    Logic:          subscribes to the local position of edrone1
    Example Call:   stateMt.local_position0.pose
    '''
    
    def edrone1_local_position_cb(self, data):
        self.local_position1 = data

    '''
    function Name:  edrone1_gripper_cb
    Input:          None
    Output:         None
    Logic:          checks whether the object is pickable or not
    Example Call:   stateMt.gripper_check2.data
    '''

    def edrone1_gripper_cb(self, data): 
        self.gripper_check2 = data

    '''
    function Name:  edrone1_image_callback
    Input:          None
    Output:         None
    Logic:          publishes the binary data images to cv2 image format for further detection use from the camera of edrone0
    Example Call:   stateMt.edrone1_image_callback()
    '''

    def edrone1_image_callback(self,data):
        self.img2 = self.bridge.imgmsg_to_cv2(data, "bgr8")

    '''
    function Name:  edrone1_camera_info
    Input:          None
    Output:         None
    Logic:          publishes the camera info of the edrone0 (distortion, matrix etc)
    Example Call:   stateMt.edrone1_camera_info()
    '''

    def edrone1_camera_info(self,data):
        self.camera1 = data

    '''
    function Name:  spawn_cb
    Input:          None
    Output:         None
    Logic:          publishes the row number of each spawned box with the clock signal
    Example Call:   stateMt.spawn_cb()
    '''

    def spawn_cb(self,data):
        self.row = data

'''
function Name:  main
Input:          None
Output:         None
Logic:          User has to execute the programm in terminal and the gazebo environment 
                will work accordingly as the algorithm suggests
Example Call:   Called automatically by the Operating System
'''

def main():
    #call variable for class stateMonitor
    stateMt = stateMonitor()
    #call variable for class strawberrystacker (ofb_ctl : offboard control)
    ofb_ctl = strawberry_stacker()

    #variables that publishes the setpoints for the edrones respectively
    edrone0_local_pos_pub = rospy.Publisher('edrone0/mavros/setpoint_position/local',PoseStamped, queue_size=10)
    edrone1_local_pos_pub = rospy.Publisher('edrone1/mavros/setpoint_position/local',PoseStamped, queue_size=10)

    #variables that publishes the setpoint velocity for the edrones respectively
    edrone0_local_vel_pub = rospy.Publisher('edrone0/mavros/setpoint_velocity/cmd_vel',Twist,queue_size=10)
    edrone1_local_vel_pub = rospy.Publisher('edrone1/mavros/setpoint_velocity/cmd_vel',Twist,queue_size=10)

    #rate at which the node will publish the custom messages to the mavlink
    rate = rospy.Rate(20.0)

    #mavros message format (PoseStamped) that will be published for edrone0
    pos0 = PoseStamped()
    pos0.pose.position.x = -2
    pos0.pose.position.y = 0
    pos0.pose.position.z = 3

    #mavros message format (PoseStamped) that will be published for edrone1
    pos1 = PoseStamped()
    pos1.pose.position.x = -2
    pos1.pose.position.y = 0
    pos1.pose.position.z = 3

    #mavros message format (Twist) that will be published for edrone0 and edrone1
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

    #list for storing the row numbers (but not unique ones)
    box_pos = [] 
    
    #publishing the initial setpoints for offboard mode
    for i in range(100):
        edrone0_local_pos_pub.publish(pos0)
        edrone1_local_pos_pub.publish(pos1)
        rate.sleep()
    
    while (stateMt.state1.mode != "OFFBOARD") and (stateMt.state2.mode != "OFFBOARD"):
        ofb_ctl.edrone0_offboard_set_mode()
        ofb_ctl.edrone1_offboard_set_mode()
        rate.sleep()
    print("edrones OFFBOARD mode activated")

    #arming the drones
    while (stateMt.state1.armed != True) and (stateMt.state2.armed != True):
        ofb_ctl.edrone0_setArm()
        ofb_ctl.edrone1_setArm()
        rate.sleep()
    print("edrones Armed!")

    #list for storing the unique row numbers published by spawn_info topic
    rows = [] 

    #flag variables for boxes to be delivered by edrone0
    box1_catched = False
    box2_catched = False
    box3_catched = False
    box4_catched = False
    
    box1_delivered = False
    box2_delivered = False
    box3_delivered = False
    box4_delivered = False
    
    #flag variables for boxes to be delivered by edrone1
    box5_catched = False
    box6_catched = False
    box7_catched = False
    box8_catched = False
    
    box5_delivered = False
    box6_delivered = False
    box7_delivered = False
    box8_delivered = False

    #flag variables for box colors used by edrone0
    blue0 = False
    red0 = False
    #flag variables for box colors used by edrone1
    blue1 = False
    red1 = False
    
    while not rospy.is_shutdown():
        #current position coordonates of edrone0
        d0_x = stateMt.local_position0.pose.position.x # d0_x : x position of drone0
        d0_y = stateMt.local_position0.pose.position.y # d0_y : y position of drone0
        d0_z = stateMt.local_position0.pose.position.z # d0_z : z position of drone0

        #current position coordinates of edrone1
        d1_x = stateMt.local_position1.pose.position.x # d1_x : x position of drone1
        d1_y = stateMt.local_position1.pose.position.y # d1_y : y position of drone1
        d1_z = stateMt.local_position1.pose.position.z # d1_z : z position of drone1

        # publishing the initial positions for both drones respectively at the same time
        if (box4_delivered == False) and (int(d0_x) == 0) and (int(d0_y) == 0):
            edrone0_local_pos_pub.publish(pos0)

        if (box8_delivered == False) and (int(d1_x) == 0) and (int(d1_y) == 0):
            edrone1_local_pos_pub.publish(pos1)

        #continuously storing the row-numbers by calling the function stateMt.row in list box_pos
        if (stateMt.row.data != None):
            box_pos.append(stateMt.row.data)

        #only storing the unique row-numbers to the list named rows
        if len(box_pos)!=0:
            for i in box_pos:
                if i not in rows:
                    rows.append(i)
        
        #code for image processing
        aruco_dict = aruco.Dictionary_get(aruco.DICT_5X5_250)
        parameters = aruco.DetectorParameters_create()
        #camera matrix
        mtx = np.array(stateMt.camera0.K)
        mtx = mtx.reshape(3,3)
        #camera distortion
        dst = np.array(stateMt.camera0.D)
        #corners and id of aruco markers for both drone cameras
        corners1,ids1,p1 = aruco.detectMarkers(stateMt.img1, aruco_dict, parameters = parameters, cameraMatrix = mtx, distCoeff = dst)
        corners2,ids2,p2 = aruco.detectMarkers(stateMt.img2, aruco_dict, parameters = parameters, cameraMatrix = mtx, distCoeff = dst)

        #if the unique row list 'rows' is not empty then..
        if len(rows) >= 2:
            #algorithm for picking up box 1
            if (box1_delivered == False):
                #converting the row number into y co-ordinate
                y1 = (rows[1]-1) * 4
                #if the drone height is 3m then move towards the box in y-axis
                if (3 - (d0_z) < 0.5):
                    pos0.pose.position.x = -2
                    pos0.pose.position.y = y1
                    pos0.pose.position.z = 3
                    edrone0_local_pos_pub.publish(pos0)
                #if the y axis of box and drone are same then move forward in x-axis towards the box
                if (y1 - (d0_y)) < 0.5:
                    pos0.pose.position.x = 3
                    pos0.pose.position.y = y1
                    pos0.pose.position.z = 3
                    edrone0_local_pos_pub.publish(pos0)
                
                #algorithm for detecting the aruco marker
                if len(corners1)!=0:
                    #calculating the rotation vectors and translation vectors of the marker from the drone camera
                    rvecs,tvecs,_ = aruco.estimatePoseSingleMarkers(corners1,0.05,mtx,dst)
                    yaw = -1 * math.atan2(tvecs[0][0][0], tvecs[0][0][2])
                    #calculating the yaw angle for and stabilizing the drone over the marker
                    angle = np.rad2deg(yaw)
                    pos0.pose.position.x = d0_x - 0.4
                    pos0.pose.position.y = y1
                    pos0.pose.position.z = 3
                    edrone0_local_pos_pub.publish(pos0)

                    #if the angle between the drone and marker is 0 then it will land on it
                    if (0 < abs(angle) < 0.4):
                        ofb_ctl.edrone0_land_mode()

                        #if the id is 2 then its a blue box or else its a red one
                        if (ids1[0][0] == 2):
                            blue0 = True
                        elif (ids1[0][0] == 1):
                            red0 = True
                
                #if the box is within the perphery then it will be gripped
                if (stateMt.gripper_check1.data == "True") and (0 < d0_y < 60):
                    ofb_ctl.edrone0_grip_activate(True)
                    box1_catched = True                                                                                                                                                      
                
                #if the box is a blue box then this block of code will deliver it to the blue truck
                if (box1_catched == True) and (blue0 == True):

                    #if the drone is landing then it will be set in offboard mode and take off till 3m                                                                                                                                         
                    if (stateMt.state1.mode == "AUTO.LAND") and (d0_y > -1):
                        ofb_ctl.edrone0_offboard_set_mode()
                        pos0.pose.position.x = d0_x            
                        pos0.pose.position.y = y1
                        pos0.pose.position.z = 3
                        edrone0_local_pos_pub.publish(pos0)

                    #if box is catched and height is 3m then it will move towards the truck
                    if (3 - d0_z) < 0.5:
                        pos0.pose.position.x = 15.5
                        pos0.pose.position.y = y1
                        pos0.pose.position.z = 3
                        edrone0_local_pos_pub.publish(pos0)

                    if (15.5 - d0_x) < 0.5:
                        pos0.pose.position.x = 15.5
                        pos0.pose.position.y = -6.5
                        pos0.pose.position.z = 3
                        edrone0_local_pos_pub.publish(pos0)
                    
                    #if the drone is over the truck then it will land at a certain panel
                    if (abs(-6.5 - d0_y) < 0.2):
                        pos0.pose.position.x = 15.5
                        pos0.pose.position.y = -6.5
                        pos0.pose.position.z = 1
                        edrone0_local_pos_pub.publish(pos0)

                    #if the altitude is less then 2.5 then it will drop the box
                    if (d0_y < -1) and (d0_z < 2.5):
                        ofb_ctl.edrone0_land_mode()
                        ofb_ctl.edrone0_grip_activate(False)
                        box1_catched = False
                        box1_delivered = True
                        blue0 = False

                #else if the box is a red box then this block of code will deliver it to the red box
                elif (box1_catched == True) and (red0 == True):

                    if (stateMt.state1.mode == "AUTO.LAND") and (d0_y < 60):
                        ofb_ctl.edrone0_offboard_set_mode()
                        pos0.pose.position.x = d0_x            
                        pos0.pose.position.y = y1
                        pos0.pose.position.z = 3
                        edrone0_local_pos_pub.publish(pos0)
                    
                    if (3 - d0_z) < 0.5:
                        pos0.pose.position.x = 59
                        pos0.pose.position.y = y1
                        pos0.pose.position.z = 3
                        edrone0_local_pos_pub.publish(pos0)
                    
                    if (59 - (d0_x) < 0.5):
                        pos0.pose.position.x = 59
                        pos0.pose.position.y = 64.5
                        pos0.pose.position.z = 3
                        edrone0_local_pos_pub.publish(pos0)

                    if (64.5 - d0_y) < 0.3:
                        pos0.pose.position.x = 59
                        pos0.pose.position.y = 64.5
                        pos0.pose.position.z = 1
                        edrone0_local_pos_pub.publish(pos0)
                   
                    if (d0_y > 60) and (d0_z < 2.5):
                        ofb_ctl.edrone0_land_mode()
                        ofb_ctl.edrone0_grip_activate(False)
                        box1_catched = False
                        box1_delivered = True 
                        red0 = False
                        #algorithm for delivering box_1 ends here
                    
            #the following algorithm is will only be executed if the previous one has ended and so on for the others as well till box-4 has not been delivered

            #algorithm for picking and delivering of box 2 by edrone0 just like the previous one to their respective trucks
            if (box1_delivered == True) and (box2_delivered == False):
                #y coordonate of the 3rd box that has been spawned
                y1 = 24
                #if the box 1 has been delivered then the drone will take off and move towards the second box
                if (stateMt.state1.mode == "AUTO.LAND") and ((d0_y < 0) or (d0_y > 60)):
                    ofb_ctl.edrone0_grip_activate(False)
                    ofb_ctl.edrone0_offboard_set_mode()
                    pos0.pose.position.x = d0_x
                    pos0.pose.position.y = d0_y
                    pos0.pose.position.z = 3
                    edrone0_local_pos_pub.publish(pos0)
        
                if (3 - (d0_z) < 0.5):
                    pos0.pose.position.x = d0_x
                    pos0.pose.position.y = y1
                    pos0.pose.position.z = 3
                    edrone0_local_pos_pub.publish(pos0)
                
                if abs(y1 - (d0_y) < 0.5):
                    pos0.pose.position.x = 2
                    pos0.pose.position.y = y1
                    pos0.pose.position.z = 3
                    edrone0_local_pos_pub.publish(pos0)

                if len(corners1)!=0:
                    rvecs,tvecs,_ = aruco.estimatePoseSingleMarkers(corners1,0.05,mtx,dst)
                    yaw = -1 * math.atan2(tvecs[0][0][0], tvecs[0][0][2])
                    angle = np.rad2deg(yaw)
                    pos0.pose.position.x = d0_x + 0.5
                    pos0.pose.position.y = y1
                    pos0.pose.position.z = 3
                    edrone0_local_pos_pub.publish(pos0)

                    if (0 < abs(angle) < 0.4):
                        ofb_ctl.edrone0_land_mode()
                        if (ids1[0][0] == 2):
                            blue0 = True
                        elif (ids1[0][0] == 1):
                            red0 = True
                
                if (stateMt.gripper_check1.data == "True") and (0 < d0_y < 60):
                    ofb_ctl.edrone0_grip_activate(True)
                    box2_catched = True 
                
                #if the box is a blue box then this block of code will be executed
                if (box2_catched == True) and (blue0 == True):

                    #if the box catched is blue and it is in the field then it will take off till 3 m
                    if (stateMt.state1.mode == "AUTO.LAND") and (d0_y > -1):
                        ofb_ctl.edrone0_offboard_set_mode()
                        pos0.pose.position.x = d0_x            
                        pos0.pose.position.y = d0_y
                        pos0.pose.position.z = 3
                        edrone0_local_pos_pub.publish(pos0)

                    #if 3m height is achieved then it will move towards the blue truck
                    if (3 - (d0_z) < 0.5):
                        pos0.pose.position.x = 15.5
                        pos0.pose.position.y = y1
                        pos0.pose.position.z = 3
                        edrone0_local_pos_pub.publish(pos0)
                    
                    if abs(15.5 - (d0_x)) < 0.5:
                        pos0.pose.position.x = 15.5
                        pos0.pose.position.y = -7.5
                        pos0.pose.position.z = 3
                        edrone0_local_pos_pub.publish(pos0)
                    
                    #if the absolute value of the drone is -7.5y then it will land on that pallete and drop the box
                    if (abs(-7.5 - d0_y) < 0.2):
                        pos0.pose.position.x = 15.5
                        pos0.pose.position.y = -7.5
                        pos0.pose.position.z = 1
                        edrone0_local_pos_pub.publish(pos0)
                       
                    if (d0_y < -1) and (d0_z < 2.5):
                        ofb_ctl.edrone0_land_mode()                    
                        ofb_ctl.edrone0_grip_activate(False)
                        box2_catched = False
                        box2_delivered = True
                        blue0 = False
                
                #else if the box is a red one then this block of code will be executed
                elif (box2_catched == True) and (red0 == True):
                    if (stateMt.state1.mode == "AUTO.LAND") and (d0_y < 60):
                        ofb_ctl.edrone0_offboard_set_mode()
                        pos0.pose.position.x = d0_x            
                        pos0.pose.position.y = y1
                        pos0.pose.position.z = 3
                        edrone0_local_pos_pub.publish(pos0)
                    
                    if (3 - d0_z) < 0.5:
                        pos0.pose.position.x = 59
                        pos0.pose.position.y = y1
                        pos0.pose.position.z = 3
                        edrone0_local_pos_pub.publish(pos0)
                    
                    if (59 - (d0_x) < 0.5):
                        pos0.pose.position.x = 59
                        pos0.pose.position.y = 64.5
                        pos0.pose.position.z = 3
                        edrone0_local_pos_pub.publish(pos0)

                    if (64.5 - d0_y) < 0.3:
                        pos0.pose.position.x = 59
                        pos0.pose.position.y = 64.5
                        pos0.pose.position.z = 1
                        edrone0_local_pos_pub.publish(pos0)
                   
                    if (d0_y > 60) and (d0_z < 2.5):
                        ofb_ctl.edrone0_land_mode()                        
                        ofb_ctl.edrone0_grip_activate(False)
                        box2_catched = False
                        box2_delivered = True
                        red0 = False
                        #algorithm for delivering box_2 ends here
            
            #algorithm for picking and delivering box 3 by edrone0 just like the previous one to their respective trucks
            if (box2_delivered == True) and (box3_delivered == False):
                y1 = (rows[5]-1)*4

                if (stateMt.state1.mode == "AUTO.LAND") and ((d0_y < 0) or (d0_y > 60)):
                    ofb_ctl.edrone0_grip_activate(False)
                    ofb_ctl.edrone0_offboard_set_mode()
                    pos0.pose.position.x = d0_x
                    pos0.pose.position.y = d0_y
                    pos0.pose.position.z = 3
                    edrone0_local_pos_pub.publish(pos0)
                
                if (3-(d0_z)<0.5):
                    pos0.pose.position.x = d0_x
                    pos0.pose.position.y = y1
                    pos0.pose.position.z = 3
                    edrone0_local_pos_pub.publish(pos0)
                
                if abs(y1-(d0_y)<0.5):
                    pos0.pose.position.x = 1.5
                    pos0.pose.position.y = y1
                    pos0.pose.position.z = 3
                    edrone0_local_pos_pub.publish(pos0)
                
                if len(corners1)!=0:
                    rvecs,tvecs,_ = aruco.estimatePoseSingleMarkers(corners1,0.05,mtx,dst)
                    yaw = -1 * math.atan2(tvecs[0][0][0], tvecs[0][0][2])
                    angle = np.rad2deg(yaw)
                    pos0.pose.position.x = d0_x + 0.5
                    pos0.pose.position.y = y1
                    pos0.pose.position.z = 3
                    edrone0_local_pos_pub.publish(pos0)

                    if (0 < abs(angle) < 0.4):
                        ofb_ctl.edrone0_land_mode()
                        if (ids1[0][0] == 2):
                            blue0 = True
                        elif (ids1[0][0] == 1):
                            red0 = True
                
                if (stateMt.gripper_check1.data == "True") and (0 < d0_y < 60):
                    ofb_ctl.edrone0_grip_activate(True)
                    box3_catched = True 
                
                #if the box is a blue box then it will be delivered to the blue truck 
                if (box3_catched == True) and (blue0 == True):
                    if (stateMt.state1.mode == "AUTO.LAND") and (d0_y > -2):
                        ofb_ctl.edrone0_offboard_set_mode()
                        pos0.pose.position.x = d0_x
                        pos0.pose.position.y = d0_y
                        pos0.pose.position.z = 3
                        edrone0_local_pos_pub.publish(pos0)

                    if (3 - (d0_z) < 0.5):
                        pos0.pose.position.x = 16
                        pos0.pose.position.y = y1
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
                        
                    if (d0_z < 2.5) and (d0_y < -2):
                        ofb_ctl.edrone0_land_mode()                        
                        ofb_ctl.edrone0_grip_activate(False)
                        box3_catched = False
                        box3_delivered = True
                        blue0 = False

                #if the box is a red box then it will be delivered to the red truck 
                elif (box3_catched == True) and (red0 == True):
                    if (stateMt.state1.mode == "AUTO.LAND") and (d0_y < 60):
                        ofb_ctl.edrone0_offboard_set_mode()
                        pos0.pose.position.x = d0_x            
                        pos0.pose.position.y = y1
                        pos0.pose.position.z = 3
                        edrone0_local_pos_pub.publish(pos0)
                    
                    if (3 - d0_z) < 0.5:
                        pos0.pose.position.x = 58
                        pos0.pose.position.y = y1
                        pos0.pose.position.z = 3
                        edrone0_local_pos_pub.publish(pos0)
                    
                    if (58 - (d0_x) < 0.5):
                        pos0.pose.position.x = 58
                        pos0.pose.position.y = 65.5
                        pos0.pose.position.z = 3
                        edrone0_local_pos_pub.publish(pos0)

                    if (65.5 - d0_y) < 0.3:
                        pos0.pose.position.x = 58
                        pos0.pose.position.y = 65.5
                        pos0.pose.position.z = 1
                        edrone0_local_pos_pub.publish(pos0)
                   
                    if (d0_y > 60) and (d0_z < 2.5):
                        ofb_ctl.edrone0_land_mode()                        
                        ofb_ctl.edrone0_grip_activate(False)
                        box3_catched = False
                        box3_delivered = True
                        red0 = False
                            
            #algorithm for picking and delivering of box 4 by edrone0 just like the previous one to their respective trucks
            if (box3_delivered == True) and (box4_delivered == False):
                y1 = (rows[7]-1)*4
                
                if (stateMt.state1.mode == "AUTO.LAND") and ((d0_y < 0) or (d0_y > 60)):
                    ofb_ctl.edrone0_grip_activate(False)
                    ofb_ctl.edrone0_offboard_set_mode()
                    pos0.pose.position.x = d0_x
                    pos0.pose.position.y = d0_y
                    pos0.pose.position.z = 3
                    edrone0_local_pos_pub.publish(pos0)
                
                if (3-(d0_z)<0.5):
                    pos0.pose.position.x = d0_x
                    pos0.pose.position.y = y1
                    pos0.pose.position.z = 3
                    edrone0_local_pos_pub.publish(pos0)
                
                if abs(y1 - (d0_y) < 0.5):
                    pos0.pose.position.x = 1
                    pos0.pose.position.y = y1
                    pos0.pose.position.z = 3
                    edrone0_local_pos_pub.publish(pos0)
                
                if len(corners1)!=0:
                    rvecs,tvecs,_ = aruco.estimatePoseSingleMarkers(corners1,0.05,mtx,dst)
                    yaw = -1 * math.atan2(tvecs[0][0][0], tvecs[0][0][2])
                    angle = np.rad2deg(yaw)
                    pos0.pose.position.x = d0_x + 0.4
                    pos0.pose.position.y = y1
                    pos0.pose.position.z = 3
                    edrone0_local_pos_pub.publish(pos0)
            
                    if (0 < abs(angle) < 0.4):
                        ofb_ctl.edrone0_land_mode()
                        if (ids1[0][0] == 2):
                            blue0 = True
                        elif (ids1[0][0] == 1):
                            red0 = True
                
                if (stateMt.gripper_check1.data == "True") and (0 < d0_y < 60):
                    ofb_ctl.edrone0_grip_activate(True)
                    box4_catched = True
                
                #if the box is a blue box then it will be delivered to the blue truck 
                if (box4_catched == True) and (blue0 == True):
                    if (stateMt.state1.mode == "AUTO.LAND") and (d0_y > -1):
                        ofb_ctl.edrone0_offboard_set_mode()
                        pos0.pose.position.x = d0_x
                        pos0.pose.position.y = d0_y
                        pos0.pose.position.z = 3
                        edrone0_local_pos_pub.publish(pos0)
                
                    if (3-(d0_z) < 0.5):
                        pos0.pose.position.x = 15
                        pos0.pose.position.y = y1
                        pos0.pose.position.z = 3
                        edrone0_local_pos_pub.publish(pos0)
                        
                    if abs(16 - d0_x) < 0.5:
                        pos0.pose.position.x = 15
                        pos0.pose.position.y = -8
                        pos0.pose.position.z = 3
                        edrone0_local_pos_pub.publish(pos0)
                    
                    if (abs(-8 - d0_y) < 0.2):
                        pos0.pose.position.x = 15
                        pos0.pose.position.y = -8
                        pos0.pose.position.z = 1
                        edrone0_local_pos_pub.publish(pos0)
                
                    if (d0_y < -1) and (d0_z < 2.5):
                        ofb_ctl.edrone0_land_mode()                        
                        ofb_ctl.edrone0_grip_activate(False)
                        box4_catched = False
                        box4_delivered = True
                        blue0 = False
                    
                #if the box is a red box then it will be delivered to the red truck     
                elif (box4_catched == True) and (red0 == True):
                    if (stateMt.state1.mode == "AUTO.LAND") and (d0_y < 60):
                        ofb_ctl.edrone0_offboard_set_mode()
                        ofb_ctl.edrone0_grip_activate(False)

                        pos0.pose.position.x = d0_x            
                        pos0.pose.position.y = y1
                        pos0.pose.position.z = 3
                        edrone0_local_pos_pub.publish(pos0)
                    
                    if (3 - d0_z) < 0.5:
                        pos0.pose.position.x = 58
                        pos0.pose.position.y = y1
                        pos0.pose.position.z = 3
                        edrone0_local_pos_pub.publish(pos0)
                    
                    if (58 - (d0_x) < 0.5):
                        pos0.pose.position.x = 58
                        pos0.pose.position.y = 64.5
                        pos0.pose.position.z = 3
                        edrone0_local_pos_pub.publish(pos0)

                    if (64.5 - d0_y) < 0.3:
                        pos0.pose.position.x = 58
                        pos0.pose.position.y = 64.5
                        pos0.pose.position.z = 1
                        edrone0_local_pos_pub.publish(pos0)
                   
                    if (d0_y > 60) and (d0_z < 2.5):
                        ofb_ctl.edrone0_land_mode()                        
                        ofb_ctl.edrone0_grip_activate(False)
                        box4_catched = False
                        box4_delivered = True
                        red0 = False

            #if 4 boxes are delivered by the drone0 then it will land at its initial position
            if (box4_delivered == True):
                if (stateMt.state1.mode == "AUTO.LAND") and ((d0_y < 0) or (d0_y > 60)):
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
                
        #'''algorithm for drone 2 starts here'''
        # the previous code has nothing to do with the following one, this will run simultaneously so that both drones work in coordination
        # box 5 is the first box of drone1 so this will be executed with the box 1 of drone0
            if (box5_delivered == False):
                #calculating the local coordinate of y axis of box with respect to drone1's local_pos
                y2 = ((rows[0]-1)*4) - 60
                if (3 - (d1_z) < 0.5):
                    pos1.pose.position.x = -2
                    pos1.pose.position.y = y2
                    pos1.pose.position.z = 3
                    edrone1_local_pos_pub.publish(pos1)
                
                if abs(y2 - (d1_y)) < 0.5:
                    pos1.pose.position.x = 3
                    pos1.pose.position.y = y2
                    pos1.pose.position.z = 3
                    edrone1_local_pos_pub.publish(pos1)

                #algorithm for detecting the marker
                if (len(corners2) != 0):
                    rvecs,tvecs,_ = aruco.estimatePoseSingleMarkers(corners2,0.05,mtx,dst)
                    yaw = -1 * math.atan2(tvecs[0][0][0], tvecs[0][0][2])
                    angle = np.rad2deg(yaw)
                    pos1.pose.position.x =  d1_x - 0.4
                    pos1.pose.position.y = y2
                    pos1.pose.position.z = 3
                    edrone1_local_pos_pub.publish(pos1)

                    if (0 < abs(angle) < 0.4):
                        ofb_ctl.edrone1_land_mode()

                        if (ids2[0][0] == 2):
                            blue1 = True
                        elif (ids2[0][0] == 1):
                            red1 = True

                if (stateMt.gripper_check2.data == "True") and (-60 < d1_y < 0):
                    ofb_ctl.edrone1_grip_activate(True)
                    box5_catched = True
                
                #if the box is a red one then this block of code will deliver it to the red truck respectively
                if (box5_catched == True) and (red1 == True):
                    
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

                    if abs(4.5 - d1_y) < 0.3:
                        pos1.pose.position.x = 58
                        pos1.pose.position.y = 4.5
                        pos1.pose.position.z = 1
                        edrone1_local_pos_pub.publish(pos1)
                        
                    if (d1_y > 0) and (d1_z < 2.5):
                        ofb_ctl.edrone1_land_mode()                        
                        ofb_ctl.edrone1_grip_activate(False)
                        box5_catched = False
                        box5_delivered = True
                        red1 = False

                #if the box is a blue one then this block of code will deliver it to the blue truck respectively
                elif (box5_catched == True) and (blue1 == True):
                    
                    if (stateMt.state2.mode == "AUTO.LAND") and (d1_y > -62):
                        ofb_ctl.edrone1_offboard_set_mode()
                        pos1.pose.position.x = d1_x
                        pos1.pose.position.y = y2
                        pos1.pose.position.z = 3
                        edrone1_local_pos_pub.publish(pos1)
                
                    if (3-(d1_z) < 0.5):
                        pos1.pose.position.x = 17
                        pos1.pose.position.y = y2
                        pos1.pose.position.z = 3
                        edrone1_local_pos_pub.publish(pos1)
                    
                    if (17 - (d1_x) < 0.5):
                        pos1.pose.position.x = 17
                        pos1.pose.position.y = -66.5
                        pos1.pose.position.z = 3
                        edrone1_local_pos_pub.publish(pos1)

                    if abs(-66.5 - d1_y) < 0.2:
                        pos1.pose.position.x = 17
                        pos1.pose.position.y = -66.5
                        pos1.pose.position.z = 1
                        edrone1_local_pos_pub.publish(pos1)
                        
                    if (d1_y < -62) and (d1_z < 2.5):
                        ofb_ctl.edrone1_land_mode()                        
                        ofb_ctl.edrone1_grip_activate(False)
                        box5_catched = False
                        box5_delivered = True
                        blue1 = False
                        #algorithm for delivering the box 5 ends here
            
            #algorithm for picking and delivering box 6 
            if (box5_delivered == True) and (box6_delivered == False):
                y2 = ((7-1)*4) - 60
                if (stateMt.state2.mode == "AUTO.LAND") and ((d1_y > 0) or (d1_y < -60)): 
                    ofb_ctl.edrone1_grip_activate(False)
                    ofb_ctl.edrone1_offboard_set_mode()
                    pos1.pose.position.x = d1_x
                    pos1.pose.position.y = d1_y
                    pos1.pose.position.z = 3
                    edrone1_local_pos_pub.publish(pos1)
                
                if (3 - (d1_z) < 0.5):
                    pos1.pose.position.x = d1_x
                    pos1.pose.position.y = y2
                    pos1.pose.position.z = 3
                    edrone1_local_pos_pub.publish(pos1)
                
                if abs(y2 - (d1_y)) < 0.5:
                    pos1.pose.position.x = 12
                    pos1.pose.position.y = y2
                    pos1.pose.position.z = 3
                    edrone1_local_pos_pub.publish(pos1)
                
                if (len(corners2) != 0):
                    rvecs,tvecs,_ = aruco.estimatePoseSingleMarkers(corners2,0.05,mtx,dst)
                    yaw = -1 * math.atan2(tvecs[0][0][0], tvecs[0][0][2])
                    angle = np.rad2deg(yaw)
                    pos1.pose.position.x = d1_x + 0.5
                    pos1.pose.position.y = y2
                    pos1.pose.position.z = 3
                    edrone1_local_pos_pub.publish(pos1)

                    if (0 < abs(angle) < 0.4):
                        ofb_ctl.edrone1_land_mode()

                        if (ids2[0][0] == 2):
                            blue1 = True
                        elif (ids2[0][0] == 1):
                            red1 = True
                
                if (stateMt.gripper_check2.data == "True") and (-60 < d1_y < 0):
                    ofb_ctl.edrone1_grip_activate(True)
                    box6_catched = True

                #if the box is a red one then this block of code will deliver it to the red truck respectively
                if (box6_catched == True) and (red1 == True):
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

                    if (58 - (d1_x) < 0.5):
                        pos1.pose.position.x = 58
                        pos1.pose.position.y = 5.5
                        pos1.pose.position.z = 3
                        edrone1_local_pos_pub.publish(pos1)
                    
                    if abs(5.5 - d1_y) < 0.2:
                        pos1.pose.position.x = 58
                        pos1.pose.position.y = 5.5
                        pos1.pose.position.z = 1
                        edrone1_local_pos_pub.publish(pos1)

                    if (d1_y > 0) and (d1_z < 2.5):
                        ofb_ctl.edrone1_land_mode()                       
                        ofb_ctl.edrone1_grip_activate(False)
                        box6_catched = False
                        box6_delivered = True
                        red1 = False

                #if the box is a blue one then this block of code will deliver it to the blue truck respectively
                elif (box6_catched == True) and (blue1 == True):
                    
                    if (stateMt.state2.mode == "AUTO.LAND") and (d1_y > -62):
                        ofb_ctl.edrone1_offboard_set_mode()
                        pos1.pose.position.x = d1_x
                        pos1.pose.position.y = y2
                        pos1.pose.position.z = 3
                        edrone1_local_pos_pub.publish(pos1)
                
                    if (3-(d1_z) < 0.5):
                        pos1.pose.position.x = 17
                        pos1.pose.position.y = y2
                        pos1.pose.position.z = 3
                        edrone1_local_pos_pub.publish(pos1)
                    
                    if (17 - (d1_x) < 0.5):
                        pos1.pose.position.x = 17
                        pos1.pose.position.y = -67.5
                        pos1.pose.position.z = 3
                        edrone1_local_pos_pub.publish(pos1)

                    if abs(-67.5 - d1_y) < 0.2:
                        pos1.pose.position.x = 17
                        pos1.pose.position.y = -67.5
                        pos1.pose.position.z = 1
                        edrone1_local_pos_pub.publish(pos1)
                        
                    if (d1_y < -62) and (d1_z < 2.5):
                        ofb_ctl.edrone1_land_mode()                        
                        ofb_ctl.edrone1_grip_activate(False)
                        box6_catched = False
                        box6_delivered = True
                        blue1 = False
                        #algorithm for delivering box 6 ends here
                            
            #algorithm for picking and delivering box 7
            if (box6_delivered == True) and (box7_delivered == False):
                y2 = ((rows[4]-1)*4)-60

                if (stateMt.state2.mode == "AUTO.LAND") and ((d1_y > 0) or (d1_y < -60)): 
                    ofb_ctl.edrone1_grip_activate(False)
                    ofb_ctl.edrone1_offboard_set_mode()
                    pos1.pose.position.x = d1_x
                    pos1.pose.position.y = d1_y
                    pos1.pose.position.z = 4
                    edrone1_local_pos_pub.publish(pos1)

                if (4 - (d1_z) < 0.5):
                    pos1.pose.position.x = d1_x
                    pos1.pose.position.y = y2
                    pos1.pose.position.z = 3
                    edrone1_local_pos_pub.publish(pos1)

                if abs(y2 - (d1_y)) < 0.5:
                    pos1.pose.position.x = 1.5
                    pos1.pose.position.y = y2
                    pos1.pose.position.z = 3
                    edrone1_local_pos_pub.publish(pos1)
                
                if (len(corners2) != 0):
                    rvecs,tvecs,_ = aruco.estimatePoseSingleMarkers(corners2,0.05,mtx,dst)
                    yaw = -1 * math.atan2(tvecs[0][0][0], tvecs[0][0][2])
                    angle = np.rad2deg(yaw)
                    pos1.pose.position.x = d1_x + 0.5
                    pos1.pose.position.y = y2
                    pos1.pose.position.z = 3
                    edrone1_local_pos_pub.publish(pos1)
                    
                    if (0 < abs(angle) < 0.4):
                        ofb_ctl.edrone1_land_mode()

                        if (ids2[0][0] == 2):
                            blue1 = True
                        elif (ids2[0][0] == 1):
                            red1 = True

                if (stateMt.gripper_check2.data == "True") and (-60 < d1_y < 0):
                    ofb_ctl.edrone1_grip_activate(True)
                    box7_catched = True

                #if the box is a red one then this block of code will deliver it to the red truck respectively
                if (box7_catched == True) and (red1 == True):
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
                   
                    if (d1_y > 0) and (d1_z < 2.5):
                        ofb_ctl.edrone1_land_mode()                        
                        ofb_ctl.edrone1_grip_activate(False)
                        box7_catched = False
                        box7_delivered = True
                        red1 = False
            
                #if the box is a blue one then this block of code will deliver it to the blue truck respectively
                elif (box7_catched == True) and (blue1 == True):
                    if (stateMt.state2.mode == "AUTO.LAND") and (d1_y > -62):
                        ofb_ctl.edrone1_offboard_set_mode()
                        pos1.pose.position.x = d1_x
                        pos1.pose.position.y = y2
                        pos1.pose.position.z = 3
                        edrone1_local_pos_pub.publish(pos1)
                
                    if (3-(d1_z) < 0.5):
                        pos1.pose.position.x = 16
                        pos1.pose.position.y = y2
                        pos1.pose.position.z = 3
                        edrone1_local_pos_pub.publish(pos1)
                    
                    if (17 - (d1_x) < 0.5):
                        pos1.pose.position.x = 17
                        pos1.pose.position.y = -68
                        pos1.pose.position.z = 3
                        edrone1_local_pos_pub.publish(pos1)

                    if abs(-68 - d1_y) < 0.2:
                        pos1.pose.position.x = 17
                        pos1.pose.position.y = -68
                        pos1.pose.position.z = 1
                        edrone1_local_pos_pub.publish(pos1)
                        
                    if  (d1_y < -62) and (d1_z < 2.5):
                        ofb_ctl.edrone1_land_mode()
                        ofb_ctl.edrone1_grip_activate(False)
                        box7_catched = False
                        box7_delivered = True
                        blue1 = False
                    #algorithm for delivering box 7 ends here
                    
            #the drone will take from the truck and will move towards the next box
            if (box7_delivered == True) and (box8_delivered == False):
                y2 = ((rows[6]-1)*4) - 60
                if (stateMt.state2.mode == "AUTO.LAND") and ((d1_y > 0) or (d1_y < -60)): 
                    ofb_ctl.edrone1_grip_activate(False)
                    ofb_ctl.edrone1_offboard_set_mode()
                    pos1.pose.position.x = d1_x
                    pos1.pose.position.y = d1_y
                    pos1.pose.position.z = 5
                    edrone1_local_pos_pub.publish(pos1)
                
                if (5 - (d1_z) < 0.5):
                    pos1.pose.position.x = d1_x
                    pos1.pose.position.y = y2
                    pos1.pose.position.z = 3
                    edrone1_local_pos_pub.publish(pos1)
                
                if abs(y2 - (d1_y)) < 0.5:
                    pos1.pose.position.x = 2
                    pos1.pose.position.y = y2
                    pos1.pose.position.z = 3
                    edrone1_local_pos_pub.publish(pos1)
                
                if (len(corners2) != 0):
                    rvecs,tvecs,_ = aruco.estimatePoseSingleMarkers(corners2,0.05,mtx,dst)
                    yaw = -1 * math.atan2(tvecs[0][0][0], tvecs[0][0][2])
                    angle = np.rad2deg(yaw)
                    pos1.pose.position.x = d1_x + 0.4
                    pos1.pose.position.y = y2
                    pos1.pose.position.z = 3
                    edrone1_local_pos_pub.publish(pos1)
        
                    if (0 < abs(angle) < 0.4):
                        ofb_ctl.edrone1_land_mode()

                        if (ids2[0][0] == 2):
                            blue1 = True
                        elif (ids2[0][0] == 1):
                            red1 = True
           
                if (stateMt.gripper_check2.data == "True") and (-60 < d1_y < 0):
                    ofb_ctl.edrone1_grip_activate(True)
                    box8_catched = True
                
                #if the box is a red one then this block of code will deliver it to the red truck respectively
                if (box8_catched == True) and (red1 == True):
                    if (stateMt.state2.mode == "AUTO.LAND") and (d1_y < 0):
                        ofb_ctl.edrone1_offboard_set_mode()
                        pos1.pose.position.x = d1_x
                        pos1.pose.position.y = y2
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

                    if (d1_y > 0) and (d1_z < 2.5):
                        ofb_ctl.edrone1_land_mode()                        
                        ofb_ctl.edrone1_grip_activate(False)
                        box8_catched = False
                        box8_delivered = True
                        red1 = False

                #if the box is a blue one then this block of code will deliver it to the blue truck respectively
                elif (box8_catched == True) and (blue1 == True):
                    if (stateMt.state2.mode == "AUTO.LAND") and (d1_y > -62):
                        ofb_ctl.edrone1_offboard_set_mode()
                        pos1.pose.position.x = d1_x
                        pos1.pose.position.y = y2
                        pos1.pose.position.z = 3
                        edrone1_local_pos_pub.publish(pos1)
                
                    if (3-(d1_z) < 0.5):
                        pos1.pose.position.x = 16
                        pos1.pose.position.y = y2
                        pos1.pose.position.z = 3
                        edrone1_local_pos_pub.publish(pos1)
                    
                    if (16 - (d1_x) < 0.5):
                        pos1.pose.position.x = 16
                        pos1.pose.position.y = -68
                        pos1.pose.position.z = 3
                        edrone1_local_pos_pub.publish(pos1)

                    if abs(-68 - d1_y) < 0.2:
                        pos1.pose.position.x = 16
                        pos1.pose.position.y = -68
                        pos1.pose.position.z = 1
                        edrone1_local_pos_pub.publish(pos1)
                        
                    if  (d1_y < -62) and (d1_z < 2.5):
                        ofb_ctl.edrone1_land_mode()                        
                        ofb_ctl.edrone1_grip_activate(False)
                        box8_catched = False
                        box8_delivered = True
                        blue1 = False
        
            #after delivering the 8th box drone1 will come back to its initial position and land
            if (box8_delivered == True):
                if (stateMt.state2.mode == "AUTO.LAND") and ((d1_y > 1) or (d1_y < -60)): 
                    ofb_ctl.edrone1_grip_activate(False)
                    ofb_ctl.edrone1_offboard_set_mode()
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

'''
AS SO MANY LINES OF CODE ARE REPEATED, I HAVE NOT MADE A COMMENT AGAIN AND AGAIN FOR THE SAME BUT FOR THE INITIAL TIME I HAVE. THANK YOU!!
'''