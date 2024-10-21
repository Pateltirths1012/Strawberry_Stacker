#!/usr/bin/env python3


'''
This is a boiler plate script that contains hint about different services that are to be used
to complete the task.
Use this code snippet in your code or you can also continue adding your code in the same file


This python file runs a ROS-node of name offboard_control which controls the drone in offboard mode. 
See the documentation for offboard mode in px4 here() to understand more about offboard mode 
This node publishes and subsribes the following topics:

	 Services to be called                   Publications                                          Subscriptions				
	/mavros/cmd/arming                       /mavros/setpoint_position/local                       /mavros/state
    /mavros/set_mode                         /mavros/setpoint_velocity/cmd_vel                     /mavros/local_position/pose   
         
    
'''



import rospy
from geometry_msgs.msg import *
from mavros_msgs.msg import *
from mavros_msgs.srv import *
from rospy.client import set_param
from rospy.exceptions import ROSException


class offboard_control:


    def __init__(self):
        # Initialise rosnode
        rospy.init_node('offboard_control', anonymous=True)

    def setArm(self):
        # Calling to /mavros/cmd/arming to arm the drone and print fail message on failure
        rospy.wait_for_service('mavros/cmd/arming')  # Waiting untill the service starts 
        try:
            armService = rospy.ServiceProxy('mavros/cmd/arming', mavros_msgs.srv.CommandBool) # Creating a proxy service for the rosservice named /mavros/cmd/arming for arming the drone 
            armService(True)
        except rospy.ServiceException as e:
            print ("Service arming call failed: %s"%e)

        # Similarly delacre other service proxies 

    def offboard_set_mode(self):
        # Call /mavros/set_mode to set the mode the drone to OFFBOARD
        # and print fail message on failure
        rospy.wait_for_service('mavros/set_mode')
        try:
            offset = rospy.ServiceProxy('mavros/set_mode',mavros_msgs.srv.SetMode)
            offset(5,"OFFBOARD")
        except rospy.ServiceException as e:
            print("Servive set_mode call failed: %s"%e)

    def land_mode(self):
        rospy.wait_for_service('mavros/set_mode')
        try:
            offset = rospy.ServiceProxy('mavros/set_mode',mavros_msgs.srv.SetMode)
            offset(0,"AUTO.LAND")
        except rospy.ServiceException as e:
            print("Servive set_mode call failed: %s"%e)
   
class stateMoniter:
    def __init__(self):
        self.state = State()
        # Instantiate a setpoints message

        
    def stateCb(self, msg):
        # Callback function for topic /mavros/state
        self.state = msg

    # Create more callback functions for other subscribers    
    def local_Position_Cb(self, data):
        self.local_position = data

def main():


    stateMt = stateMoniter()
    ofb_ctl = offboard_control()
    

    # Initialize publishers
    local_pos_pub = rospy.Publisher('mavros/setpoint_position/local', PoseStamped, queue_size=10)
    local_vel_pub = rospy.Publisher('mavros/setpoint_velocity/cmd_vel', Twist, queue_size=10)
    # Specify the rate 
    rate = rospy.Rate(20.0)

    # Make the list of setpoints 
    setpoints = [[0,0,10],[10,0,10],[10,10,10],[0,10,10],[0,0,10]] #List to setpoints

    # Similarly initialize other publishers 

    # Create empty message containers 
    pos = PoseStamped()
    pos.pose.position.x = 0
    pos.pose.position.y = 0
    pos.pose.position.z = 10

    # Set your velocity here
    vel = Twist()
    vel.linear.x = 5
    vel.linear.y = 0
    vel.linear.z = 0
    
    # Similarly add other containers 

    # Initialize subscriber 
    rospy.Subscriber("/mavros/state",State, stateMt.stateCb)

    # Similarly initialize other subscribers 
    rospy.Subscriber("mavros/local_position/pose",PoseStamped, stateMt.local_Position_Cb)

    '''
    NOTE: To set the mode as OFFBOARD in px4, it needs atleast 100 setpoints at rate > 10 hz, so before changing the mode to OFFBOARD, send some dummy setpoints  
    '''
    for i in range(100):
        local_pos_pub.publish(pos)
        rate.sleep()


    # Arming the drone
    while not stateMt.state.armed:
        ofb_ctl.setArm()
        rate.sleep()
    print("Armed!!")

    # Switching the state to auto mode
    
    while not stateMt.state.mode=="OFFBOARD":
        ofb_ctl.offboard_set_mode()
        rate.sleep()
    print ("OFFBOARD mode activated")
    
    
    # Publish the setpoints 
    while not rospy.is_shutdown():
        '''
        Step 1: Set the setpoint 
        Step 2: Then wait till the drone reaches the setpoint, 
        Step 3: Check if the drone has reached the setpoint by checking the topic /mavros/local_position/pose 
        Step 4: Once the drone reaches the setpoint, publish the next setpoint , repeat the process until all the setpoints are done  

        Write your algorithm here 
        '''
        
        pos.pose.position.x = setpoints[0][0]
        pos.pose.position.y = setpoints[0][1]
        pos.pose.position.z = setpoints[0][2]
        local_pos_pub.publish(pos)
        local_vel_pub.publish(vel)
        

        if (10 - stateMt.local_position.pose.position.z < 1):
            pos.pose.position.x = setpoints[1][0]
            pos.pose.position.y = setpoints[1][1]
            pos.pose.position.z = setpoints[1][2]
            local_pos_pub.publish(pos)
            local_vel_pub.publish(vel)
            
            
        if (10 - stateMt.local_position.pose.position.x < 1): 
            pos.pose.position.x = setpoints[2][0]
            pos.pose.position.y = setpoints[2][1]
            pos.pose.position.z = setpoints[2][2]
            local_pos_pub.publish(pos)
            local_vel_pub.publish(vel)
            
        
        if (10 - stateMt.local_position.pose.position.y < 1):
            pos.pose.position.x = setpoints[3][0]
            pos.pose.position.y = setpoints[3][1]
            pos.pose.position.z = setpoints[3][2]
            local_pos_pub.publish(pos)
            local_vel_pub.publish(vel)
            

        if (stateMt.local_position.pose.position.x < 1) and (stateMt.local_position.pose.position.y > 1): 
            pos.pose.position.x = setpoints[4][0]
            pos.pose.position.y = setpoints[4][1]
            pos.pose.position.z = setpoints[4][2]
            local_pos_pub.publish(pos)
            local_vel_pub.publish(vel)
            
        if (int(stateMt.local_position.pose.position.x) == setpoints[4][0]) and (int(stateMt.local_position.pose.position.y) == setpoints[4][1]) and (int(stateMt.local_position.pose.position.z) == setpoints[4][2]):
            ofb_ctl.land_mode()

        rate.sleep()

if __name__ == '__main__':
    try:
        main()
    except rospy.ROSInterruptException:
        pass
    