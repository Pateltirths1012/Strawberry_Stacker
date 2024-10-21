
from gazebo_msgs.msg import*
import gazebo_ros_link_attacher
import rospy
from geometry_msgs.msg import*
from mavros_msgs.msg import*
from mavros_msgs.srv import*
from std_msgs.msg import*
from gazebo_ros_link_attacher.srv import*
from rospy import exceptions
from rospy.client import set_param
from rospy.exceptions import ROSException


class pick_and_place:

    def __init__(self):
        rospy.init_node('pick_n_place',anonymous=True)
    
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
    
    
    #calling the ros srevice activate gripper
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
    
    def stateCb(self, msg):
        self.state = msg
    
    def local_position_cb(self, data):
        self.local_position = data
    
    def gripper_cb(self, data): #callback function for gripper check
        self.gripper_check = data

def main():
    stateMt = stateMonitor()
    ofb_ctl = pick_and_place()

    local_pos_pub = rospy.Publisher('mavros/setpoint_position/local', PoseStamped, queue_size=10)
    local_vel_pub = rospy.Publisher('mavros/setpoint_velocity/cmd_vel', Twist, queue_size=10)

    rate = rospy.Rate(20.0)
    
    pos = PoseStamped()
    pos.pose.position.x = 0
    pos.pose.position.y = 0
    pos.pose.position.z = 3

    vel = Twist()
    vel.linear.x = 5
    vel.linear.y = 0
    vel.linear.z = 0

    rospy.Subscriber("/mavros/state",State, stateMt.stateCb)
    rospy.Subscriber("mavros/local_position/pose",PoseStamped, stateMt.local_position_cb)
    rospy.Subscriber("gripper_check",String, stateMt.gripper_cb) #subscribing to the topic gripper check

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
    objectCatched = False
    delivered = False

    

    while not rospy.is_shutdown():
        # Reaching the First setpoint
        #if the drone reaches 3m then move forward 3m
        if  (delivered == False) and (3 - (stateMt.local_position.pose.position.z) < 1):
            pos.pose.position.x = 3.25
            pos.pose.position.y = 0.43
            pos.pose.position.z = 3
            local_pos_pub.publish(pos)
            local_vel_pub.publish(vel)
        
        #if the drone reaches x = 3 then land on the object
        if (delivered == False) and (3.25 - (stateMt.local_position.pose.position.x) < 0.5) and (stateMt.local_position.pose.position.y < 0.5):
            ofb_ctl.land_mode()
            #once landed then grab the object
            if (stateMt.gripper_check.data == "True"):
                ofb_ctl.grip_activate(True)
                objectCatched = True
        
        # if the objectcatched is true then takeoff to 3m height
        if (objectCatched == True):
            ofb_ctl.offboard_set_mode()
            pos.pose.position.x = 3
            pos.pose.position.y = 0
            pos.pose.position.z = 3
            local_pos_pub.publish(pos)
            local_vel_pub.publish(vel)
            
        
        #once it reached the height then move to the next setpoint i.e. destination
        if (objectCatched == True) and (3 - (stateMt.local_position.pose.position.z) < 1) :
            pos.pose.position.x = 3
            pos.pose.position.y = 3
            pos.pose.position.z = 3
            local_pos_pub.publish(pos)
            local_vel_pub.publish(vel)
            
        
        #if the drone reached the destination then descend and drop the object
        if (objectCatched == True) and (3 - (stateMt.local_position.pose.position.y) < 0.5):
            ofb_ctl.land_mode()

        #algorithm for dropping the object
            if (stateMt.local_position.pose.position.z < 0.3):
                ofb_ctl.grip_activate(False)
                objectCatched = False
                delivered = True

        #if the object is dropped then take off to the height of 3m 
        if (delivered == True) and (stateMt.state.mode=="AUTO.LAND"):
            ofb_ctl.offboard_set_mode()
            pos.pose.position.x = 3
            pos.pose.position.y = 3
            pos.pose.position.z = 3
            local_pos_pub.publish(pos)
            local_vel_pub.publish(vel)
        
        #if the height is reached then move to the origin point and land
        if (delivered == True) and (3 - (stateMt.local_position.pose.position.y) < 1) and (3 - (stateMt.local_position.pose.position.x) < 1) and (3 - (stateMt.local_position.pose.position.z) < 1):
            pos.pose.position.x = 0
            pos.pose.position.y = 0
            pos.pose.position.z = 3
            local_pos_pub.publish(pos)
            local_vel_pub.publish(vel)
            
        if (delivered == True) and ((stateMt.local_position.pose.position.x) < 0.3) and ((stateMt.local_position.pose.position.y) < 0.3) and (3 - (stateMt.local_position.pose.position.z) < 1):
            ofb_ctl.land_mode()
            
        rate.sleep()

if __name__ == '__main__':
    try:
        main()
    except rospy.ROSInterruptException:
        pass

    