#!/usr/bin/env python
import rospy
from std_msgs.msg import Bool, Float64, Float32MultiArray
from geometry_msgs.msg import Pose, PoseArray, PoseStamped, Point, Twist
from nav_msgs.msg import Path, Odometry, OccupancyGrid
import numpy as np
import tf

from MPC_Ackerman import MPC
# from MPC_Differential import MPC

from sensor_msgs.msg import LaserScan
from visualization_msgs.msg import Marker,MarkerArray
from std_srvs.srv import SetBool
import math

robot_type = "hunter" # hunter || scout

class Local_Planner():
    def __init__(self):
        self.replan_period = rospy.get_param('/local_planner/replan_period', 0.05) 
        self.curr_state = np.zeros(5)   #matrix 1*5
        self.z = 0.0                    
        self.N = 10                     
        self.goal_state = np.zeros((self.N,4))   # matrix 10*4 changed by JP               
        self.desired_global_path = [ np.zeros([300,4]) , 0]    
        self.have_plan = False          
        self.robot_state_set = False    
        self.ref_path_set = False       
        self.is_end=0                   
        self.__timer_replan = rospy.Timer(rospy.Duration(self.replan_period), self.__replan_cb)                     
        self.__pub_local_path = rospy.Publisher('mpc/local_path', Path, queue_size=10)                              
        if robot_type == "hunter":
            self.__pub_rtc_cmd = rospy.Publisher('/robot1/ackermann_steering_controller/cmd_vel', Twist, queue_size=10)                                     
            self._sub_odom = rospy.Subscriber('/robot1/ackermann_steering_controller/odom', Odometry, self.__odom_cb) #receive hunter SE's odometry, convert to curr_state(matix1*5):[x,y,\psi,0,0]                      
        elif robot_type == "scout":
            self.__pub_rtc_cmd = rospy.Publisher('/robot1/cmd_vel', Twist, queue_size=10)                                     
            self._sub_odom = rospy.Subscriber('/scout1/odom', Odometry, self.__odom_cb)
                                               
        self._sub_traj_waypts = rospy.Subscriber('/mpc/traj_point', Float32MultiArray, self._vomp_path_callback) 
        
        self.control_cmd = Twist()      
        


    def __odom_cb(self,data):
        
        self.robot_state_set = True
        self.curr_state[0] = data.pose.pose.position.x      
        self.curr_state[1] = data.pose.pose.position.y      
       
        roll, pitch, self.curr_state[2] = self.quart_to_rpy(
            data.pose.pose.orientation.x, data.pose.pose.orientation.y, data.pose.pose.orientation.z, data.pose.pose.orientation.w)
        self.curr_state[3] = 0.0    
        self.curr_state[4] = 0.0
        self.z = data.pose.pose.position.z  

    # quaternion and euler angle Conversion
    def quart_to_rpy(self, x, y, z, w):
        r = math.atan2(2*(w*x+y*z), 1-2*(x*x+y*y))
        p = math.asin(2*(w*y-z*x))
        y = math.atan2(2*(w*z+x*y), 1-2*(z*z+y*y))
        return r, p, y

    # Enable MPC on time to solve optimal control
    def __replan_cb(self, event):
        if self.robot_state_set and self.ref_path_set:
            self.choose_goal_state() #deal with goal_state matrix 10*4
            start_time = rospy.Time.now()
            states_sol, input_sol = MPC(np.expand_dims(self.curr_state, axis=0),self.goal_state) # elevate curr_state's dimension in axis0
            end_time = rospy.Time.now()
            rospy.loginfo('[pHRI Planner] phri solved in {} sec'.format((end_time-start_time).to_sec()))

            if(self.is_end == 0):
                self.__publish_local_plan(input_sol,states_sol)
                self.cmd(input_sol)
            self.have_plan = True
        elif self.robot_state_set==False and self.ref_path_set==True:
            print("no robot pose")
        elif self.robot_state_set==True and self.ref_path_set==False:
            print("no refer path")
        else:
            print("no path and no pose")
        
   
    def __publish_local_plan(self,input_sol,state_sol):
        local_path = Path()
        sequ = 0
        local_path.header.stamp = rospy.Time.now()
        local_path.header.frame_id = "/world"

        for i in range(self.N):
            this_pose_stamped = PoseStamped()
            this_pose_stamped.pose.position.x = state_sol[i,0]
            this_pose_stamped.pose.position.y = state_sol[i,1]
            this_pose_stamped.pose.position.z = self.z+0.2          
            this_pose_stamped.header.seq = sequ
            sequ += 1
            this_pose_stamped.header.stamp = rospy.Time.now()
            this_pose_stamped.header.frame_id="/world"
            local_path.poses.append(this_pose_stamped)

        self.__pub_local_path.publish(local_path)

    # return 2 points's Euclidean distance
    def distance_global(self,c1,c2):
        distance = np.sqrt((c1[0]-c2[0])**2+(c1[1]-c2[1])**2)
        return distance
    
    # return index of the nearest point to the robot1 in desired_global_path
    def find_min_distance(self,c1):
        number =  np.argmin( np.array([self.distance_global(c1,self.desired_global_path[0][i]) for i in range(self.desired_global_path[1])]) ) #numpy.argmin() return index of the min_value from array
        return number 

    
    def choose_goal_state(self):
        
        
        num = self.find_min_distance(self.curr_state)
        scale = 1
        num_list = []
        # get 10 points from desired_global_path, and put them in goal_state
        for i in range(self.N):  
            num_path = min(self.desired_global_path[1]-1,int(num+i*scale)) # unknown
            num_list.append(num_path)
        if(num  >= self.desired_global_path[1]): #it means num=10
            self.is_end = 1 # ?
        for k in range(self.N):
            self.goal_state[k] = self.desired_global_path[0][num_list[k]]

   
    def _vomp_path_callback(self, data):
        if(len(data.data)!=0):
            self.ref_path_set = True
            size = len(data.data)//3
            self.desired_global_path[1]=size
            
            car_yaw = self.curr_state[2]    
            for i in range(size):
                self.desired_global_path[0][i,0]=data.data[3*(size-i)-3]
                self.desired_global_path[0][i,1]=data.data[3*(size-i)-2]
                
                if(data.data[3*(size-i)-1] - car_yaw > 3.14):
                    self.desired_global_path[0][i,2] = data.data[3*(size-i)-1] - 2.0*np.pi
                elif(data.data[3*(size-i)-1] - car_yaw < -3.14):
                    self.desired_global_path[0][i,2] = data.data[3*(size-i)-1] + 2.0*np.pi
                else:
                    self.desired_global_path[0][i,2]=data.data[3*(size-i)-1]
                
                self.desired_global_path[0][i,3]=0.0

    
    def cmd(self, data):
        self.control_cmd.linear.x = data[0][0]
        self.control_cmd.angular.z = data[0][1]
        self.__pub_rtc_cmd.publish(self.control_cmd)
# 

if __name__ == '__main__':

    rospy.init_node("MPC_Traj_follower")
    phri_planner = Local_Planner()
    rospy.spin()
