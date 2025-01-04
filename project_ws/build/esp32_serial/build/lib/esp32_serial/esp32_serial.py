import rclpy
from rclpy.node import Node
from rclpy.clock import Clock
from rclpy.constants import S_TO_NS
import serial
import time
import math
import numpy as np
import rclpy.time

from nav_msgs.msg import Odometry
from sensor_msgs.msg import Imu
from geometry_msgs.msg import Twist 


    # if ser.in_waiting > 0:
    #     line = ser.readline().decode('utf-8')
    #     print(line)

transmission_raito = 168
wheel_radius = 0.05
l_x = 0.011
l_y = 0.015



def quaternion_from_euler(roll, pitch, yaw):
    """Convert Euler angles to quaternion.
    
    Args:
        roll (float): Rotation around the x-axis (in radians).
        pitch (float): Rotation around the y-axis (in radians).
        yaw (float): Rotation around the z-axis (in radians).

    Returns:
        np.ndarray: Quaternion represented as [x, y, z, w].
    """
    roll /= 2.0
    pitch /= 2.0
    yaw /= 2.0
    ci = math.cos(roll)
    si = math.sin(roll)
    cj = math.cos(pitch)
    sj = math.sin(pitch)
    ck = math.cos(yaw)
    sk = math.sin(yaw)

    cc = ci*ck
    cs = ci*sk
    sc = si*ck
    ss = si*sk

    q = np.empty((4, ))
    q[0] = cj*sc - sj*cs
    q[1] = cj*ss + sj*cc
    q[2] = cj*cs - sj*sc
    q[3] = cj*cc + sj*ss
    return q    


class ESP32_Serial(Node):

    def __init__(self):
        super().__init__('ESP32_node')
        # init publisher for odometry and imu
        self.odometry_publisher_ = self.create_publisher(Odometry, 'demo/odom', 10)
        self.IMU_publisher_ = self.create_publisher(Imu,'demo/imu', 10)
        self.vel_sub_=self.create_subscription(Twist,'/cmd_vel',self.vel_callback,10)

        timer_period = 0.02  # seconds 
        # this timer check and send data every 0.01 second
        self.timer = self.create_timer(timer_period, self.timer_callback)
        self.ser = serial.Serial('/dev/ttyUSB0', 250000,timeout= 0)
        self.ser.reset_input_buffer()
        self.data = {}


        self.receive_data1  = ""

        self.count = 0
        self.change_array = 0


        self.theta_ = 0.00
        self.position_x = 0.00
        self.position_y = 0.00

        self.linear_x = 0.00
        self.linear_y = 0.00
        self.angular_z = 0.00
        
        self.vel_cmd_angular_z = 0.0
        self.vel_cmd_linear_y  = 0.0
        self.vel_cmd_linear_x  = 0.0

        time.sleep(5)
        self.prev_time_ = self.get_clock().now()
        
    
    def vel_callback(self,msg :Twist):
        self.vel_cmd_angular_z = msg.angular.z
        self.vel_cmd_linear_y  = msg.linear.y
        self.vel_cmd_linear_x  = msg.linear.x 
        sending_msg = (str(self.vel_cmd_linear_x) + "/" + str(self.vel_cmd_linear_y) + "/" 
                       + str(self.vel_cmd_angular_z) + "/" +"\n")
        print(sending_msg)
        self.ser.write(sending_msg.encode('utf-8'))                    
        

    def timer_callback(self):
        if self.ser.in_waiting > 0:
           line = self.ser.readline().decode('utf-8')
           for i in range(0,len(line)):
              if(line[i] !="/"):
                self.receive_data1 = self.receive_data1 + line[i]
              else:
                parts = self.receive_data1.split(";")
                self.Send_data(parts)
                self.receive_data1  = ""

########################################################

        #    parts = line.split()
        #    for i in range(0, len(parts)):
        #       if(self.count <= 6):
        #         self.receive_data1[self.count] = parts[i]  
        #       elif(self.count > 6 and self.count<14):
        #         self.receive_data2[self.count-7] = parts[i]

        #       next = self.count + 1
        #       if next == 14:
        #         next = 0
        #       self.count = next    
        
        # if(self.change_array == 0 and (len(self.receive_data1)==7)):
        #    self.change_array = 1
        #    self.Send_data(self.receive_data1)
        #    self.receive_data1  = {}
        # elif(self.change_array == 1 and (len(self.receive_data2)==7)):
        #    self.change_array = 0  
        #    self.receive_data2  = {}
        #    self.Send_data(self.receive_data2)
            
             
    def Send_data(self,data_array):

           # 0, 1, 2, 3 are the number for encoder
           # 4, 5, 6 are the number for IMU sensor
           for i in range(0, len(data_array)):
               key = i
               so_am = 1
               if(data_array[i][0]=="-"):
                  data_array[i] = data_array[i].replace("-", "0")  
                  so_am = -1  
               value = float(data_array[i]) * so_am
               self.data[key] = value 

        #    print(data_array)   
           current_time = self.get_clock().now()
           dt = current_time - self.prev_time_   

           self.from_encoder_pulse_to_odom(self.data[0], self.data[1], self.data[2], self.data[3],dt)
           self.publish_odometry(current_time)
           
           self.imu_linear_acceleration_x = self.data[4]
           self.imu_linear_acceleration_y = self.data[5]
           self.imu_angular_velocity_z = self.data[6]
           self.publish_IMU_data()


        #    self.update_data(self.imu_linear_acceleration_x,self.imu_linear_acceleration_y,
        #                     self.imu_angular_velocity_z,dt)
           
        #    self.publish_odometry(current_time)
           self.prev_time_    = current_time





    def from_encoder_pulse_to_odom(self,en1, en2, en3, en4,dt):

        d_x =    (2*wheel_radius*3.14159)*(1/(11*transmission_raito*4))*(en1+en2+en3+en4)
        d_y =    (2*wheel_radius*3.14159)*(1/(11*transmission_raito*4))*(-en1+en2+en3-en4)
        d_rot =  (2*wheel_radius*3.14159)*(1/(11*transmission_raito*4*(l_x+l_y)))*(-en1+en2-en3+en4)
        
        self.linear_x = d_x/ (dt.nanoseconds / S_TO_NS)  
        self.linear_y = d_y/ (dt.nanoseconds / S_TO_NS)
        self.angular_z = d_rot/ (dt.nanoseconds / S_TO_NS)





    def publish_IMU_data(self):
        imu_msg = Imu()

        # Add the timestamp and frame_id to the header
        imu_msg.header.stamp = self.get_clock().now().to_msg()
        imu_msg.header.frame_id = 'odom'
         
        # Simulate orientation data (quaternion)
        # quat = quaternion_from_euler(0,0, self.theta_)
        # imu_msg.orientation.x = quat[0]
        # imu_msg.orientation.y = quat[1]
        # imu_msg.orientation.z = quat[2]
        # imu_msg.orientation.w = quat[3]

        # Simulate angular velocity (random values to simulate motion)
        imu_msg.angular_velocity.x = 0.00
        imu_msg.angular_velocity.y = 0.00
        imu_msg.angular_velocity.z = self.imu_angular_velocity_z
        
        imu_msg.linear_acceleration.x = self.imu_linear_acceleration_x
        imu_msg.linear_acceleration.y = self.imu_linear_acceleration_y
        imu_msg.linear_acceleration.z = 9.81


        # Optionally, set covariance (we'll set it to zero here)
        # imu_msg.orientation_covariance = [0.0] * 9
        # imu_msg.angular_velocity_covariance = [0.0] * 9
        # imu_msg.linear_acceleration_covariance = [0.0] * 9

        # Publish the IMU message
        self.IMU_publisher_.publish(imu_msg)
     
    def publish_odometry(self, current_time):
        odom = Odometry()
        # print(self.position_x)
        # print(self.position_y)
        # print(self.orientation_z)
        odom.header.stamp = self.get_clock().now().to_msg()
        
        odom.header.frame_id = 'odom'
        odom.child_frame_id = 'base_footprint'

        # Position
        # odom.pose.pose.position.x = self.position_x
        # odom.pose.pose.position.y = self.position_y
      

        # Orientation
        quat = quaternion_from_euler(0,0, self.theta_)
        odom.pose.pose.orientation.x = quat[0]
        odom.pose.pose.orientation.y = quat[1]
        odom.pose.pose.orientation.z = quat[2]
        odom.pose.pose.orientation.w = quat[3]

        # Velocity
        odom.twist.twist.linear.x = self.linear_x
        odom.twist.twist.linear.y = self.linear_y
        odom.twist.twist.angular.z = self.angular_z
        

        # Publish the odometry message
        self.odometry_publisher_.publish(odom)

    def nanoseconds_to_seconds(self,nanoseconds):
        sec = int(nanoseconds /1000000)
        sec = sec /1000
        return sec
            

def main(args=None):
    rclpy.init(args=args)

    esp32 = ESP32_Serial()

    rclpy.spin(esp32)

    # Destroy the node explicitly
    # (optional - otherwise it will be done automatically
    # when the garbage collector destroys the node object)
    esp32.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
