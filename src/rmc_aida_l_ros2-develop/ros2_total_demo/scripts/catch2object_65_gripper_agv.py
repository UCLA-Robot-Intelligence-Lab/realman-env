#!/usr/bin/python3
# -*- coding: utf-8 -*-
import numpy as np
import rclpy
from rclpy.node import Node
from scipy.spatial.transform import Rotation as R

import socket
import time
from rclpy.time import Time
from rclpy.duration import Duration

from std_msgs.msg import String, Empty, Bool
from rm_ros_interfaces.msg import Liftheight,Movej,Movejp,Armstate,Armoriginalstate,Gripperset
from ros2_total_demo.msg import ObjectPose

from rclpy.executors import MultiThreadedExecutor
import threading

import sys 
import os
import yaml

class CatchNode(Node):
    def __init__(self):
        super().__init__('catch')
        
        # 获取当前文件的绝对路径
        current_file_path = os.path.abspath(__file__)
        # 获取当前文件所在目录的路径
        current_dir = os.path.dirname(current_file_path)
        config_dir = current_dir + '/config'
        config = self.load_config(os.path.join(config_dir,'config.yaml'))
        #机械臂拍照位姿
        self.ready_pose = config['CAMERA_POSE']
        #机械臂过渡位姿
        self.transition_pose = config['MIDDLE_POSE']
        # 机械臂运动速度
        self.speed = config['SPEED']
        #抓取时的姿态（四元数）
        self.grasping_pose = config['CRAW_BEFORE']

        # agv target name
        self.agv_target_name = config['AGV_TARGET_NAME']
        
        # 发布一个底盘移动的name
        self.agv_pub = self.create_publisher(String, 'goto_mark/go', 10)
        # 订阅底盘移动结果的消息
        self.agv_subscriber = self.create_subscription( String, 'goto_mark/result', self.agv_result_callback, 10) 

        # 发布一个改变工作坐标系的消息
        self.change_work_frame_pub = self.create_publisher(String, '/right_arm_controller/rm_driver/change_work_frame_cmd', 10)
        # 发布一个控制升降机的消息
        self.set_lift_height_pub = self.create_publisher(Liftheight, '/left_arm_controller/rm_driver/set_lift_height_cmd', 10)
        
        # 发布一个控制机械臂movej运动的消息
        self.pub_to_joint = self.create_publisher(Movej, '/right_arm_controller/rm_driver/movej_cmd', 10)
        # 发布一个控制机械臂movej_p运动的消息
        self.pub_to_pose = self.create_publisher(Movejp, '/right_arm_controller/rm_driver/movej_p_cmd', 10) 
        # 发布一个请求机械臂当前状态的消息
        self.pub_get_pose = self.create_publisher(Empty, '/right_arm_controller/rm_driver/get_current_arm_state_cmd', 10)  
        
        # 发布一个请求夹爪开合的消息
        self.pub_set_gripper = self.create_publisher(Gripperset, '/right_arm_controller/rm_driver/set_gripper_position_cmd', 10)
        
        # 订阅机械臂movej运动规划是否成功
        self.movej_plan_result_sub = self.create_subscription(Bool,'/right_arm_controller/rm_driver/movej_result', self.movej_callback,10)

        # 订阅机械臂movej_p运动规划是否成功
        self.movej_p_plan_result_sub = self.create_subscription(Bool,'/right_arm_controller/rm_driver/movej_p_result', self.movej_p_callback,10)
        # 订阅机械臂当前状态-返回各关节弧度和四元数
        self.radian_quaternion_sub = self.create_subscription(Armstate,'/right_arm_controller/rm_driver/get_current_arm_state_result', self.radian_quaternion_callback, 10)  
        # 获取机械臂当前状态-返回各关节角度和欧拉角
        self.joint_euler_sub = self.create_subscription(Armoriginalstate,'/right_arm_controller/rm_driver/get_current_arm_original_state_result',self.joint_euler_callback,10)  
        # 订阅视觉识别物体位置的消息
        self.object_pose_subscriber = self.create_subscription(ObjectPose, 'object_pose_bottle', self.object_pose_callback, 10)
        
        self.object_pose = None
        # 轨迹规划状态
        self.run_state = False
        # 位姿 x y z rx ry rz
        self.joint_euler = None 

        self.radian_quaternion = None 
        
        # agv移动结果
        self.agv_result = '' 
        
        self.change_work_frame()
        
        self.worker_thread = threading.Thread(target=self.worker_function)
        self.worker_thread.start()
        
    def load_config(self,config_path):
    
        with open(config_path, 'r') as stream:
            config = yaml.safe_load(stream)
        return config
        
    def agv_send_goal(self,goal=''): 
        msg = String() 
        msg.data = goal 
        self.get_logger().info('Publishing: "%s"' % msg.data) 
        self.agv_pub.publish(msg)
    
    def change_work_frame(self):
        # 机械臂切换到base坐标系
        # 创建消息 
        msg = String() 
        msg.data = 'Base' 
        # 发布消息 
        self.change_work_frame_pub.publish(msg)
        print('机械臂切换到base坐标系')
        
    def set_gripper(self,position=1000,block=True):
        # 夹爪位置设置   
        msg = Gripperset()
        msg.position = position #手爪目标位置，范围：1～1000,代表手爪开口度：0～70mm
        msg.block = block  #是否未阻塞模式，true：阻塞，false：非阻塞
        self.pub_set_gripper.publish(msg)
        
    # 设置升降机高度 目标高度，单位 mm，范围：0~2600 #速度百分比，1~100  block 是否阻塞   
    def set_lift_height(self, height=100, speed=20,block=True):
        # 创建消息 
        msg = Liftheight() 
        msg.height = height
        msg.speed = speed
        msg.block = block
        # 发布消息 
        self.set_lift_height_pub.publish(msg)
        print(msg)
        #print(f'设置升降机高度，目标高度：{height} mm，速度百分比：{speed}%')
        
    # movej
    def movej(self,joint=[],speed=10,block=True,trajectory_connect=0,dof=6):
        movej=Movej()
        movej.joint=joint
        movej.speed=speed
        movej.block=block
        movej.trajectory_connect=trajectory_connect #0 代表立即规划，1 代表和下一条轨迹一起规划，当为 1 时，轨迹不会立即执行
        movej.dof=dof
        self.pub_to_joint.publish(movej)

    # 机械臂movej_p
    def movej_p(self,pose=[],speed=10,block=True,trajectory_connect=0):
        movejp = Movejp()
        movejp.pose.position.x = pose[0]
        movejp.pose.position.y = pose[1]
        movejp.pose.position.z = pose[2]
        movejp.pose.orientation.x = pose[3]
        movejp.pose.orientation.y = pose[4]
        movejp.pose.orientation.z = pose[5]
        movejp.pose.orientation.w = pose[6]
        movejp.speed = speed
        movejp.trajectory_connect=trajectory_connect #0 代表立即规划，1 代表和下一条轨迹一起规划，当为 1 时，轨迹不会立即执行
        movejp.block = block # 是否阻塞
        self.pub_to_pose.publish(movejp)
        
    def agv_result_callback(self, msg):
        print(msg)
        self.agv_result = msg.data 
                
    def object_pose_callback(self, msg):
        self.object_pose = msg

    def radian_quaternion_callback(self,msg):
        self.radian_quaternion = msg
        print(msg)

    def joint_euler_callback(self,msg):
        self.joint_euler = msg

    def movej_callback(self,msg):
        if msg.data:
            self.run_state = True
            print("*******Plan State OK")
        else:
            self.run_state = False
            print("*******Plan State Fail")

    def movej_p_callback(self,msg):
        if msg.data:
            self.run_state = True
            print("*******Plan State OK")
        else:
            self.run_state = False
            print("*******Plan State Fail")

    def get_pose_pub(self): 
        msg = Empty() 
        self.pub_get_pose.publish(msg) 

    def is_arrive(self):
        """机械臂动作是否规划成功
        """
        # 等待run_state变成True
        while 1:
            time.sleep(0.5)
            if self.run_state:
                self.run_state  = False
                break
        
    def convert(self,x, y, z, x1, y1, z1, rx, ry, rz):
    
        
        #使用深度相机识别到的物体坐标（x, y, z）和机械臂末端的位姿（x1,y1,z1,rx,ry,rz）来计算物体在机械臂基坐标下的位置（x, y, z）
        
        #Args:
        #    x : 相机坐标系下物体位置x
        #    y : 相机坐标系下物体位置y
        #    z : 相机坐标系下物体位置z
        #    x1 : 机械臂末端位姿 x
        #    y1 : 机械臂末端位姿 y
        #    z1 : 机械臂末端位姿 z
        #    rx : 机械臂末端位姿 rx
        #    ry : 机械臂末端位姿 ry
        #    rz : 机械臂末端位姿 rz
    
        #Returns:
        #    _type_: 物体在机械臂基坐标系下的位置 x y z
    
        # 相机坐标系到机械臂末端坐标系的旋转矩阵和平移向量
        rotation_matrix = np.array([[0.0790325, 0.98958949, -0.12027678],[-0.99682423,0.07726969, -0.0192575],[-0.00976327, 0.12141678, 0.9925536]])
        
        translation_vector = np.array([-0.0974,0.041261, 0.00108])
    
        obj_camera_coordinates = np.array([x, y, z])     # 深度相机识别物体返回的坐标
    
        end_effector_pose = np.array([x1, y1, z1,
                                      rx, ry, rz])     # 机械臂末端的位姿，单位为弧度
    
        # 将旋转矩阵和平移向量转换为齐次变换矩阵
        T_camera_to_end_effector = np.eye(4)
        T_camera_to_end_effector[:3, :3] = rotation_matrix
        T_camera_to_end_effector[:3, 3] = translation_vector
        # 机械臂末端的位姿转换为齐次变换矩阵
        position = end_effector_pose[:3]
        orientation = R.from_euler('xyz', end_effector_pose[3:], degrees=False).as_matrix()
        T_base_to_end_effector = np.eye(4)
        T_base_to_end_effector[:3, :3] = orientation
        T_base_to_end_effector[:3, 3] = position
        # 计算物体相对于机械臂基座的位姿
        obj_camera_coordinates_homo = np.append(obj_camera_coordinates, [1])  # 将物体坐标转换为齐次坐标
        # obj_end_effector_coordinates_homo = np.linalg.inv(T_camera_to_end_effector).dot(obj_camera_coordinates_homo)
        obj_end_effector_coordinates_homo = T_camera_to_end_effector.dot(obj_camera_coordinates_homo)
        obj_base_coordinates_homo = T_base_to_end_effector.dot(obj_end_effector_coordinates_homo)
        obj_base_coordinates = obj_base_coordinates_homo[:3]  # 从齐次坐标中提取物体的x, y, z坐标
    
        x, y, z = obj_base_coordinates

        return x, y, z
                
    def worker_function(self):
        
        # 1.agv到达目标点
        self.agv_send_goal(self.agv_target_name) #'7613B5D' 
        
        while self.agv_result != "任务完成":
            print("等待地盘移动完成")
            time.sleep(1) # 添加1秒的休眠
        self.agv_result = ''
        print("地盘移动完成！！！！！！！！！！！！！！！！")
        
        # 2.打开末端执行器 
        # 2.1打开夹爪  手爪目标位置，范围：1～1000,代表手爪开口度：0～70mm
        self.set_gripper(1000,True)
        
        # 3.升降机到达视觉可看到要抓取的物体的位置
        self.set_lift_height(360,50)

        # 4.机械臂到达准备抓取位置
        self.movej_p(self.ready_pose,self.speed)
        self.is_arrive()
        
        # 5.获取当前机械臂状态
        self.get_pose_pub()
        # 判断机械臂当前状态数据是否获取，如果获取不到，不会执行后续程序
        while self.joint_euler is None:
            self.get_logger().info("Waiting for self.joint_euler to be updated...")
            time.sleep(1)
        x1 = self.joint_euler.pose[0]
        y1 = self.joint_euler.pose[1]
        z1 = self.joint_euler.pose[2]
        rx = self.joint_euler.pose[3]
        ry = self.joint_euler.pose[4]
        rz = self.joint_euler.pose[5]
        print(self.joint_euler)
        self.joint_euler = None
        # 6.获取相机识别到物体的坐标
        while self.object_pose is None or (self.object_pose.x == 0 and self.object_pose.y == 0 and self.object_pose.z == 0):
            self.get_logger().info("Waiting for camera coordinate...")
            time.sleep(1)
        camera_x = self.object_pose.x
        camera_y = self.object_pose.y
        camera_z = self.object_pose.z
        print(self.object_pose)
        self.object_pose = None
 
        # 7.计算物体在基座标的位置
        x,y,z=self.convert(camera_x,camera_y,camera_z,x1,y1,z1,rx,ry,rz)

        # 8.机械臂取抓取物体
        # 8.1机械臂到达过渡位置
        self.movej_p(self.transition_pose,self.speed)
        self.is_arrive()

        # 8.2机械臂到达物体前方位置
        pose=[x,y+0.1,z,self.grasping_pose[0],self.grasping_pose[1],self.grasping_pose[2],self.grasping_pose[3]]
        self.movej_p(pose,self.speed)
        self.is_arrive()

        # 8.3机械臂到达物体位置
        pose=[x,y,z,self.grasping_pose[0],self.grasping_pose[1],self.grasping_pose[2],self.grasping_pose[3]]
        self.movej_p(pose,self.speed)
        self.is_arrive()

        # 9.关闭夹爪 手爪目标位置，范围：1～1000,代表手爪开口度：0～70mm
        self.set_gripper(10,True)

        # 10.机械臂回到初始状态
        # 10.1机械臂到达物体上方位置
        pose=[x+0.05,y+0.05,z+0.05,self.grasping_pose[0],self.grasping_pose[1],self.grasping_pose[2],self.grasping_pose[3]]
        self.movej_p(pose,self.speed)
        self.is_arrive()

        # 10.2机械臂到达过渡位置
        self.movej_p(self.transition_pose,self.speed)
        self.is_arrive()

        # 10.3机械臂到达准备抓取位置
        self.movej_p(self.ready_pose,self.speed)
        self.is_arrive()
        
        # 到达目标点
        #self.agv_send_goal('763799E') #'7613B5D'
        #while self.agv_result != "任务完成":
        #    print("等待地盘移动完成")
        #    time.sleep(1) # 添加1秒的休眠
        #print("地盘移动完成！！！！！！！！！！！！！！！！")


def main(args=None):
    rclpy.init(args=args)
    node = CatchNode()
    executor = MultiThreadedExecutor(num_threads=6)
    executor.add_node(node)
    try:
        executor.spin()
    except KeyboardInterrupt:
        pass
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
