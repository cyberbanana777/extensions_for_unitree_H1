from unitree_go.msg import MotorState
from unitree_go.msg import MotorStates
from unitree_go.msg import MotorCmd
from unitree_go.msg import MotorCmds
from unitree_go.msg import LowCmd
from unitree_sdk2py.utils.crc import CRC
from unitree_go.msg import LowState
from std_msgs.msg import String
from rclpy.node import Node
import rclpy
import numpy as np
import time
import json

JOINT_INDEX_H1 = {
    'right_hip_roll_joint': 0,
    'right_hip_pitch_joint': 1,
    'right_knee_joint': 2,
    'left_hip_roll_joint': 3,
    'left_hip_pitch_joint': 4,
    'left_knee_joint': 5,
    'torso_joint': 6,
    'left_hip_yaw_joint': 7,
    'right_hip_yaw_joint': 8,
    'NOT USED': 9,
    'left_ankle_joint': 10,
    'right_ankle_joint': 11,
    'right_shoulder_roll_joint': 12,
    'right_shoulder_pitch_joint': 13,
    'right_shoulder_yaw_joint': 14,
    'right_elbow_joint': 15,
    'left_shoulder_roll_joint': 16,
    'left_shoulder_pitch_joint': 17,
    'left_shoulder_yaw_joint': 18,
    'left_elbow_joint': 19
}

class LowLevelControlNode(Node):

    def __init__(self):
        super().__init__('low_level_control_node')
        self.get_logger().info('Node started')


        self.active_joints_H1 = [
            JOINT_INDEX_H1['right_shoulder_roll_joint'],
            JOINT_INDEX_H1['right_shoulder_pitch_joint'],
            JOINT_INDEX_H1['right_shoulder_yaw_joint'],
            JOINT_INDEX_H1['right_elbow_joint'],
            JOINT_INDEX_H1['left_shoulder_pitch_joint'],
            JOINT_INDEX_H1['left_shoulder_roll_joint'],
            JOINT_INDEX_H1['left_shoulder_yaw_joint'],
            JOINT_INDEX_H1['left_elbow_joint']
        ]

                # текущая позиция
        self.current_jpos_H1 = {
            0: 0.0,  # right_hip_roll_joint M
            1: 0.0,  # right_hip_pitch_joint M
            2: 0.0,  # right_knee_joint L
            3: 0.0,  # left_hip_roll_joint M
            4: 0.0,  # left_hip_pitch_joint M
            5: 0.0,  # left_knee_joint L
            6: 0.0,  # torso_joint M
            7: 0.0,  # left_hip_yaw_joint M
            8: 0.0,  # right_hip_yaw_joint M
            9: 0.0,  # NOT USED
            10: 0.0,  # left_ankle_joint S
            11: 0.0,  # right_ankle_joint S
            12: 0.0,  # right_shoulder_pitch_joint M
            13: 0.0,  # right_shoulder_roll_joint M
            14: 0.0,  # right_shoulder_yaw_joint M
            15: 0.0,  # right_elbow_joint M
            16: 0.0,  # left_shoulder_pitch_joint M
            17: 0.0,  # left_shoulder_roll_joint M
            18: 0.0,  # left_shoulder_yaw_joint M
            19: 0.0  # left_elbow_joint M
        }


        # подписка на топик, в который публикуется информация о углах поворота звеньев
        self.subscription_LowCmd = self.create_subscription(
            LowState,
            'lowstate',
            self.listener_callback_LowCmd,
            10)
        
    def listener_callback_LowCmd(self, msg):
        '''Обновляем текущее положение рук'''
        for i in self.active_joints_H1:
            self.current_jpos_H1[i] = round(msg.motor_state[i].q, 3)
        print(self.current_jpos_H1)

def main(args=None):
    rclpy.init(args=args)
    node = LowLevelControlNode()

    try:
        rclpy.spin(node)

    except KeyboardInterrupt:
        node.get_logger().info('Stop node.')

    except Exception as e:
        node.get_logger().error(e)

    finally:
        node.get_logger().info('Node stoped.')
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
