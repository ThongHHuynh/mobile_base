import rclpy
from rclpy.node import Node
from sensor_msgs.msg import JointState
import numpy as np

class FKNode(Node):
    def __init__(self):
        super().__init__('fk_node')
        self.subscription = self.create_subscription(
            JointState,
            '/joint_states',
            self.joint_state_callback,
            10
        )

        # Define DH parameters
        self.alpha = [90, 0, 90, -90, 90, 0]
        self.d     = [20, 0, 0, 67.5, 22.5, 10]
        self.a     = [0, 52.5, 30, 0, 0, 0]
        self.theta_offset = [0, 90, 0, 0, 0, 0]  # adjust based on your DH table

        # Order of joints in your robot
        self.joint_order = ['J1', 'J2', 'J3', 'J4', 'J5', 'J6']

    def dh_matrix(self, theta_deg, alpha_deg, d, a):
        theta = np.radians(theta_deg)
        alpha = np.radians(alpha_deg)
        return np.array([
            [np.cos(theta), -np.sin(theta)*np.cos(alpha),  np.sin(theta)*np.sin(alpha), a*np.cos(theta)],
            [np.sin(theta),  np.cos(theta)*np.cos(alpha), -np.cos(theta)*np.sin(alpha), a*np.sin(theta)],
            [0,              np.sin(alpha),                np.cos(alpha),               d],
            [0,              0,                            0,                           1]
        ])

    def joint_state_callback(self, msg: JointState):
        joint_map = dict(zip(msg.name, msg.position))  # radians

        try:
            theta_v_rad = [joint_map[name] for name in self.joint_order]
        except KeyError as e:
            self.get_logger().warn(f"Missing joint: {e}")
            return

        theta_v_deg = [np.degrees(rad) for rad in theta_v_rad]
        theta_total = [v + o for v, o in zip(theta_v_deg, self.theta_offset)]

        T = np.eye(4)
        for i in range(6):
            T = T @ self.dh_matrix(theta_total[i], self.alpha[i], self.d[i], self.a[i])

        position = T[0:3, 3]
        self.get_logger().info(f"EE Position: x={position[0]:.2f}, y={position[1]:.2f}, z={position[2]:.2f}")

def main(args=None):
    rclpy.init(args=args)
    node = FKNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
