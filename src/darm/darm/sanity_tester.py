import rclpy
from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy, DurabilityPolicy
from px4_msgs.msg import VehicleOdometry , ActuatorOutputs
import numpy as np

class SensorInterface:
    def __init__(self, node):
        qos = QoSProfile(
            reliability=ReliabilityPolicy.BEST_EFFORT,
            durability=DurabilityPolicy.TRANSIENT_LOCAL,
            history=HistoryPolicy.KEEP_LAST,
            depth=1
        )
        self.omega_max = 942.478 # Maximum angular velocity in rad/s
        self.node = node
        self.odometry = None
        node.create_subscription(VehicleOdometry, '/fmu/out/vehicle_odometry', self.odom_cb, qos)

        #create interface to recieve actuator commands
        self.actuator_commands = None
        self.actuator_velocity = None
        node.create_subscription(ActuatorOutputs, '/fmu/out/actuator_outputs', self.actuator_cb, qos)
    
    def actuator_cb(self, msg):
        u = np.clip(msg.output[:4], 0.0, 1.0)
        self.actuator_commands = np.array([u[0], u[1], u[2], u[3]])

   
    def get_rotorvel_commands(self):
        actuator = self.actuator_commands  # optional safety
        self.actuator_velocity = np.sqrt(actuator) * self.omega_max
        return self.actuator_velocity
    
    def calc_o_net(self):
        av = self.get_rotorvel_commands()
        o_net = av[0] - av[1] + av[2] -av[3]
        return o_net

    
    def odom_cb(self, msg):
        self.odometry = msg

    def quaternion_to_euler(self, qx, qy, qz, qw):
        """
        Converts quaternion (qx, qy, qz, qw) to Euler angles (roll φ, pitch θ, yaw ψ).
        Returns angles in radians.
        """

        # Roll (φ)
        sinr_cosp = 2 * (qw * qx + qy * qz)
        cosr_cosp = 1 - 2 * (qx ** 2 + qy ** 2)
        roll = np.arctan2(sinr_cosp, cosr_cosp)

        # Pitch (θ)
        sinp = 2 * (qw * qy - qz * qx)
        if abs(sinp) >= 1:
            pitch = np.pi / 2 * np.sign(sinp)  # use 90° if out of domain
        else:
            pitch = np.arcsin(sinp)

        # Yaw (ψ)
        siny_cosp = 2 * (qw * qz + qx * qy)
        cosy_cosp = 1 - 2 * (qy ** 2 + qz ** 2)
        yaw = np.arctan2(siny_cosp, cosy_cosp)

        return np.array([[roll], [pitch], [yaw]])  # φ, θ, ψ  

    def get_state(self):
        return self.odometry

    def get_quaternion(self):
        return self.odometry.q

    def get_euler(self):
        qx, qy, qz, qw = self.odometry.q
        return self.quaternion_to_euler(qx, qy, qz, qw)
    
    def get_position(self):
        return np.array([self.odometry.position[0], 
                         self.odometry.position[1], 
                         self.odometry.position[2]])
    def get_velocity(self):
        return np.array([[self.odometry.velocity[0], 
                         self.odometry.velocity[1],
                         self.odometry.velocity[2]]])
    def get_angular_velocity(self):
        return np.array([self.odometry.angular_velocity[0], 
                         self.odometry.angular_velocity[1], 
                         self.odometry.angular_velocity[2]])