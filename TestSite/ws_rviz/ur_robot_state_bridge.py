#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import JointState
from std_msgs.msg import Header
import socket
import struct
import threading
import time

class URRobotStateBridge(Node):
    def __init__(self):
        super().__init__('ur_robot_state_bridge')
        
        # Parameters
        self.declare_parameter('robot_ip', '192.168.0.100')
        self.declare_parameter('state_port', 30003)
        self.declare_parameter('publish_rate', 50.0)  # Hz
        
        self.robot_ip = self.get_parameter('robot_ip').get_parameter_value().string_value
        self.state_port = self.get_parameter('state_port').get_parameter_value().integer_value
        self.publish_rate = self.get_parameter('publish_rate').get_parameter_value().double_value
        
        # Publishers
        self.joint_state_pub = self.create_publisher(JointState, 'joint_states', 10)
        
        # Joint names for UR3e
        self.joint_names = [
            'shoulder_pan_joint',
            'shoulder_lift_joint', 
            'elbow_joint',
            'wrist_1_joint',
            'wrist_2_joint',
            'wrist_3_joint'
        ]
        
        # Robot connection
        self.robot_socket = None
        self.connected = False
        self.running = True
        
        # Current joint state
        self.current_joint_positions = [0.0] * 6
        self.current_joint_velocities = [0.0] * 6
        
        # Start connection thread
        self.connection_thread = threading.Thread(target=self._connection_loop)
        self.connection_thread.daemon = True
        self.connection_thread.start()
        
        # Start publishing thread
        self.publish_timer = self.create_timer(1.0/self.publish_rate, self._publish_joint_states)
        
        self.get_logger().info(f'UR Robot State Bridge started - connecting to {self.robot_ip}:{self.state_port}')

    def _connect_to_robot(self):
        """Connect to robot real-time interface"""
        try:
            if self.robot_socket:
                self.robot_socket.close()
                
            self.robot_socket = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
            self.robot_socket.settimeout(3.0)  # Shorter timeout
            self.robot_socket.connect((self.robot_ip, self.state_port))
            self.connected = True
            self.get_logger().info('Connected to robot real-time interface')
            return True
            
        except Exception as e:
            self.get_logger().warn(f'Failed to connect to robot: {e}')
            self.connected = False
            return False

    def _read_complete_data(self, expected_size):
        """Read complete data packet with retries"""
        data = b''
        attempts = 0
        max_attempts = 5
        
        while len(data) < expected_size and attempts < max_attempts:
            try:
                remaining = expected_size - len(data)
                chunk = self.robot_socket.recv(remaining)
                
                if not chunk:
                    # Connection closed
                    return None
                    
                data += chunk
                attempts += 1
                
                if len(data) < expected_size:
                    # Wait a bit for more data
                    time.sleep(0.001)
                    
            except socket.timeout:
                attempts += 1
                if attempts >= max_attempts:
                    return None
            except Exception as e:
                self.get_logger().debug(f'Error reading data: {e}')
                return None
                
        return data if len(data) == expected_size else None

    def _read_robot_state(self):
        """Read and parse robot state from TCP connection"""
        try:
            if not self.connected or not self.robot_socket:
                return False
                
            # Read one frame of real-time data (1220 bytes)
            data = self._read_complete_data(1220)
            
            if data is None:
                self.get_logger().debug('Failed to read complete data packet')
                return False
            
            if len(data) != 1220:
                self.get_logger().debug(f'Incomplete data received: {len(data)} bytes, expected 1220')
                return False
            
            # Parse joint positions (actual_q) - starts at byte 252
            joint_positions = []
            for i in range(6):
                start_byte = 252 + i * 8
                joint_bytes = data[start_byte:start_byte + 8]
                if len(joint_bytes) == 8:
                    joint_pos = struct.unpack('>d', joint_bytes)[0]  # Big-endian double
                    joint_positions.append(joint_pos)
                else:
                    return False
            
            # Parse joint velocities (actual_qd) - starts at byte 300  
            joint_velocities = []
            for i in range(6):
                start_byte = 300 + i * 8
                vel_bytes = data[start_byte:start_byte + 8]
                if len(vel_bytes) == 8:
                    joint_vel = struct.unpack('>d', vel_bytes)[0]  # Big-endian double
                    joint_velocities.append(joint_vel)
                else:
                    return False
            
            # Update current state
            self.current_joint_positions = joint_positions
            self.current_joint_velocities = joint_velocities
            
            return True
            
        except socket.timeout:
            self.get_logger().debug('Socket timeout reading robot state')
            return False
        except Exception as e:
            self.get_logger().debug(f'Error reading robot state: {e}')
            self.connected = False
            return False

    def _connection_loop(self):
        """Main connection loop - handles reconnection"""
        while self.running:
            if not self.connected:
                if self._connect_to_robot():
                    continue
                else:
                    time.sleep(2.0)  # Wait before retry
                    continue
            
            # Try to read robot state
            if not self._read_robot_state():
                self.connected = False
                if self.robot_socket:
                    try:
                        self.robot_socket.close()
                    except:
                        pass
                    self.robot_socket = None
                self.get_logger().debug('Lost connection to robot, attempting reconnect...')
                time.sleep(1.0)

    def _publish_joint_states(self):
        """Publish current joint states to ROS"""
        if not self.connected:
            return
            
        # Create JointState message
        joint_state = JointState()
        joint_state.header = Header()
        joint_state.header.stamp = self.get_clock().now().to_msg()
        joint_state.header.frame_id = 'base_link'
        
        joint_state.name = self.joint_names
        joint_state.position = self.current_joint_positions
        joint_state.velocity = self.current_joint_velocities
        joint_state.effort = [0.0] * 6  # We don't have effort data
        
        # Publish the message
        self.joint_state_pub.publish(joint_state)

    def destroy_node(self):
        """Clean shutdown"""
        self.running = False
        self.connected = False
        if self.robot_socket:
            try:
                self.robot_socket.close()
            except:
                pass
        super().destroy_node()

def main(args=None):
    rclpy.init(args=args)
    
    try:
        bridge_node = URRobotStateBridge()
        rclpy.spin(bridge_node)
    except KeyboardInterrupt:
        pass
    finally:
        if 'bridge_node' in locals():
            bridge_node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()