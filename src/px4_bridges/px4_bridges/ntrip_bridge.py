#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy, DurabilityPolicy

from px4_msgs.msg import GpsInjectData
import socket
import base64
from threading import Thread


class NTRIPClient(Node):
    def __init__(self):
        super().__init__('ntrip_gps_inject_node')
        
        # Declare parameters
        self.declare_parameter('ntrip_host', 'rtkbase')
        self.declare_parameter('ntrip_port', 2101)
        self.declare_parameter('ntrip_mountpoint', 'rtkbase')
        self.declare_parameter('ntrip_user', 'rtkbase')
        self.declare_parameter('ntrip_password', 'rtkbase')
        self.declare_parameter('position_lat', 0.0)  # Degrees
        self.declare_parameter('position_lon', 0.0)  # Degrees
        self.declare_parameter('position_height', 0.0)  # Meters
        
        # Get parameters
        self.host = self.get_parameter('ntrip_host').value
        self.port = self.get_parameter('ntrip_port').value
        self.mountpoint = self.get_parameter('ntrip_mountpoint').value
        self.user = self.get_parameter('ntrip_user').value
        self.password = self.get_parameter('ntrip_password').value
        self.lat = self.get_parameter('position_lat').value
        self.lon = self.get_parameter('position_lon').value
        self.height = self.get_parameter('position_height').value
        
        # Configure QoS profile for PX4 (Best Effort, Volatile)
        qos_profile_px4 = QoSProfile(
            reliability=ReliabilityPolicy.BEST_EFFORT,
            durability=DurabilityPolicy.VOLATILE,
            history=HistoryPolicy.KEEP_LAST,
            depth=10
        )

        # Create publisher
        self.publisher = self.create_publisher(
            GpsInjectData,
            '/fmu/in/gps_inject_data',
            qos_profile_px4
        )
        
        # Connection state
        self.socket = None
        self.connected = False
        
        # Start NTRIP client thread
        self.thread = Thread(target=self.ntrip_loop, daemon=True)
        self.thread.start()
        
        self.get_logger().info('NTRIP GPS Inject Node started')
        self.get_logger().info(f'Connecting to {self.host}:{self.port}/{self.mountpoint}')
    
    def connect_ntrip(self):
        """Connect to NTRIP caster"""
        try:
            # Create socket
            self.socket = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
            self.socket.settimeout(10.0)
            
            # Connect to caster
            self.socket.connect((self.host, self.port))
            
            # Build NTRIP request
            request = f"GET /{self.mountpoint} HTTP/1.1\r\n"
            request += f"Host: {self.host}:{self.port}\r\n"
            request += "Ntrip-Version: Ntrip/2.0\r\n"
            request += "User-Agent: NTRIP ROS2Client/1.0\r\n"
            
            # Add authentication if provided
            if self.user and self.password:
                auth_str = f"{self.user}:{self.password}"
                auth_bytes = auth_str.encode('ascii')
                auth_b64 = base64.b64encode(auth_bytes).decode('ascii')
                request += f"Authorization: Basic {auth_b64}\r\n"
            
            # Add NMEA position (GGA format)
            if self.lat != 0.0 or self.lon != 0.0:
                nmea = self.create_gga(self.lat, self.lon, self.height)
                request += f"Ntrip-GGA: {nmea}\r\n"
            
            request += "Connection: close\r\n"
            request += "\r\n"
            
            # Send request
            self.socket.send(request.encode('ascii'))
            
            # Read response
            response = self.socket.recv(1024).decode('ascii', errors='ignore')
            
            if "ICY 200 OK" in response or "HTTP/1.1 200 OK" in response:
                self.connected = True
                self.get_logger().info('Successfully connected to NTRIP caster')
                return True
            else:
                self.get_logger().error(f'NTRIP connection failed: {response}')
                return False
                
        except Exception as e:
            self.get_logger().error(f'Failed to connect to NTRIP caster: {e}')
            return False
    
    def create_gga(self, lat, lon, height):
        """Create NMEA GGA message"""
        # Convert to NMEA format
        lat_deg = int(abs(lat))
        lat_min = (abs(lat) - lat_deg) * 60.0
        lat_dir = 'N' if lat >= 0 else 'S'
        
        lon_deg = int(abs(lon))
        lon_min = (abs(lon) - lon_deg) * 60.0
        lon_dir = 'E' if lon >= 0 else 'W'
        
        # Build GGA string (simplified, without checksum)
        gga = f"$GPGGA,000000.00,{lat_deg:02d}{lat_min:07.4f},{lat_dir},"
        gga += f"{lon_deg:03d}{lon_min:07.4f},{lon_dir},1,12,1.0,{height:.1f},M,0.0,M,,"
        
        # Calculate checksum
        checksum = 0
        for char in gga[1:]:  # Skip the $
            checksum ^= ord(char)
        
        gga += f"*{checksum:02X}"
        return gga
    
    def ntrip_loop(self):
        """Main NTRIP receiving loop"""
        while rclpy.ok():
            if not self.connected:
                if not self.connect_ntrip():
                    self.get_logger().warn('Retrying connection in 5 seconds...')
                    rclpy.sleep(5.0)
                    continue
            
            try:
                # Receive data from NTRIP caster
                data = self.socket.recv(1024)
                
                if not data:
                    self.get_logger().warn('Connection lost, reconnecting...')
                    self.connected = False
                    self.socket.close()
                    continue
                
                # Publish data to PX4
                self.publish_rtcm_data(data)
                
            except socket.timeout:
                continue
            except Exception as e:
                self.get_logger().error(f'Error receiving NTRIP data: {e}')
                self.connected = False
                if self.socket:
                    self.socket.close()
    
    def publish_rtcm_data(self, data):
        """Publish RTCM data to PX4"""
        # Split data into chunks if needed (max 300 bytes per message)
        max_len = 300
        
        for i in range(0, len(data), max_len):
            chunk = data[i:i + max_len]
            
            msg = GpsInjectData()
            msg.timestamp = self.get_clock().now().nanoseconds // 1000  # microseconds
            msg.device_id = 0
            msg.len = len(chunk)
            msg.flags = 0
            
            # Copy data to message
            for j, byte in enumerate(chunk):
                msg.data[j] = byte
            
            self.publisher.publish(msg)
    
    def destroy_node(self):
        """Clean up on shutdown"""
        if self.socket:
            self.socket.close()
        super().destroy_node()


def main(args=None):
    rclpy.init(args=args)
    node = NTRIPClient()
    
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()