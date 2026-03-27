import rclpy
import math
import time
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy, DurabilityPolicy
from rclpy.qos import qos_profile_sensor_data
from px4_msgs.msg import OffboardControlMode, TrajectorySetpoint, VehicleCommand, VehicleLocalPosition, VehicleStatus
import os, pty, select, re, sys
from geometry_msgs.msg import PoseStamped

class OffboardShipLandNode(Node):
    """Node for controlling a vehicle in offboard mode."""

    def __init__(self) -> None:
        super().__init__('offboard_ship_land_node')

        self.get_logger().info("Offboard Ship land Node Alive!")

        # Configure QoS profile for publishing and subscribing
        qos_profile = QoSProfile(
            reliability=ReliabilityPolicy.BEST_EFFORT,
            durability=DurabilityPolicy.TRANSIENT_LOCAL,
            history=HistoryPolicy.KEEP_LAST,
            depth=1
        )

        # Create publishers
        self.offboard_control_mode_publisher = self.create_publisher(
            OffboardControlMode, '/fmu/in/offboard_control_mode', qos_profile)
        self.trajectory_setpoint_publisher = self.create_publisher(
            TrajectorySetpoint, '/fmu/in/trajectory_setpoint', qos_profile)
        self.vehicle_command_publisher = self.create_publisher(
            VehicleCommand, '/fmu/in/vehicle_command', qos_profile)
        self.vehicle_status_subscriber = self.create_subscription(
            VehicleStatus, '/fmu/out/vehicle_status', self.vehicle_status_callback, qos_profile)
        
        self.tag_subscriber = self.create_subscription(
            PoseStamped,
            '/tag_detections',        
            self.tag_pose_callback,
            qos_profile_sensor_data
        )

        # self.vehicle_local_position_subscriber = self.create_subscription(
            # PoseStamped, '/qvio', self.vehicle_local_position_callback, qos_profile_sensor_data)
        

        self.vehicle_local_position_subscriber = self.create_subscription(
            VehicleLocalPosition, '/fmu/out/vehicle_local_position',
            self.vehicle_local_position_callback, qos_profile_sensor_data)


        self.rate = 20
        self.duration = 5
        self.altitude = -1.0
        self.steps = self.duration * self.rate
        self.path = []
        self.vehicle_local_position = None
        self.vehicle_status = VehicleStatus()
        self.taken_off = False
        self.hit_path = False
        self.armed = False

        self.land_start_time = None
        self.hover_cords = None

        self.offboard_setpoint_counter = 0
        self.start_time = time.time()
        self.offboard_arr_counter = 0
        self.tag_pose = None


        self.timer = self.create_timer(0.1, self.timer_callback)

    def _shutdown(self):
        self.get_logger().info("Shutting down node.")
        self.destroy_node()
        rclpy.shutdown()

    def timer_callback(self) -> None:
        """Callback function for the timer."""
        self.publish_offboard_control_heartbeat_signal_position()

        if self.land_start_time and self.land_start_time + 5.0 < time.time():
            print("Quitting program")
            self.get_clock().call_later(0.1, self._shutdown)


        if self.offboard_setpoint_counter == 10:
           self.engage_offboard_mode()
           self.arm()
           self.armed = True

        if self.offboard_setpoint_counter < 11:
            self.offboard_setpoint_counter += 1

        if (self.start_time + 15 > time.time() and self.start_time + 10 < time.time()):
            self.publish_takeoff_setpoint(0.0, 0.0, self.altitude)
        elif self.start_time + 15 < time.time():
            if(not self.hit_path):
                print("Doing tag alignment now")             
                self.tag_align_timer = self.create_timer(1 / self.rate, self.offboard_move_callback)
                self.hit_path = True

    def tag_pose_callback(self, msg):
        # Extract position and orientation
        x = msg.pose.position.x
        y = msg.pose.position.y
        z = msg.pose.position.z
        self.tag_pose = (x, y, z)

    # def vehicle_local_position_callback(self, msg):
    #     """Callback function for vehicle_local_position topic subscriber."""
    #     x = msg.pose.position.x
    #     y = msg.pose.position.y
    #     z = msg.pose.position.z
    #     self.vehicle_local_position = (x, y, z)

    def vehicle_local_position_callback(self, msg: VehicleLocalPosition):
        # PX4 NED: x=North, y=East, z=Down
        self.vehicle_local_position = (msg.x, msg.y, msg.z)
        # print(f"Vehicle Local Position: x={msg.x:.2f} m, y={msg.y:.2f} m, z={msg.z:.2f} m")
    
    def vehicle_status_callback(self, vehicle_status):
        """Callback function for vehicle_status topic subscriber."""
        self.vehicle_status = vehicle_status

    def arm(self):
        """Send an arm command to the vehicle."""
        self.publish_vehicle_command(
            VehicleCommand.VEHICLE_CMD_COMPONENT_ARM_DISARM, param1=1.0)
        self.get_logger().info('Arm command sent')

    def disarm(self):
        """Send a disarm command to the vehicle."""
        self.publish_vehicle_command(
            VehicleCommand.VEHICLE_CMD_COMPONENT_ARM_DISARM, param1=0.0)
        self.get_logger().info('Disarm command sent')

    def engage_offboard_mode(self):
        """Switch to offboard mode."""
        self.publish_vehicle_command(
            VehicleCommand.VEHICLE_CMD_DO_SET_MODE, param1=1.0, param2=6.0)
        self.get_logger().info("Switching to offboard mode")

    def land(self):
        """Switch to land mode."""
        self.publish_vehicle_command(VehicleCommand.VEHICLE_CMD_NAV_LAND)
        self.get_logger().info("Switching to land mode")
        self.taken_off = False
        #self.hit_figure_8 = False

    def offboard_move_callback(self):
        """Callback function for offboard movement along the path."""
        if self.tag_pose is not None:
            x, y, z = self.tag_pose
            horizontal_align = x < 0.3 and x > -0.3
            ready_to_land = horizontal_align and z < 2.0

            if not self.land_start_time:

                if ready_to_land:
                    print("Ready to land, Hovering in Place!")
                    self.hover_cords = self.vehicle_local_position
                    self.land_start_time = time.time()
                    self.altitude = -0.04
                elif horizontal_align:
                    print(f"Horizontally Aligned, move forward")
                    self.publish_move_forward_setpoint(x)
                elif not horizontal_align and  x>=0.2: 
                    print(f"Not yet horizontall aligned, move right")
                    self.publish_move_right_setpoint()
                elif not horizontal_align and  x<=-0.2:
                    print(f"Not yet horizontall aligned, move left")
                    self.publish_move_left_setpoint()
            else: 
                if self.land_start_time + 1.5 < time.time():
                    print("Actually landing now")
                    self.tag_align_timer.cancel()
                else: 
                    print(f"Hover cords: {self.hover_cords}")
                    self.publish_current_hover_setpoint(self.hover_cords[0], self.hover_cords[1])

    def publish_takeoff_setpoint(self, x: float, y: float, z: float):
        """Publish the trajectory setpoint."""
        msg = TrajectorySetpoint()
        msg.position = [x, y, z]
        # msg.yaw = (45.0) * math.pi / 180.0;
        msg.yaw = 0.0
        msg.timestamp = int(self.get_clock().now().nanoseconds / 1000)
        self.trajectory_setpoint_publisher.publish(msg)

    def publish_move_left_setpoint(self): 
        curr_x, curr_y, curr_z = self.vehicle_local_position
        msg = TrajectorySetpoint()
        msg.position = [curr_x, curr_y - 4.0 / self.rate, self.altitude]
        # print(msg.position)
        msg.yaw = 0.0
        msg.timestamp = int(self.get_clock().now().nanoseconds / 1000)
        self.trajectory_setpoint_publisher.publish(msg)

    def publish_move_right_setpoint(self): 
        curr_x, curr_y, curr_z = self.vehicle_local_position
        msg = TrajectorySetpoint()
        msg.position = [curr_x, curr_y + 4.0 / self.rate, self.altitude]
        # print(msg.position)
        msg.yaw = 0.0
        msg.timestamp = int(self.get_clock().now().nanoseconds / 1000)
        self.trajectory_setpoint_publisher.publish(msg)

    def publish_move_forward_setpoint(self, tag_displacement):
        if tag_displacement > 0: 
            adjustment = min(5.0 * tag_displacement, 2.0)
        else: 
            adjustment = max(5.0 * tag_displacement, -2.0)
        curr_x, curr_y, curr_z = self.vehicle_local_position
        msg = TrajectorySetpoint()
        msg.position = [curr_x + 6.0 / self.rate, curr_y + adjustment / self.rate, self.altitude]
        print(msg.position)
        msg.yaw = 0.0
        msg.timestamp = int(self.get_clock().now().nanoseconds / 1000)
        self.trajectory_setpoint_publisher.publish(msg)
    
    def publish_current_hover_setpoint(self, x, y): 
        msg = TrajectorySetpoint()
        msg.position = [x, y, self.altitude]
        print(msg.position)
        msg.yaw = 0.0
        msg.timestamp = int(self.get_clock().now().nanoseconds / 1000)
        self.trajectory_setpoint_publisher.publish(msg)

    def publish_offboard_control_heartbeat_signal_position(self):
        """Publish the offboard control mode."""
        msg = OffboardControlMode()
        msg.position = True
        msg.velocity = False
        msg.acceleration = False
        msg.attitude = False
        msg.body_rate = False
        msg.timestamp = int(self.get_clock().now().nanoseconds / 1000)
        self.offboard_control_mode_publisher.publish(msg)

    def publish_offboard_control_heartbeat_signal_velocity(self):
        """Publish the offboard control mode."""
        msg = OffboardControlMode()
        msg.position = False
        msg.velocity = True
        msg.acceleration = False
        msg.attitude = False
        msg.body_rate = False
        msg.timestamp = int(self.get_clock().now().nanoseconds / 1000)
        self.offboard_control_mode_publisher.publish(msg)

    def publish_vehicle_command(self, command, **params) -> None:
        """Publish a vehicle command."""
        msg = VehicleCommand()
        msg.command = command
        msg.param1 = params.get("param1", 0.0)
        msg.param2 = params.get("param2", 0.0)
        msg.param3 = params.get("param3", 0.0)
        msg.param4 = params.get("param4", 0.0)
        msg.param5 = params.get("param5", 0.0)
        msg.param6 = params.get("param6", 0.0)
        msg.param7 = params.get("param7", 0.0)
        msg.target_system = 1
        msg.target_component = 1
        msg.source_system = 1
        msg.source_component = 1
        msg.from_external = True
        msg.timestamp = int(self.get_clock().now().nanoseconds / 1000)
        self.vehicle_command_publisher.publish(msg)

def main(args=None) -> None:
    rclpy.init(args=args)
    offboard_ship_land_node = OffboardShipLandNode()
    rclpy.spin(offboard_ship_land_node)
    offboard_ship_land_node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    try:
        main()
    except Exception as e:
        print(e)
