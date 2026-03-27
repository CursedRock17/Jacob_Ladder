# ROS 2 Core Libraries
import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy, DurabilityPolicy
from rclpy.qos import qos_profile_sensor_data

# ROS 2 msgs
from px4_msgs.msg import OffboardControlMode, TrajectorySetpoint, VehicleCommand, VehicleLocalPosition, VehicleStatus
from geometry_msgs.msg import PoseStamped
from nav_msgs.msg import Path

# Core Stuff
import time
import math


class DroneSmoothPlannerNode(Node):
    """Node for controlling a vehicle in offboard mode."""

    def __init__(self) -> None:
        super().__init__('drone_smooth_planner_node')
        self.get_logger().info("Offboard Ship land Node Alive!")

        # Useful Naming Parameters
        self.namespace = self.declare_parameter('namespace', "").value

        # Trajectory Parameters - Using parameters means we can adjust in CLI
        self.num_waypoints = self.declare_parameter('num_waypoints', 10).value
        self.setpoint_rate_hz = self.declare_parameter('setpoint_rate_hz', 20).value  # Hertz
        self.s_curve_steepness = self.declare_parameter('s_curve_steepness', 4.0).value
        self.waypoint_tolerance_m = self.declare_parameter('waypoint_tolerance_m', 0.15).value  # Meters
        self.max_velocity = self.declare_parameter('max_velocity', 0.5).value  # Meters/sec
        self.max_acceleration = self.declare_parameter('max_acceleration', 0.35).value  # Meters/sec^2

        # Configure QoS profile for publishing and subscribing
        qos_profile = QoSProfile(
            reliability=ReliabilityPolicy.BEST_EFFORT,
            durability=DurabilityPolicy.TRANSIENT_LOCAL,
            history=HistoryPolicy.KEEP_LAST,
            depth=1
        )

        # Create Input Publishers
        self.offboard_control_mode_publisher = self.create_publisher(
            OffboardControlMode, '/fmu/in/offboard_control_mode', qos_profile)
        self.trajectory_setpoint_publisher = self.create_publisher(
            TrajectorySetpoint, '/fmu/in/trajectory_setpoint', qos_profile)
        self.vehicle_command_publisher = self.create_publisher(
            VehicleCommand, '/fmu/in/vehicle_command', qos_profile)

        # Create Output Publishers
        self.vehicle_status_subscriber = self.create_subscription(
            VehicleStatus, '/fmu/out/vehicle_status', self.vehicle_status_callback, qos_profile)
        self.vehicle_local_position_subscriber = self.create_subscription(
            VehicleLocalPosition, '/fmu/out/vehicle_local_position',
            self.vehicle_local_position_callback, qos_profile_sensor_data)

        # Input from onboard sensors - i.e Camera, Optical Flow
        self.drogue_pose_subscriber = self.create_subscription(
            PoseStamped,
            self.namespace + '/tag_detections',
            self.drogue_pose_callback,
            qos_profile_sensor_data
        )

        # Visualization Publishers
        self.path_viz_pub = self.create_publisher(
            Path, self.namespace + '/trajectory_path_viz', qos_profile)
        self.current_pose_viz_pub = self.create_publisher(
            Path, self.namespace + '/current_pose_viz', qos_profile)

        # self.vehicle_local_position_subscriber = self.create_subscription(
            # PoseStamped, '/qvio', self.vehicle_local_position_callback, qos_profile_sensor_data)

        # S-Curve Planner Information
        self.waypoints = []
        self.current_waypoint_idx = 0
        self.replan_on_waypoint_reached = True
        self.trajectory_active = False

        # Vehicle State Information
        self.altitude = -1.0
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
        self.drogue_pose = None

        # Constantly running process timer
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
            self.timer.cancel()
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
            if not self.hit_path:
                print("Doing drogue alignment now")
                self.drogue_align_timer = self.create_timer(1 / self.setpoint_rate_hz, self.offboard_move_callback)
                self.hit_path = True

    # ------------ Essential Drone Functions --------------- #
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

    def vehicle_local_position_callback(self, msg: VehicleLocalPosition):
        """Callback function for vehicle_local_position topic subscriber."""
        # PX4 NED: x=North, y=East, z=Down
        self.vehicle_local_position = (msg.x, msg.y, msg.z)

        # Turn to Pose for visualization
        pose_msg = PoseStamped()
        pose_msg.header.stamp = self.get_clock().now().to_msg()
        # Relate position back to out map (Path of waypoints)
        pose_msg.header.frame_id = "map"

        # Grab the same position
        pose_msg.pose.position.x = msg.x
        pose_msg.pose.position.y = msg.y
        pose_msg.pose.position.z = msg.z
        # Just use a basic orientation
        pose_msg.pose.orientation.w = 1.0

        self.current_pose_viz_pub.publish(pose_msg)
        # print(f"Vehicle Local Position: x={msg.x:.2f} m, y={msg.y:.2f} m, z={msg.z:.2f} m")

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

    # ------------ Essential Drone Functions --------------- #

    def drogue_pose_callback(self, pose_msg: PoseStamped):
        """
        Receive new drogue position and orientation and generate trajectory.
        This triggers new trajectory creation.
        :params:
            pose_msg: PoseStamped ROS 2 MSG of the estimated position of the
            drouge relative to the drone.
        :returns:
            None
        """
        # Grab our new drogue pose
        x = pose_msg.pose.position.x
        y = pose_msg.pose.position.y
        z = pose_msg.pose.position.z
        new_drogue_pose = (x, y, z)
        self.drogue_pose = new_drogue_pose

    def generate_s_curve_waypoints(self, start_pos, end_pos, num_points):
        """
        Generate waypoints along an S-curve from starting position to
        the target position (drogue).

        :params:
            start_pos: Starting position (x, y, z).
            end_pos: Target position (x, y, z).
            num_points: Number of waypoints.
        :returns:
            waypoints: Array of waypoint tuples [# of points, 3].
        """
        waypoints = []

        for i in range(num_points):
            # Create a smooth, normalized range.
            t = i / (num_points - 1)
            # S-curve using tanh
            s = (math.tanh(self.s_curve_steepness * (t - 0.5)) + 1) / 2

            # Linear interpolation with S-curve timing
            x = start_pos[0] + s * (end_pos[0] - start_pos[0])
            y = start_pos[1] + s * (end_pos[1] - start_pos[1])
            z = start_pos[2] + s * (end_pos[2] - start_pos[2])

            waypoints.append((x, y, z))

        return waypoints

    def compute_distance(self, start_pos, end_pos):
        """
        Helper Function to compute distance between two positions.

        :params:
            start_pos: Starting Position (x, y, z)
            end_pos: Ending Position (x, y, z)
        :returns:
            true_distance: The correct Euclidean distance between the start/end positions.
        """
        dx = end_pos[0] - start_pos[0]
        dy = end_pos[1] - start_pos[1]
        dz = end_pos[2] - start_pos[2]
        true_distance = math.sqrt(dx*dx + dy*dy + dz*dz)
        return true_distance

    def compute_direction(self, start_pos, end_pos):
        """Helper Function to compute unit direction vector from start to end position.

        :params:
            start_pos: Starting Position (x, y, z)
            end_pos: Ending Position (x, y, z)
        :returns:
            true_direction: The correct Euclidean direction between the start/end positions.
        """
        dx = end_pos[0] - start_pos[0]
        dy = end_pos[1] - start_pos[1]
        dz = end_pos[2] - start_pos[2]

        magnitude = math.sqrt(dx*dx + dy*dy + dz*dz)
        if magnitude < 1e-6:
            return (0.0, 0.0, 0.0)

        return (dx/magnitude, dy/magnitude, dz/magnitude)

    def generate_s_curve_velocity_profile(self, total_distance, num_samples, dt):
        """
        Generate true S-curve velocity profile.
        Returning a smooth acceleration -> cruise -> deceleration.

        :params:
            total_distance: The total length of travel of the path (in meters).
            num_samples: How smooth/rigid the profile will be.
            dt: Change in time (in seconds).
        :returns:
            v_profile: Velocity profile of movement relative to the position curve.
            a_profile: Acceleration profile of movement relative to the position curve.
            j_profile: Jerk profile of movement relative to the position curve.
        """
        # Create an array for each of the profiles corresponding to each of the trajectories
        v_profile = []
        a_profile = []
        j_profile = []

        # Get the total time
        T = (num_samples - 1) * dt

        # For essentially each number of waypoints calculate the various velocity, acceleration, 
        # and jerk profiles which are just derivatives of the previous.
        for i in range(num_samples):
            ti = i * dt
            tau = ti / T if T > 0 else 0

            # Sigmoid S-curve
            exp_term = math.exp(-10.0 * (tau - 0.5))
            s = 1.0 / (1.0 + exp_term)

            # Derivatives
            v = (10.0 / T) * s * (1.0 - s) if T > 0 else 0
            a = (100.0 / (T * T)) * s * (1.0 - s) * (1.0 - 2.0 * s) if T > 0 else 0
            j = (1000.0 / (T * T * T)) * s * (1.0 - s) * (1.0 - 6.0 * s * (1.0 - s)) if T > 0 else 0

            v_profile.append(v)
            a_profile.append(a)
            j_profile.append(j)

        # Scale to actual distance using trapezoidal integration
        current_distance = 0.0
        for i in range(len(v_profile) - 1):
            current_distance += (v_profile[i] + v_profile[i+1]) / 2.0 * dt

        if current_distance > 0:
            scale = total_distance / current_distance
            v_profile = [v * scale for v in v_profile]
            a_profile = [a * scale for a in a_profile]
            j_profile = [j * scale for j in j_profile]

        # Clamp to limits
        v_profile = [min(max(v, 0), self.max_velocity) for v in v_profile]
        a_profile = [min(max(a, -self.max_acceleration), self.max_acceleration) for a in a_profile]

        return v_profile, a_profile, j_profile

    def generate_trajectory_to_drogue(self):
        """
        Generate complete trajectory from current position to drogue.
        Stores result in self.path array which is grouping of Setpoint Trajectories.

        :params:
            None
        :returns:
            None
        """
        if not self.vehicle_local_position or not self.drogue_pose:
            self.get_logger().warn("Missing position data for trajectory generation")
            return

        # Current position (NED frame)
        start_pos = self.vehicle_local_position

        # Drogue position (relative, need to convert to world frame)
        end_pos = (
               start_pos[0] + self.drogue_pose[0],
               start_pos[1] + self.drogue_pose[1],
               start_pos[2] + self.drogue_pose[2]
           )

        # Generate waypoints
        waypoints = self.generate_s_curve_waypoints(start_pos, end_pos, self.num_waypoints)

        # Compute total distance
        total_distance = 0.0
        for i in range(len(waypoints) - 1):
            total_distance += self.compute_distance(waypoints[i], waypoints[i+1])

        # Generate velocity profile
        dt = 1.0 / self.setpoint_rate_hz
        num_samples = len(waypoints)
        v_profile, a_profile, j_profile = self.generate_s_curve_velocity_profile(
            total_distance, num_samples, dt)

        # Build path array of TrajectorySetpoint messages
        self.path = []
        for i in range(len(waypoints)):
            msg = TrajectorySetpoint()

            # Position
            msg.position = [waypoints[i][0], waypoints[i][1], waypoints[i][2]]

            # Velocity (along path direction)
            if i < len(waypoints) - 1:
                direction = self.compute_direction(waypoints[i], waypoints[i+1])
            else:
                direction = self.compute_direction(waypoints[i-1], waypoints[i])

            msg.velocity = [
                v_profile[i] * direction[0],
                v_profile[i] * direction[1],
                v_profile[i] * direction[2]
            ]

            # Acceleration
            msg.acceleration = [
                a_profile[i] * direction[0],
                a_profile[i] * direction[1],
                a_profile[i] * direction[2]
            ]

            # Jerk
            msg.jerk = [
                j_profile[i] * direction[0],
                j_profile[i] * direction[1],
                j_profile[i] * direction[2]
            ]

            # Yaw toward drogue
            dx = end_pos[0] - waypoints[i][0]
            dy = end_pos[1] - waypoints[i][1]
            msg.yaw = math.atan2(dy, dx)

            self.path.append(msg)

        # Compute yawspeed
        for i in range(len(self.path)):
            next_yaw = self.path[(i + 1) % len(self.path)].yaw
            curr_yaw = self.path[i].yaw

            if next_yaw - curr_yaw < -math.pi:
                next_yaw += 2.0 * math.pi
            if next_yaw - curr_yaw > math.pi:
                next_yaw -= 2.0 * math.pi

            self.path[i].yawspeed = (next_yaw - curr_yaw) / dt

        # Reset tracking
        self.offboard_arr_counter = 0
        self.trajectory_active = True

        print(f"Generated S-curve trajectory: {len(self.path)} points, "
              f"distance: {total_distance:.2f}m")

        # Create Visualization MSG of a Path
        self.publish_path_visualization()

    def offboard_move_callback(self):
        """
        Callback function for offboard movement along the S-Curve, essentially
        passing through waypoints along the curve.
        :params:
            None
        :returns:
            None
        """
        # Generate trajectory on first call
        if not hasattr(self, 'path') or len(self.path) == 0:
            self.generate_trajectory_to_drogue()
            if not hasattr(self, 'path') or len(self.path) == 0:
                return

        # Check if we should regenerate (drogue moved significantly)
        if self.drogue_pose and self.vehicle_local_position and self.offboard_arr_counter > 0:
            start_pos = self.vehicle_local_position
            drogue_relative = self.drogue_pose
            current_target = start_pos + drogue_relative

            # Current target in world frame
            current_target = (
                start_pos[0] + drogue_relative[0],
                start_pos[1] + drogue_relative[1],
                start_pos[2] + drogue_relative[2]
            )

            # If drogue moved > 0.10m, regenerate
            if len(self.path) > 0:
                last_target = tuple(self.path[-1].position)
                movement = self.compute_distance(last_target, current_target)
                if movement > 0.10:
                    print(f"Drogue moved {movement:.2f}m, regenerating trajectory")
                    self.generate_trajectory_to_drogue()

        # Follow the waypoints, if any
        if self.offboard_arr_counter < len(self.path):
            # Get the current desired trajectory of the next waypoint & publish
            msg = self.path[self.offboard_arr_counter]
            msg.timestamp = int(self.get_clock().now().nanoseconds / 1000)
            self.trajectory_setpoint_publisher.publish(msg)

            # Check if close to current setpoint to advance
            if self.vehicle_local_position:
                target = tuple(msg.position)
                distance = self.compute_distance(self.vehicle_local_position, target)

                # If we've made it to the next waypoint to a level of tolerance, move on.
                if distance < self.waypoint_tolerance_m or self.offboard_arr_counter == 0:
                    self.offboard_arr_counter += 1

        else:
            # Reached end - check if close enough to drogue
            if self.vehicle_local_position and self.drogue_pose:
                drogue_distance = math.sqrt(
                    self.drogue_pose[0]**2 +
                    self.drogue_pose[1]**2 +
                    self.drogue_pose[2]**2
                )

                if drogue_distance < self.waypoint_tolerance_m:  # Within our tolerance
                    if not self.land_start_time:
                        print("Ready to land, Hovering in Place!")
                        self.hover_cords = self.vehicle_local_position
                        self.land_start_time = time.time()
                        self.altitude = -0.04
                    else:
                        if self.land_start_time + 1.5 < time.time():
                            print("Actually landing now")
                            self.drogue_align_timer.cancel()
                        else:
                            print(f"Hover cords: {self.hover_cords}")
                            self.publish_current_hover_setpoint(self.hover_cords[0], self.hover_cords[1])
                else:
                    # Not close enough, regenerate trajectory
                    self.generate_trajectory_to_drogue()

    def publish_path_visualization(self):
        """
        Publish the current trajectory path for visualization in Foxglove.
        Meant to model the various waypoints as a vector of PoseStamped msgs.

        :params:
            None
        :returns:
            None
        """
        if not hasattr(self, 'path') or len(self.path) == 0:
            return

        # Create a Path msg
        path_msg = Path()
        path_msg.header.stamp = self.get_clock().now().to_msg()
        # TODO: Check PX4 to see if it prefers "odom"
        path_msg.header.frame_id = "map"

        # Create an estimated pose from each of the TrajectorySetpoints
        for trajectory_point in self.path:
            pose_stamped = PoseStamped()
            pose_stamped.header = path_msg.header
            # Copy the position
            pose_stamped.pose.position.x = trajectory_point.position[0]
            pose_stamped.pose.position.y = trajectory_point.position[1]
            pose_stamped.pose.position.z = trajectory_point.position[2]

            # Copy the orientation from yaw
            pose_stamped.pose.orientation.x = 0.0
            pose_stamped.pose.orientation.y = 0.0
            pose_stamped.pose.orientation.z = math.sin(trajectory_point.yaw / 2.0)
            pose_stamped.pose.orientation.w = math.cos(trajectory_point.yaw / 2.0)

            path_msg.poses.append(pose_stamped)

        self.path_viz_pub.publish(path_msg)
        print(f"Published path, has {len(path_msg.poses)} poses")

    def publish_current_hover_setpoint(self, x, y):
        """Publish the trajectory setpoint for a hover state."""
        msg = TrajectorySetpoint()
        msg.position = [x, y, self.altitude]
        print(msg.position)
        msg.yaw = 0.0
        msg.timestamp = int(self.get_clock().now().nanoseconds / 1000)
        self.trajectory_setpoint_publisher.publish(msg)

    def publish_takeoff_setpoint(self, x: float, y: float, z: float):
        """Publish the trajectory setpoint for a takeoff state."""
        msg = TrajectorySetpoint()
        msg.position = [x, y, z]
        # msg.yaw = (45.0) * math.pi / 180.0;
        msg.yaw = 0.0
        msg.timestamp = int(self.get_clock().now().nanoseconds / 1000)
        self.trajectory_setpoint_publisher.publish(msg)


def main(args=None) -> None:
    rclpy.init(args=args)
    drone_smooth_planner_node = DroneSmoothPlannerNode()
    rclpy.spin(drone_smooth_planner_node)
    drone_smooth_planner_node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    try:
        main()
    except Exception as e:
        print(e)

