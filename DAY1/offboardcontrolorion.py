import rclpy
from rclpy.node import Node
from px4_msgs.msg import OffboardControlMode, TrajectorySetpoint, VehicleCommand, VehicleLocalPosition

class AEDDeliveryNode(Node):
    def __init__(self):
        super().__init__('aed_delivery_node')
        
        self.offboard_ctrl_pub = self.create_publisher(OffboardControlMode, '/fmu/in/offboard_control_mode', 10)
        self.trajectory_pub = self.create_publisher(TrajectorySetpoint, '/fmu/in/trajectory_setpoint', 10)
        self.vehicle_command_pub = self.create_publisher(VehicleCommand, '/fmu/in/vehicle_command', 10)
        self.local_pos_sub = self.create_subscription(VehicleLocalPosition, '/fmu/out/vehicle_local_position', self.position_callback, 10)
        
        self.timer = self.create_timer(0.1, self.timer_callback)
        self.setpoint_counter = 0
        self.current_x = 0.0
        self.current_z = 0.0
        self.mission_phase = "TAKEOFF"

    def position_callback(self, msg):
        self.current_x = msg.x
        self.current_z = msg.z  # In PX4 NED frame, Z is negative as you go UP

    def timer_callback(self):
        self.publish_offboard_control_mode()

        if self.mission_phase == "TAKEOFF":
            # Command straight up to 30 meters (Z = -30.0) with no forward movement
            self.publish_trajectory_setpoint(0.0, 0.0, -30.0)

            if self.setpoint_counter == 10:
                self.publish_vehicle_command(VehicleCommand.VEHICLE_CMD_DO_SET_MODE, 1.0, 6.0)
                self.publish_vehicle_command(VehicleCommand.VEHICLE_CMD_COMPONENT_ARM_DISARM, 1.0)
            self.setpoint_counter += 1

            # Once the drone reaches 28 meters altitude, switch to forward sprint
            if self.current_z <= -28.0:
                self.get_logger().info("Altitude reached. Sprinting 5 km to target...")
                self.mission_phase = "FLYING"

        elif self.mission_phase == "FLYING":
            # Command aggressive forward flight at 30m altitude
            self.publish_trajectory_setpoint(5000.0, 0.0, -30.0)

            # Trigger landing when within 10 meters of the pad
            if self.current_x >= 4990.0:
                self.get_logger().info("5 KM TARGET REACHED! Descending to pad...")
                self.mission_phase = "LANDING"

        elif self.mission_phase == "LANDING":
            # Hold X/Y over the target and command altitude to 0
            self.publish_trajectory_setpoint(5000.0, 0.0, 0.0)

    def publish_offboard_control_mode(self):
        msg = OffboardControlMode()
        msg.position = True
        msg.velocity = False
        msg.acceleration = False
        msg.attitude = False
        msg.body_rate = False
        msg.timestamp = int(self.get_clock().now().nanoseconds / 1000)
        self.offboard_ctrl_pub.publish(msg)

    def publish_trajectory_setpoint(self, x, y, z):
        msg = TrajectorySetpoint()
        msg.position = [x, y, z]
        msg.yaw = 0.0
        msg.timestamp = int(self.get_clock().now().nanoseconds / 1000)
        self.trajectory_pub.publish(msg)

    def publish_vehicle_command(self, command, param1=0.0, param2=0.0):
        msg = VehicleCommand()
        msg.command = command
        msg.param1 = param1
        msg.param2 = param2
        msg.target_system = 1
        msg.target_component = 1
        msg.source_system = 1
        msg.source_component = 1
        msg.from_external = True
        msg.timestamp = int(self.get_clock().now().nanoseconds / 1000)
        self.vehicle_command_pub.publish(msg)

def main(args=None):
    rclpy.init(args=args)
    node = AEDDeliveryNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()