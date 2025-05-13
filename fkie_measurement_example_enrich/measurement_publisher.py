import rclpy
from rclpy.node import Node
from rclpy.duration import Duration
from rcl_interfaces.msg import ParameterDescriptor
from builtin_interfaces.msg import Time
from geometry_msgs.msg import PoseStamped
from nav_msgs.msg import Odometry
from std_msgs.msg import Float32
import tf2_ros
from fkie_measurement_msgs.msg import Measurement
from fkie_measurement_msgs.msg import MeasurementLocated
from fkie_measurement_msgs.msg import MeasurementValue


class MeasurementNode(Node):

    current_pose: PoseStamped = None

    def __init__(self):
        super().__init__('measurement_publisher_odom')

        # create parameter descriptors
        self.use_tf = False
        use_tf_descriptor = ParameterDescriptor(description='Get the position from ROS tf')
        self.declare_parameter('use_tf', self.use_tf, use_tf_descriptor)
        map_frame_descriptor = ParameterDescriptor(description='ROS frame_id of the device')
        self.declare_parameter('map_frame', 'map_frame', map_frame_descriptor)

        device_frame_descriptor = ParameterDescriptor(description='ROS frame_id of the device')
        self.declare_parameter('device_frame', 'device_frame', device_frame_descriptor)
        device_serial_id_descriptor = ParameterDescriptor(description='Unique ID that identifies the device ')
        self.declare_parameter('device_serial_id', 'device_serial_id', device_serial_id_descriptor)
        device_name_descriptor = ParameterDescriptor(description='Generic name assigned by the device manufacturer')
        self.declare_parameter('device_name', 'device_name', device_name_descriptor)
        device_classification_descriptor = ParameterDescriptor(description='Generic name assigned by the device manufacturer')
        self.declare_parameter('device_classification', 'R', device_classification_descriptor)

        sensor_name_descriptor = ParameterDescriptor(description='Name or internal ID of the sensor (commonly used when a single device contains multiple sensors)')
        self.declare_parameter('sensor_name', 'sensor_name', sensor_name_descriptor)
        source_type_descriptor = ParameterDescriptor(description='Source of the sensor data, e.g. wind, temperature, rain or other.')
        self.declare_parameter('source_type', 'Dose', source_type_descriptor)
        source_unit_descriptor = ParameterDescriptor(description='Unit of measure: e.g. °C, uSv/h, neutron/s etc...')
        self.declare_parameter('source_unit', 'uSv/h', source_unit_descriptor)

        # get parameter values
        self.use_tf = self.get_parameter('use_tf').value
        self.param_map_frame = self.get_parameter('map_frame').value

        self.param_device_frame = self.get_parameter('device_frame').value
        self.param_device_serial_id = self.get_parameter('device_serial_id').value
        self.param_device_name = self.get_parameter('device_name').value
        self.param_device_classification = self.get_parameter('device_classification').value

        self.param_sensor_name = self.get_parameter('sensor_name').value
        self.param_source_type = self.get_parameter('source_type').value
        self.param_source_unit = self.get_parameter('source_unit').value

        # show parameters in terminal
        self.get_logger().info(f"Parameter 'use_tf': {self.use_tf}")
        self.get_logger().info(f"Parameter 'map_frame': {self.param_map_frame}")
        self.get_logger().info(f"Parameter 'device_frame': {self.param_device_frame}")
        self.get_logger().info(f"Parameter 'device_serial_id': {self.param_device_serial_id}")
        self.get_logger().info(f"Parameter 'device_name': {self.param_device_name}")
        self.get_logger().info(f"Parameter 'device_classification': {self.param_device_classification}")
        self.get_logger().info(f"Parameter 'sensor_name': {self.param_sensor_name}")
        self.get_logger().info(f"Parameter 'source_type': {self.param_source_type}")
        self.get_logger().info(f"Parameter 'source_unit': {self.param_source_unit}")
    
        # create topics
        if self.use_tf:
            self.tf_buffer = tf2_ros.Buffer()
            self.tf_listener = tf2_ros.TransformListener(self.tf_buffer, self)
        else:
            self.sub_odom = self.create_subscription(
                Odometry,
                'odom',
                self.odom_callback,
                10)
        self.sub_measurement = self.create_subscription(
            Float32,
            'sub_measurement',
            self.sub_callback,
            10)
    
        self.pub_measurement_located = self.create_publisher(
            MeasurementLocated,
            '/measurement_located',
            10)
        
        # show topics in terminal
        if not self.use_tf:
            self.get_logger().info(f"Subscribed to topic: {self.sub_odom.topic}[{self.sub_odom.msg_type}]")
        self.get_logger().info(f"Subscribed to topic: {self.sub_measurement.topic}[{self.sub_measurement.msg_type}]")
        self.get_logger().info(f"Publishing to topic: {self.pub_measurement_located.topic}[{self.pub_measurement_located.msg_type}]")
    
    def odom_callback(self, msg: Odometry):
        pose = PoseStamped()
        pose.pose = msg.pose.pose
        pose.header = msg.header
        self.current_pose = pose


    def sub_callback(self, msg: Float32):
        measurement_located = MeasurementLocated()
        now = self.get_clock().now()
        now_msg = now.to_msg()
        if self.use_tf:
            # Look up for the transformation between map_frame and device_frame
            try:
                t = self.tf_buffer.lookup_transform(
                    target_frame=self.param_map_frame,
                    source_frame=self.param_device_frame,
                    time=now,
                    timeout=Duration(seconds=3.))
                pose = PoseStamped()
                pose.header.frame_id = self.param_map_frame
                pose.header.stamp = t.header.stamp
                pose.pose.position.x = t.transform.translation.x
                pose.pose.position.y = t.transform.translation.y
                pose.pose.position.z = t.transform.translation.z
                pose.pose.orientation.x = t.transform.rotation.x
                pose.pose.orientation.y = t.transform.rotation.y
                pose.pose.orientation.z = t.transform.rotation.z
                pose.pose.orientation.w = t.transform.rotation.w
                self.current_pose = pose
            except tf2_ros.TransformException as ex:
                self.get_logger().info(f'Could not transform {self.param_map_frame} to {self.param_device_frame}: {ex}')
                self.current_pose = None
            except tf2_ros.LookupException as ex:
                self.get_logger().info(f'Could not lookup {self.param_map_frame} to {self.param_device_frame}: {ex}')
                self.current_pose = None

        if self.current_pose:
            # we have no GPS position yet, do not set zone values
            # measurement_located.utm_zone_number = 32
            # measurement_located.utm_zone_letter = 'U'
            measurement_located.pose = self.current_pose

        measurement = Measurement()
        # add device description
        measurement.header.stamp = now_msg
        measurement.header.frame_id = self.param_device_frame
        measurement.unique_serial_id = self.param_device_serial_id
        measurement.manufacturer_device_name = self.param_device_name
        measurement.device_classification = self.param_device_classification
        # add measurement value
        measurement_value = MeasurementValue()
        # A certain amount of time is normally required for the measurement.
        # If the measurement time is available, you can set it here. We take the current timestamp.
        measurement_value.begin = now_msg
        measurement_value.end = now_msg
        measurement_value.value_single = msg.data
        # add sensor description to measurement value
        #   Name or internal ID of the sensor (commonly used when a single device contains multiple sensors)
        measurement_value.sensor = self.param_sensor_name
        #   Source of the sensor data, e.g. wind, temperature, rain or other.
        measurement_value.source_type = self.param_source_type 
        #   Unit of the sensor data, e.g. °C or m/s.
        measurement_value.unit = self.param_source_unit
        measurement.values.append(measurement_value)
        measurement_located.measurement = measurement
        self.pub_measurement_located.publish(measurement_located)


def main(args=None):
    print('This is a measurement publisher example.')
    rclpy.init(args=args)

    measurement_node = MeasurementNode()
    rclpy.spin(measurement_node)
    measurement_node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
