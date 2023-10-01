import sys
import threading
import math
from bno055.connectors.i2c import I2C
from bno055.connectors.uart import UART
from bno055.error_handling.exceptions import BusOverRunException
from bno055.params.NodeParameters import NodeParameters
from bno055.sensor.SensorService import SensorService
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Imu
from geometry_msgs.msg import Vector3

class Bno055Node(Node):
    sensor = None
    param = None
    # v0 goes in the order of x y z starting from index 0 

    def __init__(self):
        super().__init__('bno055')

        # Create a subscriber for the IMU topic
        self.sub_imu = self.create_subscription(
            Imu,
            '/bno055/imu',  # Change to the actual IMU topic
            self.imu_callback,
            10  # Adjust the queue size as needed
        )
        self.v = Vector3()
        self.v.x = 0
        self.v.y = 0
        self.v.z = 0
        self.prev_time = None
        self.curr_time = None
        self.was_flipped = False
        self.unflip = False

    def setup(self):
        self.param = NodeParameters(self)

        if self.param.connection_type.value == UART.CONNECTIONTYPE_UART:
            connector = UART(self,
                             self.param.uart_baudrate.value,
                             self.param.uart_port.value,
                             self.param.uart_timeout.value)
        elif self.param.connection_type.value == I2C.CONNECTIONTYPE_I2C:
            connector = I2C(self,
                            self.param.i2c_bus.value,
                            self.param.i2c_addr.value)
        else:
            raise NotImplementedError('Unsupported connection type: ' + str(self.param.connection_type.value))

        # Connect to BNO055 device:

        connector.connect()
        self.sensor = SensorService(self, connector, self.param)
        self.sensor.configure()

    def imu_callback(self, msg):
        # Log current velocity (not acceleration) and orientation
        linear_acceleration = msg.linear_acceleration
        linear_velocity = self.calculate_linear_velocity(msg.linear_acceleration)

        self.get_logger().info('Linear Velocity (meters/sec): {}'.format(linear_velocity))

        # Log when the IMU flips over or returns to the upright position
        if self.orientation_flipped(msg.linear_acceleration):
            self.get_logger().info('IMU flipped over')
        if self.unflip:
            self.get_logger().info('IMU unflipped')

        # Log a warning for sudden rapid acceleration (e.g., collision detection)
        if self.detect_rapid_acceleration(msg.linear_acceleration):
            self.get_logger().warn('Sudden rapid acceleration detected! Maybe crashed into something...')


    def calculate_linear_velocity(self, linear_acceleration):
        if self.prev_time is None:
            self.prev_time = rclpy.clock.Clock().now()  # Initialize prev_time

        if self.curr_time is None:
            self.curr_time = rclpy.clock.Clock().now()  # Initialize curr_time

        # Calculate time interval (in seconds) since the last callback
        delta_time = (self.curr_time - self.prev_time).nanoseconds / 1e9

        # Calculate linear velocity using Euler method (assuming constant acceleration)
        self.v.x += linear_acceleration.x * delta_time
        self.v.y += linear_acceleration.y * delta_time
        self.v.z += linear_acceleration.z * delta_time

        # Update prev_time and curr_time for the next callback
        self.prev_time = self.curr_time
        self.curr_time = rclpy.clock.Clock().now()

        # Calculate the magnitude of linear velocity
        return math.sqrt(self.v.x ** 2 + self.v.y ** 2 + self.v.z ** 2)


    def orientation_flipped(self, linear_acceleration):
        # Check if the orientation has flipped over or returned to the upright position
        # Jason suggestion: just check if lin_accel.z is negative or not
        # if it's flipped that should be reading positive.

        # case 1: flipped
        if linear_acceleration.z > 0:
            if self.was_flipped == False:
                self.was_flipped = True
                self.unflip = False
            return True
        # case 2: not flipped
        elif linear_acceleration.z < 0:
            if self.was_flipped:
                self.unflip = True
                self.was_flipped = False
            return False
        return False

    def detect_rapid_acceleration(self, linear_acceleration):
        # threshold arbitrary, test & change constant
        threshold = 9.8 * 0.5
        acceleration_magnitude = math.sqrt(
            linear_acceleration.x ** 2 + linear_acceleration.y ** 2 + linear_acceleration.z ** 2
        )
        return acceleration_magnitude > threshold
            

def main(args=None):
    try:
        rclpy.init()
        node = Bno055Node()
        node.setup()

        lock = threading.Lock()

        def read_data():
            if lock.locked():
                node.get_logger().warn('Message communication in progress - skipping query cycle')
                return

            lock.acquire()
            try:
                node.sensor.get_sensor_data()
            except BusOverRunException:
                return
            except ZeroDivisionError:
                return
            except Exception as e:
                node.get_logger().warn('Receiving sensor data failed with %s:"%s"' % (type(e).__name__, e))
            finally:
                lock.release()

        def log_calibration_status():
            if lock.locked():
                node.get_logger().warn('Message communication in progress - skipping query cycle')
                return

            lock.acquire()
            try:
                node.sensor.get_calib_status()
            except Exception as e:
                node.get_logger().warn('Receiving calibration status failed with %s:"%s"' % (type(e).__name__, e))
            finally:
                lock.release()

        f = 1.0 / float(node.param.data_query_frequency.value)
        data_query_timer = node.create_timer(f, read_data)

        f = 1.0 / float(node.param.calib_status_frequency.value)
        status_timer = node.create_timer(f, log_calibration_status)

        rclpy.spin(node)

    except KeyboardInterrupt:
        node.get_logger().info('Ctrl+C received - exiting...')
        sys.exit(0)
    finally:
        node.get_logger().info('ROS node shutdown')
        try:
            node.destroy_timer(data_query_timer)
            node.destroy_timer(status_timer)
            # node.destroy_timer()
        except UnboundLocalError:
            node.get_logger().info('No timers to shutdown')
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
