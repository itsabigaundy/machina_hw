import rclpy
import sys

from rclpy.node import Node
from sensor_interfaces.msg import SensorSampleSet
from sensor_interfaces.srv import GetSensorSample


class SensorClient(Node):
    def __init__(self,
        publish_frequency: int) -> None:

        super().__init__('sensor_client_node')

        self.cli1 = self.create_client(GetSensorSample, 'get_sensor_sample_1')
        self.cli2 = self.create_client(GetSensorSample, 'get_sensor_sample_2')
        for i, cli in enumerate((self.cli1, self.cli2)):
            while not cli.wait_for_service(timeout_sec=1.0):
                self.get_logger().info('get_sensor_sample_{} not available, waiting again...'.format(i))
        self.req = GetSensorSample.Request()

        self.pub = self.create_publisher(SensorSampleSet, 'sensor_topic', 10)
        self.rate = self.create_rate(int(publish_frequency))

    def get_sensor_samples(self) -> tuple:
        future1 = self.cli1.call_async(self.req)
        future2 = self.cli2.call_async(self.req)
        rclpy.spin_until_future_complete(self, future1)
        rclpy.spin_until_future_complete(self, future2)

        return future1.result(), future2.result()

    def pub_sensor_samples(self, samples: tuple) -> None:
        msg = SensorSampleSet()
        msg.sample1 = samples[0].sample
        msg.sample2 = samples[1].sample
        self.pub.publish(msg)
        self.get_logger().info('Publishing: {}'.format(samples))


def main(argv=sys.argv):
    rclpy.init()
    sensor_client = SensorClient(argv[1])

    while rclpy.ok():
        sensor_client.pub_sensor_samples(sensor_client.get_sensor_samples())
        sensor_client.rate.sleep()

    sensor_client.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()