import numpy as np
import rclpy
import socket
import sys

from collections import deque
from rclpy.node import Node
from sensor_interfaces.msg import SensorSample
from sensor_interfaces.srv import GetSensorSample


class SensorServer(Node):
    def __init__(self,
        name: str,
        service_name: str,
        address: str,
        port: int,
        sample_frequency: int,
        num_samples: int) -> None:

        super().__init__(name)

        self.sock = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
        self.sensor_address = (address, int(port))
        self.sock.connect(self.sensor_address)

        self.num_samples = int(num_samples)
        self.sensor_req = str(self.num_samples).encode()
        self.samples_buffer = deque(maxlen=5)

        self.ret_len = 18

        timer_period = 1 / int(sample_frequency)
        self.timer = self.create_timer(timer_period, self.sample_sensor)

        self.srv = self.create_service(GetSensorSample, service_name, self.ret_sensor_sample)

    def sample_sensor(self) -> None:
        self.sock.sendall(self.sensor_req)
        byte_data = self.sock.recv(10000)
        data = np.frombuffer(byte_data)
        while data.size:
            newest_samples = self.safe_pop()
            if newest_samples.size < self.ret_len:
                remainder = min(data.size, self.ret_len - newest_samples.size)
                newest_samples = np.concatenate((newest_samples, data[:remainder]))
                self.samples_buffer.append(newest_samples)
                data = data[remainder:]
            else:
                self.samples_buffer.append(newest_samples)
                remainder = min(data.size, self.ret_len)
                self.samples_buffer.append(data[:remainder])
                data = data[remainder:]

    def ret_sensor_sample(self, request, response) -> GetSensorSample.Response:
        def helper() -> np.ndarray:
            ret = self.safe_pop()
            if (not ret.size) or (ret.size == self.ret_len):
                return ret

            prev = self.safe_pop()
            if not prev.size:
                prev = np.empty(self.ret_len)
            else:
                self.samples_buffer.append(prev[:ret.size])
            return np.concatenate((prev[ret.size:], ret))

        self.get_logger().info('Incoming request')
        response.sample = SensorSample()
        response.sample.sample = helper()
        return response

    def safe_pop(self) -> np.ndarray:
        return np.empty(self.ret_len) if not len(self.samples_buffer) else self.samples_buffer.pop()


def main(argv=sys.argv):
    rclpy.init()
    sensor_service = SensorServer(*argv[1:7])
    rclpy.spin(sensor_service)
    sensor_service.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()