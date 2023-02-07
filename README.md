# Robotic_HW
 
## Objective
The goal of This project is to build a ROS2 network that collects data from 3-DOF sensors and makes the filtered data available as a ROS service and topic. Since we cannot send a real sensor to all of our applicants, we made a simple simulator (sensor.py) that mimics the behavior of a real sensor but with random data. 
- The first task is to make a costume service for 3-DOF sensor 
- The second task is to make a ROS2 service server that continuously reads data from the sensor and has the latest filter data available for the client service that you make. 
- Finally, please make a simple client that calls two of these services and publishes them a topic with 500Hz. Please keep in mind that your service servers can run slower than 500Hz. 
- You can define a second server in the simulator to modify the code and run two at the same time.
- You can check the example.py to see how to make calls to the sensor

## Running instructions
This project was developed in Linux; as such, it has not been configured nor tested on any other platform. ROS2 Humble is required.

1. Source the ROS underlay:
```
source /opt/ros/humble/setup.bash
```

2. Navigate to this repository's root directory and check for missing dependencies:
```
cd machina_hw
rosdep install -i --from-path src --rosdistro humble -y
```

3. Build all included packages:
```
colcon build
```

4. In a new terminal, navigate to `machina_hw` and source the setup files. Repeat this step for a third and fourth terminal.
```
cd machina_hw
. install/setup.bash
```

5. In the first terminal, run `python3 sensor.py`. You should see messages indicating it's ready for connection.

6. In the second terminal, start up the first service server node:
```
ros2 run sensor_service service sensor_node_1 get_sensor_sample_1 127.0.0.1 10000 200 8
```

7. In the third terminal, start up the second service server node. You should see connection confirmation for both nodes in the first terminal:
```
ros2 run sensor_service service sensor_node_2 get_sensor_sample_2 127.0.0.1 10001 200 8
```

8. In the fourth terminal, start up the only service client node. You should see its output as it broadcasts to topic `sensor_topic`:
```
ros2 run sensor_service client 500
```

9. `Ctrl + C` to end each program.

## Thoughts

1. I notice that during development, when building packages only replacement-type or additive changes take effect. Specifically, renaming or removing files does not remove the corresponding component from the package on rebuild. I'm sure there is an option to do a clean build aside from removing the `build`, `install`, and `log` folders, but I'm not aware of it.

2. Would've liked to have had `sensor_service` and `sensor_interfaces` in the same package, but I wrote `sensor_service` using Python. Unsure of naming conventions for sibling packages.

3. I assume you wanted two nodes with one service server each; I could also see one node with two service servers.

4. I'm unsure what you meant by "filtered" data, so I chose to only keep the most recent data sampled from the sensor in `sensor_service.server`.

5. I chose to request 8 samples per sensor call. Because of the delay on top of the sampling time, we can never actually reach 2000Hz, or 4 samples per topic publish. To simplify the topic interface, I decided the next best thing to accomplish would be 3 samples per topic publish. To achieve this rate with a perfect 1ms delay requires 6 samples per sensor call, and requesting 7 samples per sensor call gives us an odd number, so I bumped it up to 8. Then all we have to do is store at least 9 samples since it takes approximately 2.5 periods of publishing topic data to obtain the samples from the sensor.