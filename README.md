# Sensors Onboarding

Welcome to the sensors subteam! We work on integrating, configuring, and fusing all the different sensors of the robot together so that it can localize itself and understand its surroundings. You can think of sensors as the backbone or glue of the team—we help connect subsystems together and provide the crucial information necessary for them to function. In this onboarding project, you will get your local environment setup so you can work with our software stack, learn ROS (the set of libraries and tools we use to run our robot), and get to work with real sensors on a project that should help give you an idea of what you will be doing on the team.

## Setup

Please follow the [environment setup instructions](https://github.com/umigv/environment) to get your local environment up and running, then clone this repository into the `ws/src` folder as follows:

```sh
cd [your environment folder]/ws/src
git clone --recursive https://github.com/umigv/sensors-onboarding.git
```

## Learning ROS

If you don't already know ROS or haven't used ROS 2 yet, go through the beginner ROS 2 tutorials linked [here](https://docs.ros.org/en/humble/Tutorials.html). You don't have to work through all of these (especially not step-by-step), but a once-over of the materials will help you understand a lot of the concepts that we will be working with in ROS like nodes, topics, publishers and subscribers, services, parameters, and more. The beginner tutorials (**CLI Tools** and **Client Libraries**) should be all you need to get started. You can skip **Beginner: CLI Tools/Configuring environment** since those steps have already been done for you in the Docker container. The ROS tutorial packages for **Beginner: Client Libraries** have also already been cloned for you in this repository under `ros_tutorials`, so you don't need to clone them yourself. When you get to the **Client Libraries** section, you only need to familiarize yourself with either the C++ or Python documentation (not both) based on your personal preference. However, you are welcome and encouraged to learn how to use the other language as well, since you are likely to use packages written in both languages.

## Hands-On Project - Modifying an IMU Driver

Now that you're familiar with ROS, let's start working with some real sensors! In this project, you will be adding some features to a ROS2 driver for an IMU sensor. You will have the opportunity to test your modified driver on real hardware and hopefully gain  a good understanding of how we can use ROS to interact with sensors! Make sure to check out the [tips](#tips) section below for some best practices.

### Overview

To start, open the `bno055` folder. This is a modified version of [this package](https://github.com/flynneva/bno055) that has had some features removed in order to serve for this project. If you ever get really stuck and can't figure something out, you can look at the original package for guidance, but before resorting to that please ask your fellow members or team leads for help, as they may be able to assist you better in actually learning rather than just copying the answer. You will be adding/fixing the following features to the driver:

- Accounting for gravity in the acceleration output as per [this ROS standard](https://www.ros.org/reps/rep-0145.html#data-sources)
  - This is necessary for many packages that work with IMUs (e.g. SLAM, localization) to operate properly and was a feature that was missing from previous drivers we've used on this team
- Add additional logging

### Guidance

This section will walk you through the structure of the package and provide guidance for the various steps of the project for your reference. You can (and are encouraged) to try to tackle these challenges on your own first, but feel free to reference this section and your peers for help if you ever you get stuck.

#### Structure

- A sample launch file to start the driver is in `launch/bno055.launch`
- Other import package files include `setup.cfg`, `setup.py`, and `requirements.txt`. These files tell `colcon` where to find the code and how to build the package
- Main driver and communication code is housed in the `bno055` folder
  - Node definition and general runtime logic in `bno055.py`
  - Direct sensor interface code defined in `sensor/SensorService.py`
  - Helper functions for interfacing with sensor using different connection protocols defined in `connectors`
  - Other code defines parameters for the node and error handling logic
- We will mostly be making changes to `bno055.py` and `sensor/SensorService.py`, but it is good to get familiar with the rest of the code as well to really understand how the drivers we use interact with the real hardware

#### Understanding Sensor Communication

In order to communicate with actual hardware sensors, there are multiple communications protocols that are commonly used. These protocols define how hardware components "talk" over wires, describing the exact timing, order, and meaning of the electrical signals sent over the set of wires connecting the devices. All together, these wires form a unit called a "bus", and include both signal and power wires. The protocols we will learn about here are digital, serial protocols. Digital means that messages are sent as a series of discrete values, in this case pulses on or off. Serial means that messages are sent one bit at a time, meaning that less wires are needed to transfer messages. For more information, check out [this page](https://learn.sparkfun.com/tutorials/serial-communication/all). This specific IMU driver has options to use both the [I2C](https://learn.sparkfun.com/tutorials/i2c/all) and [UART](https://www.embedded.com/understanding-the-uart/) protocols and provides abstractions to use either depending on your configuration. To actually connect to the sensors, a USB adapter is used to convert to the desired protocol. We won't get super deep into either of these protocols or the underlying serial communications details, but it's good to be aware of them and have a basic idea of what's going on under the hood.

#### The Project

For this project, we will work through adding some features to this IMU driver and fixing some bugs with it so you can gain a better understanding of how we work with sensors and how to debug issues with them. You will also get the opportunity to test your code on real hardware and see the impact in real time! After familiarizing yourself with the structure of the code, we will get started with the first task: accounting for gravity in the output acceleration.

##### Task 1: Gravity

ROS passes values like the acceleration of an object in a specific format called a messsage, with each message having a rigidly defined structure. The structure for an IMU acceleration message can be found [here](https://www.ros.org/reps/rep-0145.html). The important part here can be found under `Data Sources`, where it says that accelerometers should report a vector corresponding to the acceleration due to gravity in the z axis while the device is at rest. This is not currently accounted for in our BNO055 driver's IMU output, but we would like to account for it because certain SLAM algorithms expect the absolute gravity reference for localization. Therefore, your first task is to modify the IMU driver to add the acceleration due to gravity to the IMU output. There are already gravity registers available for you to use in `bno055/registers.py`. Try to figure out how to read the values from these registers and incorporate them into the IMU message output published on `/bno055/imu`.

Note: this driver already does correctly incorporate gravity into `/bno055/imu_raw`. It does this through a direct interface with a sensor register that does this for you, but for the purpose of this exercise you can just ignore that part and add the gravity accleration to `/bno055/imu` directly. We won't actually be using this code, but it's good to get a feel for how we can interface with the IMU ourselves and add new/missing features to pre-existing drivers.

##### Task 1 Hints

- If you launch the IMU driver with an IMU plugged in using `ros2 launch bno055 bno055.launch.py` and then view the output on the IMU topic using `ros2 topic echo /bno055/imu`, you can see under the `linear_acceleration` section that all values are near 0 when the sensor is at rest, meaning that gravity is not factored into the data. If you run `ros2 topic echo /bno055/imu_raw`, you can see our desired output, which includes the absolute gravity acceleration while the sensor is at rest.
- Ignore the lines in `sensor/SensorService.py` that refer to `imu_msg_raw`. We are only interested in modifying `imu_msg` in this exercise. When using this driver on our actual robot, we will probably just use the `imu_raw` topic when we need it, but for this exercise we want to emulate its functionality ourselves.
- A very useful resource to learn about a particular sensor is its datasheet. When working with a new sensor, it's always a good idea to find its datasheet and reference it to learn about how a sensor represents and reports the data it collects. In this case, the IMU sensor we are using is a BNO055 from Bosch, and we can find its datasheet [here](https://cdn-shop.adafruit.com/datasheets/BST_BNO055_DS000_12.pdf). An important area of interest is section **3.6.5 Output data registers** starting on page 34, where the registers that our data is stored in are listed along with their properties and sizes.
- Notice that in line 148 of `sensor/SensorService.py`, the driver calls `self.con.receive()` with a register argument of `registers.BNO055_ACCEL_DATA_X_LSB_ADDR` and a length argument of `45`. This line is calling a library function to read 45 bytes of data from the IMU registers starting at the memory address specified by `registers.BNO055_ACCEL_DATA_X_LSB_ADDR`.
  - You can think of the IMU registers as a linear array of memory. If you check `registers.py`, you can see the layout of these registers in memory. For example, `BNO055_ADDRESS_A` is at memory address `0x28`, `BNO055_ADDRESS_B` is at address `0x29`, and so on. You can think of these addresses as indices into the register memory array. They are encoded in hexadecimal, which is why they each start with `0x` and have letters as well as numbers. If you are interested in learning more, feel free to google hexadecimal notation and why computers use it, but for now the exact reason or meaning of the numbers isn't important. The driver we are using already has the memory addresses labeled, so we don't have to worry much about their actual values.
- Each register contains a value that is 1 byte in size, so by reading 45 registers, the driver is actually reading 45 different registers in a row into one buffer!
  - The memory address (or "index") given by `registers.BNO055_ACCEL_DATA_X_LSB_ADDR` is just a starting point, and the driver is actually reading all the registers in a row from `BNO055_ACCEL_DATA_X_LSB_ADDR` to `BNO055_TEMP_ADDR`.
  - This driver code is not documented very well to represent this, and one interesting exercise would be to redesign the code and data structures so that this behavior is more clear, either through changing the buffer structure to incorporate names or tags somehow or adding comments describing each memory location. When writing your own code in the future, be sure to think about how to make it readable and understandable by other people!
- In the code following the `receive()` call, the driver transforms individual bytes or pairs of bytes into the necessary forms needed for each part of the standard IMU message specified [here](https://github.com/ros2/common_interfaces/blob/humble/sensor_msgs/msg/Imu.msg). For example, by cross-referencing the code in `sensor/SensorService.py` with the register listings in `registers.py`, we can see that lines 208 and 209 of `sensor/SensorService.py` take the data from registers `LINEAR_ACCEL_DATA_X_LSB` and `LINEAR_ACCEL_DATA_X_MSB`, pack them into a float, scale them by an appropriate value so they are represented in the correct units, and set `imu_msg.linear_acceleration.x` to the resulting value.
  - A lot of important documentation on ROS packages can be found [here](https://index.ros.org/), including the IMU message documentation linked above. You will want to reference this site often, and especially with the sections on [sensor messages](https://index.ros.org/p/sensor_msgs/) and [geometry messages](https://index.ros.org/p/geometry_msgs/) for this subteam.
- We can see in `registers.py` that there are registers that just hold the gravity data. How can we access the data from these registers and add them to the linear acceleration values in `imu_msg`?
  - Hint: you don't need to call `self.con.receive()` again—the data we want is already in `buf`.

##### Task 2: Logging

Coming soon!

## Tips

- Make sure you always run `colcon` commands from the root directory of your workspace, e.g. `~/ws`. Don't run them from subdirectories like `~/ws/src`!
- To test with RViz, make sure you set the fixed frame to `bno055` in the RViz settings
- If you are on Mac, you probably won't be able to test your code by plugging in an IMU directly since Mac doesn't allow USB passthrough to Docker containers eassily. When you need to test your code, push your code to a fork and test it on a team computer running Linux as follows:
  - On Github, fork the `sensors-onboarding` repo by clicking the Fork button in the upper right corner
  - Once you've forked the repository, click the green Code dropdown and copy the link to your forked repository
  - Open your `sensors-onboarding` repo on your local machine and enter the following commands:
    - `git remote remove origin`
    - `git remote add origin [link to your fork here]`
    - `git remote add upstream https://github.com/umigv/sensors-onboarding.git`
  - Set your `main` branch to track your fork instead of the original repository with `git branch main -u origin/main`
  - Push your code with `git push`
  - On the team laptop, pull your code to an appropriate folder with `git clone [link to your fork here] sensors-onboarding-[your uniqname]` (your uniqname is there to keep different people's code separate)
  - For more information on forks, check [this page](https://docs.github.com/en/get-started/quickstart/fork-a-repo)

## Timeline

- Week 1: Introductions, environment setup
- Week 2: Finish environment setup, start on ROS tutorials
- Week 3: Continue with ROS tutorials, introduce main onboarding project
- Week 4: Work time for both tutorials and onboarding project
- Week 5: Wrap up onboarding project and introduce robot goals and projects
