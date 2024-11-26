# BNO055_ROS2

## Overview
BNO055_ROS2 is a Ros2 Humble package designed to interface with the Bosch BNO055 sensor on Jetson devices, providing orientation and motion sensing capabilities. This project uses BNO055_SensorAPI and libi2c libraries.

## Features
- 9-axis sensor fusion
- Real-time orientation data
- 

## Requirements
- C++17 or higher
- CMake (3.8 or higher)
- ROS 2 Humble
- Jetson device

## Installation
1. Clone the repository to the src directory of your workspace with submodules:
    ```sh
    git clone --recurse-submodules https://github.com/alpertng02/jetson-imu.git
    ```
2. Navigate to the project directory:
    ```sh
    cd your_workspace
    ```
3. Build using colcon:
    ```sh
    colcon build --packages-select jetson_imu
    ```
## Usage
1. Connect the BNO055 sensor to your Jetson device through GPIO 3 and 5.
2. Source the ROS 2 setup script:
    ```sh
    source /opt/ros/<distro>/setup.bash
    source install/setup.bash
    ```
3. Run the ROS node:
    ```sh
    ros2 run jetson_imu bno055_publisher /dev/i2c-1 0x28
    ```

## License
This project is licensed under the Apache-2.0 License. See the [LICENSE](LICENSE) file for details.

## Contributing
Contributions are welcome! Please open an issue or submit a pull request.

## Contact
For any questions or suggestions, please contact alpert.guven@gmail.com.
