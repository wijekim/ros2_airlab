# dxl_wsl

ros2 dxl test program on WSL2-ubuntu 20.04

# run subscriber on Jetson nano

$ cd ~/ros2_ws/src

$ git clone https://github.com/2sungryul/dxl_nano.git

$ cd ~/ros2_ws

$ colcon build --symlink-install --packages-select dxl_nano

$ source install/local_setup.bash

$ sudo chmod a+rw /dev/ttyUSB0

$ ros2 run dxl_nano sub

# run dxl publisher on WSL2-Ubuntu 20.04

$ cd ~/ros2_ws/src

$ git clone https://github.com/2sungryul/dxl_wsl.git

$ cd ~/ros2_ws

$ colcon build --symlink-install --packages-select dxl_wsl

$ source install/local_setup.bash

$ ros2 run dxl_wsl pub
