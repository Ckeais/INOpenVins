# INOpenVIns


# OpenVins documentation:

* Github project page - https://github.com/rpng/open_vins
* Documentation - https://docs.openvins.com/
* Getting started guide - https://docs.openvins.com/getting-started.html
* Publication reference - http://udel.edu/~pgeneva/downloads/papers/c10.pdf

# OpenVins Set Up:

## install ROS1
```txt
sudo sh -c 'echo "deb http://packages.ros.org/ros/ubuntu $(lsb_release -sc) main" > /etc/apt/sources.list.d/ros-latest.list'
sudo apt-key adv --keyserver 'hkp://keyserver.ubuntu.com:80' --recv-key C1CF6E31E6BADE8868B172B4F42ED6FBAB17C654
sudo apt-get update
export ROS1_DISTRO=noetic # kinetic=16.04, melodic=18.04, noetic=20.04
sudo apt-get install ros-$ROS1_DISTRO-desktop-full
sudo apt-get install python-catkin-tools # ubuntu 16.04, 18.04
sudo apt-get install python3-catkin-tools python3-osrf-pycommon # ubuntu 20.04
sudo apt-get install libeigen3-dev libboost-all-dev libceres-dev
echo "source /opt/ros/$ROS1_DISTRO/setup.bash" >> ~/.bashrc
source ~/.bashrc
```

# clone openvins
```txt
mkdir -p ~/workspace/catkin_ws_ov/src/
cd ~/workspace/catkin_ws_ov/src/
git clone https://github.com/rpng/open_vins/
cd ..
catkin build # ROS1
```

# additional evaluation requirement
```txt
sudo apt-get install python3-dev python3-matplotlib python3-numpy python3-psutil python3-tk # for python3 systems
catkin build -DDISABLE_MATPLOTLIB=OFF # build with viz (default)
```

# openCV dependency
```txt
# Try to first build with your system / ROS OpenCV. Only fall back onto this if it does not allow you to compile, or want a newer version!
git clone https://github.com/opencv/opencv/
git clone https://github.com/opencv/opencv_contrib/
mkdir opencv/build/
cd opencv/build/
cmake -DOPENCV_EXTRA_MODULES_PATH=../../opencv_contrib/modules ..
make -j8
sudo make install
```

# Ceres Solver dependency
```txt
sudo apt-get install libceres-dev
```

# The UZH-FPV Drone Racing Dataset:

* https://fpv.ifi.uzh.ch/

```txt
roscore # term 0
source devel/setup.bash # term 1
roslaunch ov_msckf subscribe.launch config:=uzhfpv_indoor
```
In another two terminals we can run the following. For RVIZ, one can open the ov_msckf/launch/display.rviz configuration file. You should see the system publishing features and a state estimate.

```txt
rviz # term 2
rosbag play V1_01_easy.bag # term 3
```


## Credit / Licensing

This OpenVins code was written by the [Robot Perception and Navigation Group (RPNG)](https://sites.udel.edu/robot/) at the
University of Delaware. If you have any issues with the code please open an issue on our github page with relevant
implementation details and references. For researchers that have leveraged or compared to this work, please cite the
following:

```txt
@Conference{Geneva2020ICRA,
  Title      = {{OpenVINS}: A Research Platform for Visual-Inertial Estimation},
  Author     = {Patrick Geneva and Kevin Eckenhoff and Woosik Lee and Yulin Yang and Guoquan Huang},
  Booktitle  = {Proc. of the IEEE International Conference on Robotics and Automation},
  Year       = {2020},
  Address    = {Paris, France},
  Url        = {\url{https://github.com/rpng/open_vins}}
}
```

The codebase is licensed under the [GNU General Public License v3 (GPL-3)](https://www.gnu.org/licenses/gpl-3.0.txt).


