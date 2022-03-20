# Setup

## versions
**OS**: Ubuntu 20.04.03 LTS <br>
**g++**: (Ubuntu 9.3.0-17ubuntu1~20.04) 9.3.0 <br>
**clang-format**: 12 <br>
**cmake**: 3.16.3 <br>
**ROS2**: galactic <br>
**Gazebo**: 11.10.0 <br>


# ROS2 and Gazebo installation
install ROS-Base: https://docs.ros.org/en/galactic/Installation/Ubuntu-Install-Debians.html <br>
install Gazebo: 
```sh
curl -sSL http://get.gazebosim.org | sh
```

# Dependencies:
```sh
sudo apt update
```
install colcon: 
```sh
sudo apt install -y python3-colcon-common-extensions
```
install vcstool: 
```sh
sudo apt install -y python3-vsctool
```
install rosdep
```sh
sudo apt install -y python3-rosdep
```

# import and install dependencies
clone dependencies:
```sh
vcs import ./src < dependencies.repos
```
setup rosdep

```sh
sudo rosdep init
```
```sh
rosdep update
```
install dependencies
```sh
rosdep install --from-paths src --ignore-src -r -y
```

# build and launch file
make files executable
```sh
chmod +x sd_build.sh
chmod +x sd_launch.sh
```