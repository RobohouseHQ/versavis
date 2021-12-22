# VersaVIS -- An Open Versatile Multi-Camera Visual-Inertial Sensor Suite
[![Build Test](https://github.com/ethz-asl/versavis/actions/workflows/build_test.yml/badge.svg)](https://github.com/ethz-asl/versavis/actions/workflows/build_test.yml)

<a href="https://youtu.be/bsvSZJq76Mc"> <img src="https://user-images.githubusercontent.com/17647950/133631034-f633df6b-7ce7-4851-8cd3-f67c0bbec67a.jpg" alt="https://youtu.be/bsvSZJq76Mc" width="600"> </a>

VersaVIS provides a complete, open-source hardware, firmware and software bundle to perform time synchronization of multiple cameras with an IMU featuring exposure compensation, host clock translation and independent and stereo camera triggering.

## News
* Enjoy our new [trailer video](https://youtu.be/bsvSZJq76Mc) explaining the main conecpt behind VersaVIS.
* VersaVIS runs on Ubuntu 20.04 focal fossa / ROS Noetic now, look at this [issue](https://github.com/ethz-asl/versavis/issues/21#issuecomment-853007617).

## Supported camera drivers
* [Basler (not open-source)](https://github.com/ethz-asl/ros_basler_camera/tree/devel/versavis) tested with acA1920-155uc
* [MatrixVision](https://github.com/ethz-asl/bluefox2/tree/devel/versavis) tested with Bluefox 2 MLC200WG, needs adaption for new format
* [PointGrey/Flir](https://github.com/ethz-asl/flir_camera_driver/tree/devel/versavis) tested with Chameleon 3, Blackfly S
* [CamBoard](https://github.com/ethz-asl/pico_flexx_driver/tree/devel/versavis) tested with CamBoard pico monstar

## Citing

Please cite the [following paper](https://www.mdpi.com/1424-8220/20/5/1439) when using VersaVIS for your research:

```bibtex
@article{Tschopp2020,
author = {Tschopp, Florian and Riner, Michael and Fehr, Marius and Bernreiter, Lukas and Furrer, Fadri and Novkovic, Tonci and Pfrunder, Andreas and Cadena, Cesar and Siegwart, Roland and Nieto, Juan},
doi = {10.3390/s20051439},
journal = {Sensors},
number = {5},
pages = {1439},
publisher = {Multidisciplinary Digital Publishing Institute},
title = {{VersaVIS—An Open Versatile Multi-Camera Visual-Inertial Sensor Suite}},
url = {https://www.mdpi.com/1424-8220/20/5/1439},
volume = {20},
year = {2020}
}
```

Additional information can be found [here](https://docs.google.com/presentation/d/1Yi71cYtIBGUP5bFDKDFcUF2MjNS_CV3LM7W1_3jNLEs/edit?usp=sharing).
## Install - Instructions for BFS-USB3 and VN-100 IMU

### Prepare your hardware

The IMU is meant to communicate with the versavis board through UART, specifically Serial UART #2 of the VN100 (Serial UART #1 being reserved for programming the sensor). To do that, adapt the VN100 cable as follows:

|VN-100 Pin|Versavis UART connector pin|
|:---:|:---:|
|[User Manual](https://www.vectornav.com/resources/user-manuals/vn-100-user-manual) section 2.2|[Datasheet](https://drive.google.com/file/d/11QCjc5PVuMU9bAr8Kjvqz2pqVIhoMbHA/view?ts=5dc98776) section 6.6|
| 1 - VCC | 1 - 5V |
| 9 - RX2_TTL | 2 - TX1 |
| 8 - TX2_TTL | 3 - RX1 |
| 5 - GND | 4 - GND |
|2,3,4,6,7,10 - not connected|5,6,7 - not connected|

The camera has a Hirose HR10A-7R-6PB connector that connects with the Cam0 port of the versavis board.
THe pinout of the hirose connector can be found [here](https://flir.app.boxcn.net/s/o61v7jopcswquqagntbdre996ccvjday).

### Prepare your ROS workspace (in case you don't already have one setup)

You will need to clone the ```versavis```and ```flir-camera-driver``` packages into the same ROS workspace. 

```bash
# create new folder 
mkdir -p ~/temp_ws/src

# build ROS workspace 
cd temp_ws 

catkin build -DPYTHON_EXECUTABLE=/usr/bin/python3

# check that the build was excuted ok 
ls 
# You should see the following
			build devel src logs

# source workspace
source devel/setup.bash

# check that the workspace is on your ROS PATH
echo $ROS_PACKAGE_PATH
# You should see the following 
		/home/user/temp_ws/src:opt/ros/melodic/share

```

### Install the camera drivers

You will need to install the drivers for the BFS3-USB3 camera manually before being able to use the versavis framework. 

* Download and install ``` spinnaker camera``` 
    * get the files [here](https://www.flir.com/products/spinnaker-sdk/)
    * unpack the archive you downloaded and look for the directory containing the install script. (Ex: `spinnaker-2.5.0.80-amd64`)
    ```bash
    sudo sh install_spinnaker.sh

    ```
    * install additional dependencies
    ```bash
    sudo apt install libunwind8-dev
    ```
    * you can check that the drivers are installed properly by launching ```spinview```. Hit: Windows key + search for ```spinview``` , connect your camera to your PC via USB. It should get detected by ```spinview```


    > Notes: the installation script can install some QT5 dependencies that could interfere with your QT5 setup, should you have one. If you prefer not install these, comment out lines 48 and 49 of the installation script to skip them. Be aware however that you won't be able to use Spinview to test your installation if you do that. 

* Once you've installed the camera drivers, clone the related ROS package into the same workspace you plan to clone the ```versavis``` package.
``` bash
# Clone
cd ~/temp_ws/src

git clone https://github.com/ethz-asl/flir_camera_driver/

# Make sure you change to the versavis brach
git checkout devel/versavis 
```

* The build instructions in the package reference an outdated installation of ```spinnaker camera```: it looks for the drivers in the wrong folders. To address that, you need to modify the ```FindSpinnaker.cmake```

```bash
# open the FindSpinnaker.cmake file. It's stored in:
cd ~/temp_ws/src/flir_camera_driver/spinnaker_camera_driver/cmake/modules

```

* Add the Spinnaker installation directories as such:

```bash
# This is what the file should look line 
unset(Spinnaker_FOUND)
unset(Spinnaker_INCLUDE_DIRS)
unset(Spinnaker_LIBRARIES)

find_path(Spinnaker_INCLUDE_DIRS NAMES
  Spinnaker.h
  PATHS
  /opt/spinnaker/include/ # add this line 
  /usr/include/spinnaker/
  /usr/local/include/spinnaker/
)

find_library(Spinnaker_LIBRARIES NAMES Spinnaker
  PATHS
  /usr/lib
  /usr/local/lib
  /opt/spinnaker/lib # and this line
)

if (Spinnaker_INCLUDE_DIRS AND Spinnaker_LIBRARIES)
  set(Spinnaker_FOUND 1)
endif (Spinnaker_INCLUDE_DIRS AND Spinnaker_LIBRARIES)

```
* With that done, rebuild your ROS workspace and source it again
```bash
cd ~/temp_ws/src

catkin clean --y && catkin build # clears and rebuilds the workspace

source ../devel/setup.bash # sources the workspace

```

### Install the Versavis package 

```bash
# go to the workspace source directory
cd ~/temp_ws/src

# clone repository
git clone git@github.com:ethz-asl/versavis.git --recursive

# build package 
cd ..
catkin build versavis

# setup
cd versavis/firmware
./setup.sh

# Add yourself to the dialout group
sudo adduser <username> dialout

# go back to versavis directory
cd ..

#copy udev file to you system
sudo cp firmware/98-versa-vis.rules /etc/udev/rules.d/98-versa-vis.rules

# Afterwards use the following commands to reload the rules
sudo udevadm control --reload-rules
sudo udevadm trigger
sudo ldconfig

# Reboot!! 
```
### Configure
Adapt the [configuration file](https://github.com/ethz-asl/versavis/blob/master/firmware/libraries/versavis/src/versavis_configuration.h) to your setup needs. Also check the [datasheet](https://drive.google.com/file/d/11QCjc5PVuMU9bAr8Kjvqz2pqVIhoMbHA/view?ts=5dc98776) for how to configure the hardware switches.

### Flash firmware on the VersaVIS board
* Install Arduino IDE (from tarball, not package manager!!): [https://www.arduino.cc/en/software](https://www.arduino.cc/en/software). Version 1.8.16 was proven to work. 

* Open ```firmware/versavis/versavis.ino``` in the Arduino IDE
* Go to ```File -> Preferences```
* Change Sketchbook location to ```versavis/fimrware/```
* Install board support for the versavis board: 
    * Add [https://github.com/ethz-asl/versavis_hw/raw/master/package_VersaVIS_index.json](https://github.com/ethz-asl/versavis_hw/raw/master/package_VersaVIS_index.json) in ```Arduino IDE -> File -> Preferences -> Additional Boards Manager URLs```
    * Add the VersaVIS board to the board manager ```Tools -> Board -> Board Manager```
* Plug in the VersaVIS board to your PC via USB. You should be able to select the board in the ```Tools -> Board``` menu (Ex: on port /dev/ttyACM0) 
* Make sure you give your user permission to read the serial port, you can check [this](https://www.arduino.cc/en/Guide/Linux) or [this](https://www.arduino.cc/en/Guide/Linux) tutorial.

```bash
ls -l /dev/ttyUSB*
# or 
ls -l /dev/ttyACM*

# If you see the following, your user doesn't have permission to access the serial port 
crw-rw---- 1 root dialout 188, 0 Nov  4 15:28 /dev/ttyUSB0

# Add your user to the group 
sudo usermod -a -G dialout <username>

# The changes should be effective when you logout and log back in or reboot
```
* Set `Tools -> Port -> tty/ACM0 (Arduino Zero)`, and `Tools -> Board -> VersaVIS`.
* Compile the ```versavis.ino``` sketch using ```Verify```
* Flash the board using ```Upload```

## Calibration
Typically, a VI Setup needs to be carefully calibrated for camera intrinsics and camera-camera extrinsics and camera-imu extrinsics.
Refer to [Kalibr](https://github.com/ethz-asl/kalibr) for a good calibration framework. Note: To enable a good calibration, a high-quality calibration target needs to be available. Furthermore, a good and uniform light source is needed in order to reduce motion blur, especially during camera-imu calibration.

## Usage
* Adapt `versavis/launch/run_versavis.launch` to your needs.
* This repository has an adapted version test with the BFS-USB3 and VN100 setup: ```versavis/launch/run_versavis_with_rosbag.launch```
* Run with
```
roslaunch versavis run_versavis_with_rosbag_with_rosbag.launch
```
* Wait for successfull initialization.


## Troubleshooting

### I want to change the configuration of my VN100
The IMU is setup in ```/firmware/libraries/versavis/src/VN100.cpp```, ```VN100::setup()``` method, by directly writing to the relevant registers. You can modify this configuration by editing the commands sent to these registers, which you can get from the [VN100 User manual](https://www.vectornav.com/resources/user-manuals/vn-100-user-manual) or even test out with the [Vectornav Control Center](https://www.vectornav.com/resources/software) App (Warning: Windows only App!!)


### My sensor is stuck at initialization
Main symptoms are:
* No IMU message published.
* Cameras are not triggered or only very slowly (e.g. 1 Hz).

Troubleshooting steps:
* Check that your camera receives a triggering signal by checking the trigger LED (Note: As the trigger pulse is very short, look for a dim flicker.).
* Check that all topics are correctly set up and connected.
* If the USB3 blackfly is powered over the Hirose plug, there seems to be a longer delay (0.2s+) until the image arrives on the host computer. Initialization does not succeed because it only allows successful synchronization if the image message is not older than 0.1s compared to the time message coming from the triggering board. With power over USB things seem to work fine. One can increase the threshold [kMaxImageDelayThreshold](https://github.com/ethz-asl/versavis/blob/af83f34d4471a7886a197f305dbe76603b92747a/versavis/src/versavis_synchronizer.cpp#L20) but keep in mind that `kMaxImageDelayThreshold >> 1/f_init`. Decrease initialization frequency [`f_init`](https://github.com/ethz-asl/versavis/blob/af83f34d4471a7886a197f305dbe76603b92747a/firmware/versavis/versavis.ino#L87) if necessairy.

### The board is not doing what I expect / How can I enter debug mode
Easiest way to debug is to enable `DEBUG` mode in `firmware/libraries/versavis/versavis_configuration.h` and check the debug output using the Arduino Serial Monitor or `screen /dev/versavis`.
Note: In debug mode, ROS/rosserial communication is deactivated!
### I don't get any IMU messages on `/versavis/imu`
This is normal during initialization as no IMU messages are published. Check [Inintialization issues](https://github.com/ethz-asl/versavis#my-sensor-is-stuck-at-initialization) for further info.
### After uploading a new firmware, I am unable to communicate with the VersaVIS board
This is most likely due to an infinite loop in the code in an error case. Reset the board by double clicking the reset button and upload your code in `DEBUG` mode. Then check your debug output.
### IMU shows strange data or spikes
To decrease the bandwidth between VersaVIS and host but keep all information, only the IMu raw data is transferred and later scaled. If there is a scale offset, adapt the [scale/sensitivity parameters](https://github.com/ethz-asl/versavis/blob/af83f34d4471a7886a197f305dbe76603b92747a/versavis/launch/run_versavis.launch#L140) in your launch file.

Depending on the IMU, recursive data grabbing is implemented with a CRC or temperature check. If this fails multiple time, the latest message is used. Check your IMU if this persists.
### It looks like my exposure time is not correctly compensated
Check whether your board can correctly detect your exposure time in the [debug output](https://github.com/ethz-asl/versavis#my-sensor-is-stuck-at-initialization).
Troubleshooting steps:
* Enable exposure/strobe output on your camera.
* Check that all dip switches are set according to the [datasheet](https://drive.google.com/file/d/11QCjc5PVuMU9bAr8Kjvqz2pqVIhoMbHA/view?ts=5dc98776).
* Check that the exposure LED on the board flashes with exposure time.
### I receive errors on the host computer
#### Time candidate overflow
```bash
[ WARN] [1619791310.834852014]: /versavis/camO/tmage_raw: Time candidates buffer overflow at 1025.
...
```
Means that the synchronizer receives more timestamps than images. Double check if the camera is actually triggering with every pulse it receives. A typical problem is when the exposure time is higher than the measurement period.

#### Image candidate overflow
```bash
[ WARN] [1619791310.834852014]: /versavis/camO/tmage_raw: Image candidates buffer overflow at 1025.
...
```
Means that the synchronizer receives more iamges than timestamps. Double check if the camera is actually triggering and not free running.
