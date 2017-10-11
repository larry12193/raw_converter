# Raw Converter

This repository contains software to convert 12-bit (stored as 16-bit) grayscale raw images to jpgs as well as a ROS node to interface with the See3CAM CU51 Monochrome camera.

## Requirements 

Ubuntu 14.04.5 LTS (Trusty Tahr)

ROS Indigo

OpenCV 2.4.9 (w/ patch)

e-con Systems custom OpenCV 2.4.9 patch (for custom Y16 grayscale image format)

## Installation

### ROS Indigo

Follow the standard installation steps found at http://wiki.ros.org/indigo/Installation/Ubuntu. Depending on the target application, download the Desktop-Full for a personal computer and the Desktop Install for a more embedded applicaton.

### OpenCV 2.4.9 with e-con Systems Patch

The steps outlined need to be followed closely to ensure that the custom image format that is provided by e-con Systems is properly built with OpenCV so that the ROS node can read and publish the grayscale images streaming from the camera.

#### 1. Clone OpenCV source into the dependencies folder
```
cd dependecies
git clone https://github.com/opencv/opencv.git
git checkout tags/2.4.9
```
#### 2. Unzip the e-con Systems patch source and install the source code patch
```
unzip e-con_Modified_OpenCV.zip
cd e-con_Modified_OpenCV
sudo chmod +x install_econ_patch.sh
sudo ./install_econ_patch.sh
```

#### 3. Create a patched opencv installation folder in the dependencies folder
```
cd ../ && mkdir opencv_econ
```

#### 4. Create enviornment variable that points to the installation folder
```
gedit ~/.bashrc
```
Add the following line to the bottom of your bashrc
```
export OPENCV_ECON_PATH=/path/to/opencv_econ
```
where you replace '/path/to/opencv_econ' with the actual fixed path to the folder. This is key in allowing cmake to find the patched OpenCV files.

#### 5. Open a new terminal and enter the opencv source folder and create and enter a build folder
```
mkdir -p opencv/build
cd opencv/build
````

#### 6. Run cmake for opencv
```
cmake -D CMAKE_BUILD_TYPE=RELEASE -D WITH_TBB=ON -D BUILD_TBB=ON -D WITH_V4L=ON -D WITH_LIBV4L=OFF -D BUILD_TESTS=OFF -D BUILD_PERF_TESTS=OFF -D CMAKE_INSTALL_PREFIX=../../opencv_econ ..
```
Notice: The CMAKE_INSTALL_PREFIX is set to the opencv_econ folder created. This is where the patched opencv will live and be referenced as to no interfere with the opencv installed by ROS.

#### 7. Make and install opencv
```
sudo make -j4 install
```

### Udev Rules

#### 1. Enter the udev_rules directory in the dependencies folder
```
cd dependenceis/udev_rules
```

#### 2. Run the udev rules installation script
```
sudo chmod +x install-udev-rules.sh
sudo ./install-udev-rules.sh
```

Unplug and plug the camera back into your computer for the udev permissions to take effect.

## Raw Converter Usage

#### 1. Build the entire catkin_ws 
```
cd catkin_ws
catkin_make
````

#### 2. Navigate to the built executable directory
```
cd devel/lib/raw_converter
```

#### 3. Execute the raw converter, passing in the path to the directoy of raw images
```
./raw_converter -d /path/to/images
```
where you replace '/path/to/images' with the real path to the directory containing the .raw images you wish to convert. The converted images will show up in a directory named 'jpgs' in the raw images path directory.
