# Raw Converter

This repository contains software to convert 12-bit (stored as 16-bit) grayscale raw images to jpgs as well as a ROS node to interface with the See3CAM CU51 Monochrome camera.

## Requirements 

Ubuntu 14.04.5 LTS (Trusty Tahr)

OpenCV 2.4.9

CMake

## Raw Converter Usage

#### 1. Build and make project
```
mkdir build && cd build
cmake ..
make
````

#### 2. Execute the raw converter, passing in the path to the directoy of raw images
From within the build folder, execute
```
./raw_converter -d /path/to/images -H image_height -W image_width
```
where you replace '/path/to/images' with the real path to the directory containing the .raw images you wish to convert. You must also provide the image height and width for the conversion to handle different resolutions. The converted images will show up in a directory named 'jpgs' in the raw images path directory.
