# Raw Converter

This repository contains software to convert 12-bit (stored as 16-bit) grayscale raw images to jpgs.

## Requirements 

Ubuntu 14.04.5 LTS (Trusty Tahr)

ROS Indigo
# raw_converter
2

## Usage

Once the package is built within the ROS framework, navigate to
```
cd catkin_ws/devel/lib/raw_converter
```
and execute the following
```
./raw_converter -d /path/to/images
```
where you replace '/path/to/images' with the real path to the directory containing the .raw images you wish to convert. The converted images will show up in a directory named 'jpgs' in the raw images path directory.
