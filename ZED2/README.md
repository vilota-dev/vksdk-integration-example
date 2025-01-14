# ZED SDK

This sample shows how to get the camera pose in a world reference and output the odometry to eCAL.

## Getting Started
 - Get the latest [ZED SDK](https://www.stereolabs.com/developers/release/)
 - Check the [Documentation](https://www.stereolabs.com/docs/)

## Build the program for ZED SDK
 - Build for [Windows](https://www.stereolabs.com/docs/app-development/cpp/windows/)
 - Build for [Linux/Jetson](https://www.stereolabs.com/docs/app-development/cpp/linux/)
 
## Install vk-system
 - Install vk-system binary, please contact vilota support for obtaining the deb file

## Install sophus and eigen
```
git clone https://github.com/strasdat/Sophus.git
cd Sophus/scripts
./install_ubuntu_deps_incl_ceres.sh
cd ..
mkdir build && cd build
cmake ..
make -j$(nproc)
sudo make install
```

## Compile this sample code
Navigate to the ZED2 folder
```
mkdir build && cd build
cmake ..
make -j$(nproc)
```

## Run the sample code
Inside the build directory run
```
./ZED2_publisher
```

## Observe results
 - Open eCAL Monitor
 - Running vk_viewer
      - Add these to the vk_viewer config located in /opt/vilota/configs/tools/viewer/***.json
      ```
      "odom" : [
          "S0/vio_odom",
          "S1/vio_odom",
	      "T265/vio_odom",
	      "ZED2/vio_odom"
      ],
      ```
