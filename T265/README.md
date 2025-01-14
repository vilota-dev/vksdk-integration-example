# Intel librealsense

This sample shows how to get the camera pose in a world reference and output the odometry to eCAL. No need to have NVIDIA GPU.

## Getting Started
Clone librealsense and checkout version 2.50.0 as this is the latest version which supports Intel T265.
 ```
git clone https://github.com/IntelRealSense/librealsense.git 
cd librealsense 
git checkout v2.50.0
 ```

## Build the program for librealsense SDK
```
mkdir build && cd build 
cmake .. -DFORCE_RSUSB_BACKEND=TRUE -DBUILD_WITH_CUDA=OFF -DBUILD_EXAMPLES=ON -DCMAKE_BUILD_TYPE=Release

make -j$(nproc) 
sudo make install
```
## Install vk-system
Install vk-system binary, please contact Vilota support for obtaining the deb file

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
Navigate to the T265 folder
```
mkdir build && cd build
cmake ..
make -j$(nproc)
```

## Run the sample code
Inside the build directory run
```
./T265_publisher
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
