# Running OpenVINS without roscore

## Building OpenVINS

In order to build OpenVINS, ROS still needs to be installed on the machine due to library dependencies.
Here are the install steps assuming ROS kinetic or melodic are installed:
```
sudo apt-get install python-catkin-tools
mkdir -p ~/workspace/catkin_ws_ov/src/
cd ~/workspace/catkin_ws_ov/src/
git clone https://github.com/ILLIXR/open_vins.git
cd ..
catkin build
```

## Running OpenVINS

Now to run OpenVINS do the following:
```
cd ~/workspace/catkin_ws_ov/devel/lib/ov_msckf
./run_illixr_msckf path_to_cam0 path_to_cam1 path_to_imu0 path_to_cam0_images path_to_cam1_images
```

When running OpenVINS we assume the data is formatted in the [EUROC Dataset standard](https://projects.asl.ethz.ch/datasets/doku.php?id=kmavvisualinertialdatasets).