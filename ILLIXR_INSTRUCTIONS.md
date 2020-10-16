# Running OpenVINS without roscore

## Building OpenVINS

```
git clone https://github.com/ILLIXR/open_vins.git
cd open_vins
mkdir build
cd build
cmake ../
make -j
```

## Running OpenVINS

Now to run OpenVINS do the following:
```
cd build/ov_msckf
./run_illixr_msckf path_to_cam0 path_to_cam1 path_to_imu0 path_to_cam0_images path_to_cam1_images
```

When running OpenVINS we assume the data is formatted in the [EUROC Dataset standard](https://projects.asl.ethz.ch/datasets/doku.php?id=kmavvisualinertialdatasets).
