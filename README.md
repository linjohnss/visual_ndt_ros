# visual_ndt_ros
[![demo_tango_visual_ndt](https://user-images.githubusercontent.com/61956056/187384658-648a06e5-3c2d-4127-977e-84175bc07ba3.gif)](https://youtu.be/zvkta897cJQ "Tango Demo")
## Overview
ROS base Camera Localization in LiDAR Map, using ndt_omp for 3D-3D point cloud matching. 
## Installation
This is a ROS package, you can use catkin_make to build it.
### ndt_omp
```shell
git clone https://github.com/koide3/ndt_omp.git
```
### visual_ndt_ros
```shell
git clone https://github.com/linjohnss/visual_ndt_ros.git
```
## Dowe
Download tango dataset
1. pointcloud map : `tango_test.pcd` from [URL](https://drive.google.com/file/d/1kzbPUMD_gfyvqm9TltnuP_rLgSJ5-_B9/view?usp=sharing)
2. rosbag : `tango_test.bag` from [URL](https://drive.google.com/file/d/1eUDrwx4n6b3ECQn7VCzqvpp7EUh1GG1l/view?usp=sharing)

```shell
roslaunch visual_ndt_ros demo_tango_ndt.launch
```

## Reference
[基於智慧行動裝置的視覺 SLAM 與光達點雲匹配之定位整合研究](https://hackmd.io/@linjohn/vndt)
