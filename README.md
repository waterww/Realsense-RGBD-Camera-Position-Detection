# Realsense-RGBD-Camera-Position-Detection
The program is based on ros_pcl and pcl, filter and cluster the raw pointcloud data to get object position at 30Hz.

# How to use it?
1.install [librealsense2](http://wiki.ros.org/librealsense2) and [realsense ros](wiki.ros.org/realsense2_camera)

2.verify that the packages are installed successfully

    roslaunch realsense2_camera rs_camera.launch

3.add the files in repository to **realsense2_camera** package.
If the file already exits (eg.CMakeLists.txt), please substitute it.

4.run the program

launch the node to publish raw pointcloud
    
    roslaunch realsense2_camera demo_poincloud.launch

launch the nodelets to preprocess pointcloud
    
    roslaunch realsense2_camera pcl_preprocess.launch
    
run the node to clustering pointcloud
    
    rosrun realsense2_camera detection
    
5.detected objects information is published in topic **/objectlist** as user-defined ros message

# Paramters
## pcl_preprocess.launch

**filter_limit_min**: the minimum range to keep points. Z direction points outward from the camera.

**filter_limit_max**: the maximum range to keep points.

**distance_threshold**: the threshold of plane thickness.

## detection.cpp

**setClusterTolerance**: epsilon, radius to search points.

**setMinClusterSize**: minmum number of points in one cluster.

**setMaxClusterSize**: maximum number of points in one cluster.

# Topics
* /voxel_grid/output
* /extract_plane/output
* /cloud_cluster
