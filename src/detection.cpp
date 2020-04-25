#include <ros/ros.h>
// PCL specific includes
#include <pcl_conversions/pcl_conversions.h>
#include <pcl/conversions.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl_ros/point_cloud.h>

#include <pcl/kdtree/kdtree.h>
#include <pcl/segmentation/extract_clusters.h>

#include <realsense2_camera/Object.h>
#include <realsense2_camera/ObjectArray.h>

#include <pcl/visualization/cloud_viewer.h>
#include <pcl/common/common.h>
#include <sensor_msgs/PointCloud2.h>

#include <iostream>
#include <cstdlib>

ros::Publisher object_pub;
ros::Publisher cluster_pub;

void clustering_cb(const pcl::PCLPointCloud2ConstPtr& cloud)
{
  //clock_t startTime, endTime;
  //startTime = clock();

  if (cloud->width != 0)
  {
    //convert pointcloud2 to PointXYZRGB
    pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_blob(new pcl::PointCloud<pcl::PointXYZ>);
    pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud_rgb(new pcl::PointCloud<pcl::PointXYZRGB>);
    pcl::fromPCLPointCloud2(*cloud,*cloud_blob);
    pcl::fromPCLPointCloud2(*cloud,*cloud_rgb);

    //clustering the points
    pcl::search::KdTree<pcl::PointXYZ>::Ptr tree (new pcl::search::KdTree<pcl::PointXYZ>);
    tree->setInputCloud (cloud_blob); //创建点云索引向量，用于存储实际的点云信息

    std::vector<pcl::PointIndices> cluster_indices;
    pcl::EuclideanClusterExtraction<pcl::PointXYZ> ec;
    ec.setClusterTolerance (0.1); //设置近邻搜索的搜索半径为2cm
    ec.setMinClusterSize (10);//设置一个聚类需要的最少点数目为100
    ec.setMaxClusterSize (25000);//设置一个聚类需要的最大点数目为25000
    ec.setSearchMethod (tree);//设置点云的搜索机制
    ec.setInputCloud (cloud_blob);
    ec.extract (cluster_indices);//从点云中提取聚类，并将点云索引保存在cluster_indices中

    int j = 0;
    realsense2_camera::ObjectArray ob_array;
    
  
    for (std::vector<pcl::PointIndices>::const_iterator it = cluster_indices.begin (); it != cluster_indices.end (); ++it)
    {

      j++;

      realsense2_camera::Object object;
      

      double x = 0.0;
      double y = 0.0;
      double z = 0.0;

      ros::Time currenttime = ros::Time::now();

      //产生rgb随机数
      srand(j);
      int r = rand()%256;
      int g = rand()%256;
      int b = rand()%256;
      //std::cout << "cluster color:" << r << "," << g << "," << b << std::endl;

      pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_cluster_blob (new pcl::PointCloud<pcl::PointXYZ>);
      //创建新的点云数据集cloud_cluster_blob，将所有当前聚类写入到点云数据集中
      for (std::vector<int>::const_iterator pit = it->indices.begin (); pit != it->indices.end (); ++pit)
      {
        //save each point to one cluster
        cloud_cluster_blob->points.push_back (cloud_blob->points[*pit]); 
        
        //calculate average x,y,z
        x = x + cloud_blob->points[*pit].x;
        y = y + cloud_blob->points[*pit].y;
        z = z + cloud_blob->points[*pit].z;

        //set points in the same color
        cloud_rgb->points[*pit].r = r;
        cloud_rgb->points[*pit].g = g;
        cloud_rgb->points[*pit].b = b;


      }

      //find min and max x,y,z
      pcl::PointXYZ minPt, maxPt;
      pcl::getMinMax3D (*cloud_cluster_blob, minPt, maxPt);

      //create publish object message
      object.header.frame_id =  "/camera_depth_frame";//需要修改
      object.header.stamp = currenttime;
      object.id = j;
      object.x = x/cloud_cluster_blob->points.size();
      object.y = y/cloud_cluster_blob->points.size();
      object.z = z/cloud_cluster_blob->points.size();
      object.x_distance = maxPt.x - minPt.x;
      object.y_distance = maxPt.y - minPt.y;
      object.z_distance = maxPt.z - minPt.z;

      //std::cout << "number:" << j << std::endl;
      //std::cout << "position: ("<<object.x << "," << object.y << "," << object.z << ")" << std::endl;
      
      //save the object to list
      ob_array.objects.push_back(object);

      

    }

    //publish detected objectlist of one scan
    object_pub.publish(ob_array);
  
    //publish clustering result
    cluster_pub.publish(cloud_rgb);

    
  }

  //endTime = clock();
  //std::cout << (double)(endTime - startTime)/CLOCKS_PER_SEC << std::endl;
  //std::cout << "" << std::endl;
  
}


int main (int argc, char** argv)
{
  // Initialize ROS
  ros::init (argc, argv, "detection");
  ros::NodeHandle nh;

  // Create a ROS subscriber for the input point cloud
  ros::Subscriber sub = nh.subscribe ("/extract_plane/output", 10, clustering_cb);

  object_pub = nh.advertise<realsense2_camera::ObjectArray> ("/objectlist",10);
  cluster_pub = nh.advertise<pcl::PointCloud<pcl::PointXYZRGB>>("/cloud_cluster",1);

  // Spin
  ros::spin();
}
