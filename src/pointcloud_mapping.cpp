#include <iostream>
#include <ros/ros.h>
#include <pcl/io/pcd_io.h>
#include <pcl/point_types.h>
#include <pcl/point_cloud.h>
#include <pcl/filters/voxel_grid.h>
#include <sensor_msgs/PointCloud2.h>
#include <tf/transform_listener.h>
#include <tf/transform_broadcaster.h>
#include <pcl/io/pcd_io.h>
#include <pcl_conversions/pcl_conversions.h>
#include <pcl_ros/transforms.h>
#include <pcl_ros/point_cloud.h>
#include <signal.h>
#include <pcl/visualization/pcl_visualizer.h>

sensor_msgs::PointCloud2 pcl2pointcloud (pcl::PointCloud<pcl::PointXYZ>::Ptr cloud, char *fix_frame);
void cloud_callback(const sensor_msgs::PointCloud2::ConstPtr& msg);
void signalHandler(int signal);
pcl::PointCloud<pcl::PointXYZ>::Ptr target_cloud(new pcl::PointCloud<pcl::PointXYZ>());
ros::Subscriber sub_sourcecloud;
ros::Publisher pub_targetcloud;
bool is_initial;
Eigen::Matrix4f pre_trans;
Eigen::Matrix4f delta_trans;
char lidar_map_dir[] = "/home/ros20/Desktop/ndt_ws/src/visual_ndt_ros/map/target_cloud3.pcd";

int main(int argc, char** argv) {
    ros::init (argc, argv, "pointcloud_mapping");
    ros::NodeHandle nh;
    signal(SIGTSTP,signalHandler);
    is_initial = true;
    ros::Subscriber sub_sourcecloud = nh.subscribe("/velodyne_points", 1, cloud_callback);
    pub_targetcloud = nh.advertise<sensor_msgs::PointCloud2> ("/cloud_map", 1);
    ros::spin ();
	return 0;
}

sensor_msgs::PointCloud2 pcl2pointcloud (pcl::PointCloud<pcl::PointXYZ>::Ptr cloud, char *fix_frame) {
    sensor_msgs::PointCloud2 ros_cloud;
    pcl::toROSMsg(*cloud, ros_cloud); //convert from PCL to ROS type this way
    ros_cloud.header.frame_id = fix_frame;
    return ros_cloud;
}

void cloud_callback(const sensor_msgs::PointCloud2::ConstPtr& msg) {
    static tf::TransformListener listener;
    static tf::StampedTransform transform;
    pcl::PointCloud<pcl::PointXYZ>::Ptr input_cloud(new pcl::PointCloud<pcl::PointXYZ>());
    try {
        pcl::fromROSMsg (*msg, *input_cloud);
        listener.waitForTransform("/camera_init", "aft_mapped", ros::Time(0), ros::Duration(3.0) );
        listener.lookupTransform("/camera_init", "aft_mapped", ros::Time(0), transform);
    } catch (tf::TransformException ex){
        ROS_WARN("%s",ex.what());
        ros::Duration(1.0).sleep();
        return;
    }
    Eigen::Quaterniond init_rotation_Qua = Eigen::Quaterniond(transform.getRotation());
    Eigen::Translation3f init_translation (transform.getOrigin().x(), transform.getOrigin().y(), transform.getOrigin().z());
    Eigen::Matrix3f init_rotation = init_rotation_Qua.toRotationMatrix().cast <float> ();
    Eigen::Matrix4f init_guess = (init_translation * init_rotation).matrix ();
    
    if(is_initial){
        delta_trans = Eigen::Matrix4f::Identity();
        pre_trans = init_guess;
        pcl::transformPointCloud(*input_cloud, *input_cloud, init_guess);
        *target_cloud = *input_cloud;
        is_initial = false;
        std::cout<<"initialed !!" << std::endl;
    }
    else{
        delta_trans = pre_trans.inverse() * init_guess;
        // std::cout<<"concat !!" << std::endl;
        if ((fabs(delta_trans(0,3)) + fabs(delta_trans(1,3)) + fabs(delta_trans(2,3))) > 0.2) {
            std::cout<<"concat !!" << std::endl;
            pcl::transformPointCloud(*input_cloud, *input_cloud, init_guess);
            *target_cloud = *target_cloud + *input_cloud;
            pre_trans = init_guess;
        }
    }

    pub_targetcloud.publish( pcl2pointcloud(target_cloud, (char*)"camera_init"));

    return;
}

void signalHandler(int signal){
    if(signal == SIGTSTP){
        //downsampling
        // pcl::PointCloud<pcl::PointXYZ>::Ptr downsampled(new pcl::PointCloud<pcl::PointXYZ>());
        // pcl::VoxelGrid<pcl::PointXYZ> voxelgrid;
        // voxelgrid.setLeafSize(0.7f, 0.7f, 0.7f);
        // voxelgrid.setInputCloud(target_cloud);
        // voxelgrid.filter(*downsampled);
        // target_cloud = downsampled;
        pcl::io::savePCDFileASCII (lidar_map_dir, *target_cloud);  
        std::cout<<"Saved !!" << std::endl;
        pcl::PointCloud<pcl::PointXYZ>::Ptr visualize_cloud(new pcl::PointCloud<pcl::PointXYZ>());
        pcl::io::loadPCDFile(lidar_map_dir, *visualize_cloud);
        // visualization
        pcl::visualization::PCLVisualizer vis("vis");
        pcl::visualization::PointCloudColorHandlerCustom<pcl::PointXYZ> visualize_handler(visualize_cloud, 255.0, 0.0, 0.0);
        vis.addPointCloud(visualize_cloud, visualize_handler, "target");
        vis.spin();
    }
}