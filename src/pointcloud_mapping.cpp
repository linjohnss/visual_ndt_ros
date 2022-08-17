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
void ndt_node(ros::NodeHandle node);
void signalHandler(int signal);

//pclomp::NormalDistributionsTransform<pcl::PointXYZ, pcl::PointXYZ>::Ptr ndt_omp(new pclomp::NormalDistributionsTransform<pcl::PointXYZ, pcl::PointXYZ>());
pcl::PointCloud<pcl::PointXYZ>::Ptr target_cloud(new pcl::PointCloud<pcl::PointXYZ>());
pcl::PointCloud<pcl::PointXYZ>::Ptr input_cloud(new pcl::PointCloud<pcl::PointXYZ>());
ros::Subscriber sub_sourcecloud;
ros::Subscriber sub_device_tf;
ros::Publisher pub_targetcloud;
ros::Publisher pub_transformcloud;
bool is_initial;

int main(int argc, char** argv) {
    ros::init (argc, argv, "pointcloud_mapping");
    ros::NodeHandle nh;
    signal(SIGTSTP,signalHandler);
    ndt_node(nh);

    ros::Rate loop_rate(100);

    while(ros::ok())
    {
        ros::spinOnce(); //invokes callback
        loop_rate.sleep();
    }
    
	return 0;
}

void ndt_node(ros::NodeHandle nh) {
    sub_sourcecloud = nh.subscribe ("/velodyne_points", 1, cloud_callback);
    pub_targetcloud = nh.advertise<sensor_msgs::PointCloud2> ("/target_cloud", 1);
    pub_transformcloud = nh.advertise<sensor_msgs::PointCloud2> ("/transform_cloud", 1);
    is_initial = true;
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
    listener.waitForTransform("/camera_init", "aft_mapped", ros::Time(0), ros::Duration(10.0) );
    listener.lookupTransform("/camera_init", "aft_mapped", ros::Time(0), transform);
    Eigen::Quaterniond init_rotation_Qua = Eigen::Quaterniond(transform.getRotation());
  	Eigen::Translation3f init_translation (transform.getOrigin().x(), transform.getOrigin().y(), transform.getOrigin().z());
    Eigen::Matrix3f init_rotation = init_rotation_Qua.toRotationMatrix().cast <float> ();
    Eigen::Matrix4f init_guess = (init_translation * init_rotation).matrix ();
    pcl::fromROSMsg (*msg, *input_cloud);
    //downsampling
	pcl::PointCloud<pcl::PointXYZ>::Ptr downsampled(new pcl::PointCloud<pcl::PointXYZ>());
    pcl::VoxelGrid<pcl::PointXYZ> voxelgrid;
	voxelgrid.setLeafSize(0.7f, 0.7f, 0.7f);
	voxelgrid.setInputCloud(input_cloud);
	voxelgrid.filter(*downsampled);
	input_cloud = downsampled;
    pcl::transformPointCloud(*input_cloud, *input_cloud, init_guess);
    if(is_initial){
        //pcl::io::savePCDFileASCII ("data/target_cloud.pcd", *input_cloud);
        *target_cloud = *input_cloud;
        is_initial = false;
        std::cout<<"initialed !!" << std::endl;
    }
    else{
        // if(pcl::io::loadPCDFile("data/target_cloud.pcd", *target_cloud)) {
        //     std::cerr << "failed to load " << std::endl;
        //     return;
        // }
        *target_cloud = *target_cloud + *input_cloud;
    }

    pub_targetcloud.publish( pcl2pointcloud(target_cloud, (char*)"start_of_service"));

    return;
}

void signalHandler(int signal){
    if(signal == SIGTSTP){
        pcl::io::savePCDFileASCII ("data/target_cloud.pcd", *target_cloud);  
        std::cout<<"Saved !!" << std::endl;
        pcl::PointCloud<pcl::PointXYZ>::Ptr visualize_cloud(new pcl::PointCloud<pcl::PointXYZ>());
        pcl::io::loadPCDFile("data/target_cloud.pcd", *visualize_cloud);
        // visualization
        pcl::visualization::PCLVisualizer vis("vis");
        pcl::visualization::PointCloudColorHandlerCustom<pcl::PointXYZ> visualize_handler(visualize_cloud, 255.0, 0.0, 0.0);
        vis.addPointCloud(visualize_cloud, visualize_handler, "target");
        vis.spin();
    }
}