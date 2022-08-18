#include <ros/ros.h>
#include <pcl/io/pcd_io.h>
#include <pcl/point_types.h>
#include <pcl/point_cloud.h>
#include <pcl/registration/ndt.h>
#include <pcl/filters/voxel_grid.h>
#include <sensor_msgs/PointCloud2.h>
#include <tf/transform_listener.h>
#include <tf/transform_broadcaster.h>
#include <tf_conversions/tf_eigen.h>
#include <Eigen/Core>
#include <Eigen/Geometry>
#include <pclomp/ndt_omp.h>
#include <pcl_conversions/pcl_conversions.h>
#include <pcl_ros/transforms.h>
#include <pcl_ros/point_cloud.h>

sensor_msgs::PointCloud2 pcl2pointcloud (pcl::PointCloud<pcl::PointXYZ>::Ptr cloud, char *fix_frame);
void cloud_callback(const sensor_msgs::PointCloud2::ConstPtr& msg);
void create_map(char *map_dir);

pcl::PointCloud<pcl::PointXYZ>::Ptr target_cloud(new pcl::PointCloud<pcl::PointXYZ>());
pcl::PointCloud<pcl::PointXYZ>::Ptr source_cloud(new pcl::PointCloud<pcl::PointXYZ>());
pcl::PointCloud<pcl::PointXYZ>::Ptr transform_cloud(new pcl::PointCloud<pcl::PointXYZ>());
ros::Subscriber sub_sourcecloud;
ros::Publisher pub_targetcloud;
ros::Publisher pub_transformcloud;
pclomp::NormalDistributionsTransform<pcl::PointXYZ, pcl::PointXYZ>::Ptr ndt_omp(new pclomp::NormalDistributionsTransform<pcl::PointXYZ, pcl::PointXYZ>());


bool is_initial;
char map_frame[] = "start_of_service";
char odom_frame[] = "ndt_transformed";
char sub_cloud[] = "/tango/point_cloud";
char lidar_map_dir[] = "/home/ros20/Desktop/ndt_ws/src/ndt_odometry/map/target_cloud.pcd";
Eigen::Matrix4f pre_trans;
Eigen::Matrix4f delta_trans;
tf::StampedTransform transform_final;
tf::StampedTransform transform;
std::mutex mutex_;

int main(int argc, char** argv) {
    ros::init (argc, argv, "ndt_odometry");
    ros::NodeHandle nh;
    sub_sourcecloud = nh.subscribe (sub_cloud, 1, cloud_callback);
    pub_targetcloud = nh.advertise<sensor_msgs::PointCloud2> ("/target_cloud", 1);
    pub_transformcloud = nh.advertise<sensor_msgs::PointCloud2> ("/transform_cloud", 1);
    is_initial = true;
    tf::TransformBroadcaster broadcaster;
    tf::TransformListener listener;
    create_map(lidar_map_dir);
    ros::Rate rate(20.0);
    while (nh.ok()){
        if(is_initial) {
            try{
                const std::lock_guard<std::mutex> lock(mutex_);
                listener.waitForTransform(map_frame, "/camera_depth", ros::Time(0), ros::Duration(1.0) );
                listener.lookupTransform(map_frame, "/camera_depth", ros::Time(0), transform_final);                
                Eigen::Quaterniond init_rotation_Qua = Eigen::Quaterniond(transform_final.getRotation());
                Eigen::Translation3f init_translation (transform_final.getOrigin().x(), transform_final.getOrigin().y(), transform_final.getOrigin().z());
                Eigen::Matrix3f init_rotation = init_rotation_Qua.toRotationMatrix().cast <float> ();
                pre_trans = (init_translation * init_rotation).matrix ();
                delta_trans = Eigen::Matrix4f::Identity();
            } catch (tf::TransformException ex){
                ROS_WARN("%s",ex.what());
                ros::Duration(1.0).sleep();
                continue;
            }
            is_initial = false;
        }
        else {
            const std::lock_guard<std::mutex> lock(mutex_);
            broadcaster.sendTransform(tf::StampedTransform(transform_final, ros::Time::now(), map_frame, odom_frame));
        }
        rate.sleep();
        ros::spinOnce();
    }
	return 0;
}

sensor_msgs::PointCloud2 pcl2pointcloud (pcl::PointCloud<pcl::PointXYZ>::Ptr cloud, char *fix_frame) {
    sensor_msgs::PointCloud2 ros_cloud;
    pcl::toROSMsg(*cloud, ros_cloud); //convert from PCL to ROS type this way
    ros_cloud.header.frame_id = fix_frame;
    return ros_cloud;
}

void cloud_callback(const sensor_msgs::PointCloud2::ConstPtr& msg) {
    //tf parameter
    static tf::TransformListener listener;
    static tf::StampedTransform transform;
    // initialize
    if(!is_initial) {
        pcl::fromROSMsg (*msg, *source_cloud);
    
        //downsampling
        pcl::PointCloud<pcl::PointXYZ>::Ptr downsampled(new pcl::PointCloud<pcl::PointXYZ>());
        pcl::VoxelGrid<pcl::PointXYZ> voxelgrid;
        voxelgrid.setLeafSize(0.05f, 0.05f, 0.05f);
        voxelgrid.setInputCloud(source_cloud);
        voxelgrid.filter(*downsampled);
        source_cloud = downsampled;
        
        //Set initial alignment estimate found using linear predicted pose.
        Eigen::Matrix4f initial_pose_matrix;
        initial_pose_matrix = pre_trans * delta_trans;

        ndt_omp->setInputSource(source_cloud);	
        ndt_omp->setTransformationEpsilon (0.001);
        ndt_omp->setStepSize (0.001); 
        ndt_omp->setResolution(1.0);
        ndt_omp->setMaximumIterations (100);
        ndt_omp->setNumThreads(10);
        ndt_omp->setNeighborhoodSearchMethod(pclomp::DIRECT7);
        ndt_omp->align(*transform_cloud, initial_pose_matrix);
        ROS_INFO(" score: %f", ndt_omp->getFitnessScore());
        Eigen::Affine3f target_pose(ndt_omp->getFinalTransformation());
        if((fabs(target_pose(0,3))+fabs(target_pose(1,3))+fabs(target_pose(2,3))) > 0.2) {
            ROS_INFO(" transformed!");
            const std::lock_guard<std::mutex> lock(mutex_);
            tf::transformEigenToTF(target_pose.cast <double> (), transform_final);
            pub_transformcloud.publish( pcl2pointcloud(transform_cloud, map_frame));
            delta_trans = pre_trans.inverse() * target_pose.matrix();
            pre_trans = target_pose.matrix();
        }
    }
    // pub_targetcloud.publish( pcl2pointcloud(target_cloud, map_frame));
    return;
}

void create_map(char *map_dir)
{
    if(pcl::io::loadPCDFile(map_dir, *target_cloud)) {
        std::cerr << "failed to load " << std::endl;
        return;
    }
    pcl::PointCloud<pcl::PointXYZ>::Ptr downsampled(new pcl::PointCloud<pcl::PointXYZ>());
    pcl::VoxelGrid<pcl::PointXYZ> voxelgrid;
    voxelgrid.setLeafSize(0.05f, 0.05f, 0.05f);
    voxelgrid.setInputCloud(target_cloud);
    voxelgrid.filter(*downsampled);
    // lidar frame transform to camera frame
    Eigen::Affine3f r = Eigen::Affine3f::Identity();
    r.translation() << 0.0, 0.0, 0.0;
    r.rotate(Eigen::AngleAxisf(M_PI/2, Eigen::Vector3f::UnitZ()));
    pcl::transformPointCloud(*downsampled, *target_cloud, r);
    ROS_INFO("Loaded %d data points from target", target_cloud->size ());
    ndt_omp->setInputTarget(target_cloud);
    pub_targetcloud.publish( pcl2pointcloud(target_cloud, map_frame));
}