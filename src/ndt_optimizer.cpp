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
pcl::PointCloud<pcl::PointXYZ>::Ptr target_cloud(new pcl::PointCloud<pcl::PointXYZ>());
pcl::PointCloud<pcl::PointXYZ>::Ptr source_cloud(new pcl::PointCloud<pcl::PointXYZ>());
pcl::PointCloud<pcl::PointXYZ>::Ptr transform_cloud(new pcl::PointCloud<pcl::PointXYZ>());
bool TransformToMatrix(const tf::StampedTransform& stemp_transform, Eigen::Matrix4f& transform_matrix);
void create_map(void);
ros::Subscriber sub_sourcecloud;
ros::Subscriber sub_device_tf;
ros::Publisher pub_targetcloud;
ros::Publisher pub_transformcloud;

bool is_initial;
char map_frame[] = "map";
char odom_frame[] = "start_of_service";
char sensor_frame[] = "camera_depth";
char sub_cloud[] = "/tango/point_cloud";
tf::StampedTransform transform_final;
tf::StampedTransform transform;
std::mutex mutex_;

int main(int argc, char** argv) 
{
    ros::init (argc, argv, "ndt_odometry");
    ros::NodeHandle nh;
    sub_sourcecloud = nh.subscribe (sub_cloud, 1, cloud_callback);
    pub_targetcloud = nh.advertise<sensor_msgs::PointCloud2> ("/target_cloud", 1);
    pub_transformcloud = nh.advertise<sensor_msgs::PointCloud2> ("/transform_cloud", 1);
    is_initial = true;
    // ros::spin ();
    tf::TransformBroadcaster broadcaster;
    tf::TransformListener listener;
    transform_final.setOrigin( tf::Vector3(0.0, 0.0, 0.0) );
    tf::Quaternion q;
    q.setRPY(0, 0, -M_PI/2);
    transform_final.setRotation(q);
    create_map();
    ros::Rate rate(30.0);
    while (nh.ok()){
        // const std::lock_guard<std::mutex> lock(mutex_);
        broadcaster.sendTransform(tf::StampedTransform(transform_final, ros::Time::now(), map_frame, odom_frame));
        rate.sleep();
        ros::spinOnce();
    }
    
	return 0;
}

sensor_msgs::PointCloud2 pcl2pointcloud (pcl::PointCloud<pcl::PointXYZ>::Ptr cloud, char *fix_frame) 
{
    sensor_msgs::PointCloud2 ros_cloud;
    pcl::toROSMsg(*cloud, ros_cloud); //convert from PCL to ROS type this way
    ros_cloud.header.frame_id = fix_frame;
    return ros_cloud;
}

void cloud_callback(const sensor_msgs::PointCloud2::ConstPtr& msg) 
{
    //tf parameter
    // static tf::Transform transform_pose;
    //initialize
    if(is_initial) {
        // if(pcl::io::loadPCDFile("/home/ros20/Desktop/ndt_ws/src/ndt_odometry/map/target_cloud.pcd", *target_cloud)) {
        //     std::cerr << "failed to load " << std::endl;
        //     return;
        // }
        // pcl::PointCloud<pcl::PointXYZ>::Ptr downsampled(new pcl::PointCloud<pcl::PointXYZ>());
        // pcl::VoxelGrid<pcl::PointXYZ> voxelgrid;
        // voxelgrid.setLeafSize(0.05f, 0.05f, 0.05f);
        // voxelgrid.setInputCloud(target_cloud);
        // voxelgrid.filter(*downsampled);
        // target_cloud = downsampled;

    	// std::cout << "Loaded " << target_cloud->size () << " data points from target" << std::endl;
        // is_initial = false;
        // transform.setOrigin( tf::Vector3(0.0, 0.0, 0.0) );
        // tf::Quaternion q;
        // q.setRPY(0, 0, -M_PI/2);
        // transform.setRotation(q);
        // broadcaster.sendTransform(tf::StampedTransform(transform, ros::Time::now(), map_frame, odom_frame));

    }
    else{
        static tf::TransformBroadcaster broadcaster;
        static tf::TransformListener listener;
        pcl::fromROSMsg (*msg, *source_cloud);
    
        //downsampling
        pcl::PointCloud<pcl::PointXYZ>::Ptr downsampled(new pcl::PointCloud<pcl::PointXYZ>());
        pcl::VoxelGrid<pcl::PointXYZ> voxelgrid;
        voxelgrid.setLeafSize(0.1f, 0.1f, 0.1f);
        voxelgrid.setInputCloud(source_cloud);
        voxelgrid.filter(*downsampled);
        source_cloud = downsampled;
        
        //Set initial alignment estimate found using robot odometry.
        listener.waitForTransform(odom_frame, sensor_frame, ros::Time(0), ros::Duration(1.0));
        listener.lookupTransform(odom_frame, sensor_frame, ros::Time(0), transform);
        // Eigen::Quaterniond init_rotation_Qua = Eigen::Quaterniond(transform.getRotation());
        // Eigen::Translation3f init_translation (transform.getOrigin().x(), transform.getOrigin().y(), transform.getOrigin().z());
        // Eigen::Matrix3f init_rotation = init_rotation_Qua.toRotationMatrix().cast <float> ();
        // Eigen::Matrix4f init_guess = (init_translation * init_rotation).matrix ();
        Eigen::Matrix4f init_guess, base_frame;
        TransformToMatrix(transform, init_guess);
        TransformToMatrix(transform_final, base_frame);
        init_guess = init_guess * base_frame;
        pclomp::NormalDistributionsTransform<pcl::PointXYZ, pcl::PointXYZ>::Ptr ndt_omp(new pclomp::NormalDistributionsTransform<pcl::PointXYZ, pcl::PointXYZ>());
        ndt_omp->setInputTarget(target_cloud);
        ndt_omp->setInputSource(source_cloud);	
        ndt_omp->setTransformationEpsilon (0.01);
        ndt_omp->setStepSize (0.1); 
        ndt_omp->setResolution(1.0);
        ndt_omp->setMaximumIterations (100);
        ndt_omp->setNumThreads(6);
        ndt_omp->setNeighborhoodSearchMethod(pclomp::DIRECT7);
        ndt_omp->align(*transform_cloud, init_guess);
        std::cout << " score: " << ndt_omp->getFitnessScore() << std::endl;

        Eigen::Affine3d target_pose(ndt_omp->getFinalTransformation().cast <double> ());
        if((fabs(target_pose(0,3))+fabs(target_pose(1,3))+fabs(target_pose(2,3))) > 0.4)        //transform pose
        {
            target_pose.matrix() = target_pose.matrix() * init_guess.inverse().cast <double> ();
            const std::lock_guard<std::mutex> lock(mutex_);
            tf::transformEigenToTF(target_pose, transform_final);
            // broadcaster.sendTransform(tf::StampedTransform(transform_pose, ros::Time::now(), map_frame, odom_frame));
            pub_transformcloud.publish( pcl2pointcloud(transform_cloud, map_frame));
        }

    }
    pub_targetcloud.publish(pcl2pointcloud(target_cloud, map_frame));
    return;
}

void create_map(void)
{
    if(pcl::io::loadPCDFile("/home/ros20/Desktop/ndt_ws/src/ndt_odometry/map/target_cloud.pcd", *target_cloud)) {
        std::cerr << "failed to load " << std::endl;
        return;
    }
    pcl::PointCloud<pcl::PointXYZ>::Ptr downsampled(new pcl::PointCloud<pcl::PointXYZ>());
    pcl::VoxelGrid<pcl::PointXYZ> voxelgrid;
    voxelgrid.setLeafSize(0.05f, 0.05f, 0.05f);
    voxelgrid.setInputCloud(target_cloud);
    voxelgrid.filter(*downsampled);
    target_cloud = downsampled;

    std::cout << "Loaded " << target_cloud->size () << " data points from target" << std::endl;
    is_initial = false;
}

bool TransformToMatrix(const tf::StampedTransform& stemp_transform, Eigen::Matrix4f& transform_matrix) 
{
    Eigen::Translation3f tl_btol(
    stemp_transform.getOrigin().getX(), 
    stemp_transform.getOrigin().getY(), 
    stemp_transform.getOrigin().getZ());
    double roll, pitch, yaw;
    tf::Matrix3x3(stemp_transform.getRotation()).getEulerYPR(yaw, pitch, roll);
    Eigen::AngleAxisf rot_x_btol(roll, Eigen::Vector3f::UnitX());
    Eigen::AngleAxisf rot_y_btol(pitch, Eigen::Vector3f::UnitY());
    Eigen::AngleAxisf rot_z_btol(yaw, Eigen::Vector3f::UnitZ());
    transform_matrix = (tl_btol * rot_z_btol * rot_y_btol * rot_x_btol).matrix();
    return true;
}