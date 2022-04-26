#include <ros/ros.h>
#include <pcl/io/pcd_io.h>
#include <pcl/point_types.h>
#include <pcl/point_cloud.h>
#include <pcl/registration/ndt.h>
#include <pcl/filters/voxel_grid.h>
#include <sensor_msgs/PointCloud2.h>
#include <tf/transform_listener.h>
#include <tf/transform_broadcaster.h>
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
ros::Subscriber sub_sourcecloud;
ros::Subscriber sub_device_tf;
ros::Publisher pub_targetcloud;
ros::Publisher pub_transformcloud;

bool is_initial;

int main(int argc, char** argv) {
    ros::init (argc, argv, "ndt_odometry");
    ros::NodeHandle nh;
    sub_sourcecloud = nh.subscribe ("/tango/point_cloud", 1, cloud_callback);
    pub_targetcloud = nh.advertise<sensor_msgs::PointCloud2> ("/target_cloud", 1);
    pub_transformcloud = nh.advertise<sensor_msgs::PointCloud2> ("/transform_cloud", 1);
    is_initial = true;
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
    //tf parameter
    static tf::TransformBroadcaster broadcaster;
    static tf::TransformListener listener;
    static tf::StampedTransform transform;
    static tf::Transform transform_pose;
    //initialize
    if(is_initial) {
        if(pcl::io::loadPCDFile("/home/ros20/Desktop/ndt_ws/src/ndt_odometry/map/target_cloud.pcd", *target_cloud)) {
            std::cerr << "failed to load " << std::endl;
            return;
        }
        pcl::PointCloud<pcl::PointXYZ>::Ptr downsampled(new pcl::PointCloud<pcl::PointXYZ>());
        pcl::VoxelGrid<pcl::PointXYZ> voxelgrid;
        voxelgrid.setLeafSize(0.1f, 0.1f, 0.1f);
        voxelgrid.setInputCloud(target_cloud);
        voxelgrid.filter(*downsampled);
        target_cloud = downsampled;

    	std::cout << "Loaded " << target_cloud->size () << " data points from target" << std::endl;
        is_initial = false;
        listener.waitForTransform("/start_of_service", "/camera_depth", ros::Time(0), ros::Duration(1.0) );
        listener.lookupTransform("/start_of_service", "/camera_depth", ros::Time(0), transform);
        transform_pose.setOrigin( transform.getOrigin());
        transform_pose.setRotation( transform.getRotation());
        broadcaster.sendTransform(tf::StampedTransform(transform_pose, ros::Time::now(), "start_of_service", "ndt_transformed"));

    }
    else{
        pcl::fromROSMsg (*msg, *source_cloud);
    
        //downsampling
        pcl::PointCloud<pcl::PointXYZ>::Ptr downsampled(new pcl::PointCloud<pcl::PointXYZ>());
        
        pcl::VoxelGrid<pcl::PointXYZ> voxelgrid;
        voxelgrid.setLeafSize(0.1f, 0.1f, 0.1f);

        voxelgrid.setInputCloud(source_cloud);
        voxelgrid.filter(*downsampled);
        source_cloud = downsampled;
        
        //Set initial alignment estimate found using robot odometry.

        listener.waitForTransform("/start_of_service", "/ndt_transformed", ros::Time(0), ros::Duration(1.0) );
        listener.lookupTransform("/start_of_service", "/ndt_transformed", ros::Time(0), transform);
        Eigen::Quaterniond init_rotation_Qua = Eigen::Quaterniond(transform.getRotation());
        Eigen::Translation3f init_translation (transform.getOrigin().x(), transform.getOrigin().y(), transform.getOrigin().z());
        Eigen::Matrix3f init_rotation = init_rotation_Qua.toRotationMatrix().cast <float> ();
        Eigen::Matrix4f init_guess = (init_translation * init_rotation).matrix ();

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

        Eigen::Matrix4f target_pose= ndt_omp->getFinalTransformation();
        if((fabs(target_pose(0,3))+fabs(target_pose(1,3))+fabs(target_pose(2,3))) > 0.4)        //transform pose
        {
            Eigen::Matrix3f rotate_3f = target_pose.topLeftCorner(3,3);
            Eigen::Quaternionf rotate_q(rotate_3f);
            transform_pose.setOrigin( tf::Vector3(target_pose(0,3), target_pose(1,3), target_pose(2,3)) );
            transform_pose.setRotation(tf::Quaternion(rotate_q.x(), rotate_q.y(), rotate_q.z(), rotate_q.w()) );
            //transform_pose.setRotation( tf::Quaternion(0, 0, 0, 1) );
            broadcaster.sendTransform(tf::StampedTransform(transform_pose, ros::Time::now(), "start_of_service", "ndt_transformed"));

            pub_transformcloud.publish( pcl2pointcloud(transform_cloud, (char*)"start_of_service"));
        }

        pub_targetcloud.publish( pcl2pointcloud(target_cloud, (char*)"start_of_service"));
    }
    return;
}

