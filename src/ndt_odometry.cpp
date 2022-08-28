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

pcl::PointCloud<pcl::PointXYZ>::Ptr transform_cloud(new pcl::PointCloud<pcl::PointXYZ>());
ros::Subscriber sub_sourcecloud;
ros::Publisher pub_targetcloud;
ros::Publisher pub_transformcloud;
pclomp::NormalDistributionsTransform<pcl::PointXYZ, pcl::PointXYZ>::Ptr ndt_omp(new pclomp::NormalDistributionsTransform<pcl::PointXYZ, pcl::PointXYZ>());


bool is_initial;
bool fusion_guess = false;
// char map_frame[] = "start_of_service";
char map_frame[] = "camera_init";
char odom_frame[] = "ndt_transformed";
char sub_cloud[] = "/tango/point_cloud";
char lidar_map_dir[] = "/home/ros20/Desktop/ndt_ws/src/visual_ndt_ros/map/lab_arround2.pcd";
Eigen::Matrix4f pre_trans, fusion_pre_trans;
Eigen::Matrix4f delta_trans;
tf::StampedTransform transform_final;
std::mutex mutex_;

class MatchPoint
{
  public:
  Eigen::Matrix4f transMatrix = Eigen::Matrix4f::Identity(4,4);
  pcl::PointCloud<pcl::PointXYZ>::Ptr cloud{new pcl::PointCloud<pcl::PointXYZ>};
};
std::vector<MatchPoint> multiPoint;

int main(int argc, char** argv) {
    ros::init (argc, argv, "ndt_odometry");
    ros::NodeHandle nh;
    sub_sourcecloud = nh.subscribe (sub_cloud, 1, cloud_callback);
    pub_targetcloud = nh.advertise<sensor_msgs::PointCloud2> ("/target_cloud", 1);
    pub_transformcloud = nh.advertise<sensor_msgs::PointCloud2> ("/transform_cloud", 1);
    is_initial = true;
    fusion_guess = true;
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
                fusion_pre_trans = pre_trans;
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
        pcl::PointCloud<pcl::PointXYZ>::Ptr curr_cloud(new pcl::PointCloud<pcl::PointXYZ>());
        pcl::PointCloud<pcl::PointXYZ>::Ptr source_cloud(new pcl::PointCloud<pcl::PointXYZ>());
        pcl::fromROSMsg (*msg, *curr_cloud);
        std::vector<int> indices;
        pcl::removeNaNFromPointCloud(*curr_cloud, *curr_cloud, indices);
        
        //Set initial alignment estimate found using linear predicted pose.
        Eigen::Matrix4f initial_pose_matrix;
        Eigen::Matrix4f curr_trans;
        if (fusion_guess) {
            try{
                listener.lookupTransform(map_frame, "/camera_depth", ros::Time(0), transform);
            } catch (tf::TransformException ex){
                ROS_WARN("%s",ex.what());
                return;
            }
            Eigen::Quaterniond init_rotation_Qua = Eigen::Quaterniond(transform.getRotation());
            Eigen::Translation3f init_translation (transform.getOrigin().x(), transform.getOrigin().y(), transform.getOrigin().z());
            Eigen::Matrix3f init_rotation = init_rotation_Qua.toRotationMatrix().cast <float> ();
            curr_trans = (init_translation * init_rotation).matrix ();
            delta_trans = fusion_pre_trans.inverse() * curr_trans;
            fusion_pre_trans = curr_trans;
        }
        initial_pose_matrix = pre_trans * delta_trans;
        pcl::PointCloud<pcl::PointXYZ>::Ptr tmp_cloud(new pcl::PointCloud<pcl::PointXYZ>());
        Eigen::Matrix4f curr_trans_inv = curr_trans.inverse();
        *source_cloud = *curr_cloud;
        for (int i=0;i<multiPoint.size();i++) {
            pcl::transformPointCloud(*multiPoint[i].cloud, *tmp_cloud, curr_trans_inv * multiPoint[i].transMatrix);
            *source_cloud += *tmp_cloud;
        }

        // downsampling
        pcl::PointCloud<pcl::PointXYZ>::Ptr downsampled(new pcl::PointCloud<pcl::PointXYZ>());
        pcl::VoxelGrid<pcl::PointXYZ> voxelgrid;
        voxelgrid.setLeafSize(0.05f, 0.05f, 0.05f);
        voxelgrid.setInputCloud(source_cloud);
        voxelgrid.filter(*downsampled);
        source_cloud = downsampled;

        ndt_omp->setInputSource(source_cloud);	
        ndt_omp->setTransformationEpsilon (0.01);
        ndt_omp->setStepSize (0.001); 
        ndt_omp->setResolution(1.0);
        ndt_omp->setMaximumIterations (35);
        ndt_omp->setNumThreads(10);
        ndt_omp->setNeighborhoodSearchMethod(pclomp::DIRECT7);
        ndt_omp->align(*transform_cloud, initial_pose_matrix);
        
        ROS_INFO(" score: %f", ndt_omp->getFitnessScore());
        Eigen::Affine3f target_pose(ndt_omp->getFinalTransformation());
        if ((fabs(target_pose(0,3) - pre_trans(0,3)) + 
             fabs(target_pose(1,3) - pre_trans(1,3)) +
             fabs(target_pose(2,3) - pre_trans(2,3))) > 0.05) {
            ROS_INFO(" transformed!");
            const std::lock_guard<std::mutex> lock(mutex_);
            tf::transformEigenToTF(target_pose.cast <double> (), transform_final);
            MatchPoint matchpoint;
            matchpoint.cloud->points.clear();
            matchpoint.cloud->points.assign(curr_cloud->points.begin(),curr_cloud->points.end()); 
            matchpoint.transMatrix = curr_trans;
            multiPoint.push_back(matchpoint);
            if(multiPoint.size()>10) 
                multiPoint.erase(std::begin(multiPoint));
        }
        else {
            target_pose.matrix() = initial_pose_matrix;
            const std::lock_guard<std::mutex> lock(mutex_);
            tf::transformEigenToTF(target_pose.cast <double> (), transform_final);
        }
        if (!fusion_guess)
            delta_trans = pre_trans.inverse() * target_pose.matrix();
        pre_trans = target_pose.matrix();
        pub_transformcloud.publish( pcl2pointcloud(source_cloud, odom_frame));
    }
    // pub_targetcloud.publish( pcl2pointcloud(target_cloud, map_frame));
    return;
}

void create_map(char *map_dir)
{
    pcl::PointCloud<pcl::PointXYZ>::Ptr target_cloud(new pcl::PointCloud<pcl::PointXYZ>());
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
    // Eigen::Affine3f r = Eigen::Affine3f::Identity();
    // r.translation() << 0.0, 0.0, 0.0;
    // r.rotate(Eigen::AngleAxisf(M_PI/2, Eigen::Vector3f::UnitZ()));
    // pcl::transformPointCloud(*downsampled, *target_cloud, r);
    target_cloud = downsampled;
    ROS_INFO("Loaded %zu data points from target", target_cloud->size ());
    ndt_omp->setInputTarget(target_cloud);
    pub_targetcloud.publish( pcl2pointcloud(target_cloud, map_frame));
}