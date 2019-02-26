#ifndef NDT_MAPPING_H
#define NDT_MAPPING_H

#include <eigen3/Eigen/Eigen>
#include <sensor_msgs/PointCloud2.h>
#include <geometry_msgs/PoseStamped.h>
#include <tf/transform_datatypes.h>

#include <ctime>
#include <pcl/point_cloud.h>
#include <ros/ros.h>
#include <pcl/point_types.h>
#include <pcl_conversions/pcl_conversions.h>
#include <pcl/filters/filter.h>
#include <pcl/filters/conditional_removal.h>
#include <pcl/io/io.h>
#include <pcl/io/pcd_io.h>
#include <pcl/registration/ndt.h>

#ifdef CUDA_FOUND
#include <ndt_gpu/NormalDistributionsTransform.h>
#endif


namespace NDTMapping{
  using PointT = pcl::PointXYZ;

  struct Pose{
    double x;
    double y;
    double z;
    double roll;
    double pitch;
    double yaw;
    void init(){
      x = y = z = 0;
      roll = pitch = yaw = 0;
    }
    Eigen::Matrix4f rotateRPY(){
      Eigen::Translation3f tf_trans(x,y,z);
      Eigen::AngleAxisf rot_x(roll, Eigen::Vector3f::UnitX());
      Eigen::AngleAxisf rot_y(pitch, Eigen::Vector3f::UnitY());
      Eigen::AngleAxisf rot_z(yaw, Eigen::Vector3f::UnitZ());
      Eigen::Matrix4f mat = (tf_trans*rot_z*rot_y*rot_x).matrix();
      return mat;
    }
    };

  struct LidarMap{
    typedef boost::shared_ptr<LidarMap> Ptr;
    Pose pose;
    pcl::PointCloud<PointT>::Ptr map_ptr;
    LidarMap():map_ptr(new pcl::PointCloud<PointT>){
      pose.init();
    }
  };

  class NDTMapping{
    public:
    NDTMapping(): globalMap(new LidarMap()), localMap(new LidarMap()){
      // 初始化参数，如果launch文件中有参数初始值，在这里其实可以覆盖掉
      maxIter = 30;
      ndt_res = 1.0;
      step_size = 0.1;
      trans_eps = 0.01;
      voxel_leaf_size = 0.1;

      min_add_scan_shift = 1;
      min_scan_range = 5.0;
      max_scan_range = 200.0;
      map_initialed = false;

      previous_pose.init();
      guess_pose.init();
      current_pose.init();
      added_pose.init();
      pubCounter = 10;
      
    }
    
    void setup(ros::NodeHandle &handle);
    void points_callback(const sensor_msgs::PointCloud2::ConstPtr &input_cloud);

    protected:

    void update_region_map();

    private:
    LidarMap::Ptr globalMap;
    LidarMap::Ptr localMap;

    #ifdef CUDA_FOUND
    gpu::GNormalDistributionsTransform gpu_ndt;
    std::shared_ptr<gpu::GNormalDistributionsTransform> gpu_ndt_ptr = std::make_shared<gpu::GNormalDistributionsTransform>();
    #else
    pcl::NormalDistributionsTransform<PointT, PointT> pcl_ndt;
    #endif

    Pose previous_pose;
    Pose guess_pose;
    Pose current_pose;
    Pose added_pose;

    int maxIter;
    float ndt_res;
    double step_size;
    double trans_eps;
    double voxel_leaf_size;
    double min_add_scan_shift;
    double region_move_shift;
    double min_scan_range;
    double max_scan_range;
    bool map_initialed;
    double region_x_length;
    double region_y_length;
    std::string lidar_topic;
    std::string lidar_frame;

    int pubCounter;

    ros::Publisher current_pose_pub;
    ros::Publisher ndt_map_pub;
    
    ros::Subscriber points_sub;
    ros::Subscriber output_sub;

  };

}
#endif
