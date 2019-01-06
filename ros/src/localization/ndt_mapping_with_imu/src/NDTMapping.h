#ifndef NDT_MAPPING_H
#define NDT_MAPPING_H

#include <ros/ros.h>
#include <iostream>
#include <fstream>
#include <sstream>
#include <string>
#include <pthread.h>

#include <nav_msgs/Odometry.h>
#include <sensor_msgs/Imu.h>
#include <sensor_msgs/PointCloud2.h>
#include <std_msgs/Float32.h>
#include <std_msgs/Bool.h>

#include <tf/transform_broadcaster.h>
#include <tf/transform_listener.h>
#include <tf/transform_datatypes.h>

#include <pcl/io/io.h>
#include <pcl/io/pcd_io.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl_conversions/pcl_conversions.h>
#include <pcl/registration/ndt.h>
#include <ctime>
#ifdef CUDA_FOUND
#include <ndt_gpu/NormalDistributionsTransform.h>
#endif

namespace NDT_MAPPING{
    using PointI = pcl::PointXYZI;
    //定义pose这个结构，除了xyzrpy，还有由此得到的矩阵
    struct pose{
        double x;
		double y;
		double z;
		double roll;
		double pitch;
		double yaw;

		void init(){
			x = y = z = 0.0;
			roll = pitch = yaw = 0.0;
		}
        // TODO: 看一下Eigen库中这几个函数
        Eigen::Matrix4d rotateRPY(){
			Eigen::Translation3d tf_trans(x,y,z);
			Eigen::AngleAxisd rot_x(roll,Eigen::Vector3d::UnitX());
			Eigen::AngleAxisd rot_y(pitch,Eigen::Vector3d::UnitY());
			Eigen::AngleAxisd rot_z(yaw,Eigen::Vector3d::UnitZ());
			Eigen::Matrix4d mat=(tf_trans*rot_z*rot_y*rot_x).matrix();
			return mat;
		}
    };
    //定义使用flag，我只使用cpu, gpu和pcl自带的ndt，pcl自带的ndt非常慢
    enum class MethodType{
		use_pcl = 0,
		use_cpu = 1,
		use_gpu = 2,
		use_omp = 3,
		use_gpu_ptr = 4,
	};
    static MethodType _method_type = MethodType::use_cpu; // 默认使用cpu

    class NDTMapping{
    private: //数据放在private
        //Transform
        tf::TransformBroadcaster tf_broadcaster;
		tf::TransformListener tf_listener;
        double param_tf_timeout;
        std::string param_base_frame;
		std::string param_laser_frame;
        Eigen::Matrix4f tf_btol, tf_ltob;

        pose previous_pose,guess_pose,guess_pose_imu,guess_pose_odom,guess_pose_imu_odom;
		pose current_pose,current_pose_imu,current_pose_odom,current_pose_imu_odom;
		pose ndt_pose,localizer_pose;
        pose added_pose;

        // 定义Publisher
		ros::Publisher debug_map_pub;
		ros::Publisher matching_map_pub;        // 地图发布
		ros::Publisher refiltered_map_pub;
		ros::Publisher current_pose_pub;   // 位置发布

		ros::Subscriber points_sub;
		ros::Subscriber imu_sub;
		ros::Subscriber odom_sub;
		geometry_msgs::PoseStamped current_pose_msg, guess_pose_msg;

    };

}

#endif #NDT_MAPPING_H