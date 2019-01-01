#include "LidarMapping.h"

namespace NDT {

    void LidarMapping::setup(ros::NodeHandle &handle){
        // init
        handle.param<double>("tf_x", current_pose.x, 0);
        handle.param<double>("tf_y", current_pose.y, 0);
        handle.param<double>("tf_z", current_pose.z, 0);
        handle.param<double>("tf_roll", current_pose.roll, 0);
        handle.param<double>("tf_pitch", current_pose.pitch, 0);
        handle.param<double>("tf_yaw", current_pose.yaw, 0);
        handle.param<double>("min_add_scan_shift", min_add_scan_shift, 1.0);
		handle.param<double>("region_move_shift", region_move_shift, 20.0);
		handle.param<double>("region_x_length", region_x_length, 100.0);
		handle.param<double>("region_y_length", region_y_length, 100.0);

        ROS_INFO("tf_x, tf_y, tf_z, tf_roll, tf_pitch, tf_yaw (%.4f, %.4f, %.4f, %.4f, %.4f, %.4f)", current_pose.x, current_pose.y, current_pose.z, current_pose.roll, current_pose.pitch, current_pose.yaw);
        globalMap->pose = current_pose;
        localMap->pose = current_pose;
        ndt_map_pub = handle.advertise<sensor_msgs::PointCloud2>("ndt/map",10000);
        current_pose_pub = handle.advertise<sensor_msgs::PointCloud2>("/current_pose", 1000);

        points_sub = handle.subscribe("/velodyne_points", 100000, &LidarMapping::points_callback, this);

    }

    void LidarMapping::points_callback(const sensor_msgs::PointCloud2::ConstPtr &input_cloud){
        pcl::PointCloud<PointT> scan, tmp;
        pcl::PointCloud<PointT>::Ptr filtered_scan_ptr(new pcl::PointCloud<PointT>());
        pcl::PointCloud<PointT>::Ptr transformed_scan_ptr(new pcl::PointCloud<PointT>);
        PointT p;
        
        pcl::fromROSMsg(*input_cloud, tmp);

        for(pcl::PointCloud<PointT>::const_iterator item=tmp.begin(); item !=tmp.end(); item++){
            p.x = static_cast<float>((double) item->x);
            p.y = static_cast<float>((double) item->y);
            p.z = static_cast<float>((double) item->z);
            double r = std::sqrt(p.x*p.x + p.y*p.y);
            if (min_scan_range < r && r < max_scan_range){
                scan.push_back(p);
            }
        }
        //初始化地图
        if(!map_initialed){
            pcl::transformPointCloud(scan, *transformed_scan_ptr, current_pose.rotateRPY());
            *(localMap->map_ptr) += scan;

            #ifdef CUDA_FOUND
                gpu_ndt_ptr->setInputTarget(localMap->map_ptr);
            #else
                pcl_ndt.setInputTarget(localMap->map_ptr);
            #endif

            map_initialed = true;
            added_pose = localMap->pose;
            return;
        }
        //过滤地图
        pcl::VoxelGrid<PointT> voxel_grid_filter;
        voxel_grid_filter.setLeafSize(static_cast<float>(voxel_leaf_size),
                                        static_cast<float>(voxel_leaf_size),
                                        static_cast<float>(voxel_leaf_size));

        pcl::PointCloud<PointT>::Ptr scan_ptr(new pcl::PointCloud<PointT>(scan));
        voxel_grid_filter.setInputCloud(scan_ptr);
        voxel_grid_filter.filter(*filtered_scan_ptr);

        #ifdef CUDA_FOUND
            gpu_ndt_ptr->setTransformationEpsilon(trans_eps);
            gpu_ndt_ptr->setMaximumIterations(maxIter);
            gpu_ndt_ptr->setStepSize(step_size);
            gpu_ndt_ptr->setResolution(ndt_res);
            gpu_ndt_ptr->setInputSource(filtered_scan_ptr);
        #else
            pcl_ndt.setTransformationEpsilon(trans_eps);
            pcl_ndt.setMaximumIterations(maxIter);
            pcl_ndt.setStepSize(step_size);
            pcl_ndt.setResolution(ndt_res);
            pcl_ndt.setInputSource(filtered_scan_ptr);
        #endif

        //简化guess
        guess_pose.x = current_pose.x;
		guess_pose.y = current_pose.y;
		guess_pose.z = current_pose.z;
		guess_pose.roll = current_pose.roll;
		guess_pose.pitch = current_pose.pitch;
		guess_pose.yaw = current_pose.yaw;

		Eigen::Matrix4f init_guess = guess_pose.rotateRPY();
        pcl::PointCloud<PointT>::Ptr output_cloud(new pcl::PointCloud<PointT>());

        //用ndt计算，得到tranform矩阵，就可以把得到的这帧点云(scan)变换到transformed_scan_ptr了
        double t1 = ros::Time::now().toNSec();
        #ifdef CUDA_FOUND
            //align函数是ndt中的计算，因为前面已经setInputSource和setInputTarget了
            gpu_ndt_ptr->align(init_guess);
            double fitness_score = gpu_ndt_ptr->getFitnessScore();
            Eigen::Matrix4f finalTrans = gpu_ndt_ptr->getFinalTransformation();
            bool has_converged = gpu_ndt_ptr->hasConverged();
            int final_num_iteration = gpu_ndt_ptr->getFinalNumIteration();
        #else
            pcl_ndt.align(*output_cloud, init_guess);
            double fitness_score = pcl_ndt.getFitnessScore();
            Eigen::Matrix4f finalTrans = pcl_ndt.getFinalTransformation();
            bool has_converged = pcl_ndt.hasConverged();
            int final_num_iteration = pcl_ndt.getFinalNumIteration();
        #endif

        pcl::transformPointCloud(scan, *transformed_scan_ptr, finalTrans);
        double t2 = ros::Time::now().toNSec();

        //
        tf::Matrix3x3 mat_l;
        mat_l.setValue(static_cast<double >(finalTrans(0, 0)), static_cast<double >(finalTrans(0, 1)), static_cast<double >(finalTrans(0, 2)),
						static_cast<double >(finalTrans(1, 0)), static_cast<double >(finalTrans(1, 1)), static_cast<double >(finalTrans(1, 2)),
						static_cast<double >(finalTrans(2, 0)), static_cast<double >(finalTrans(2, 1)), static_cast<double >(finalTrans(2, 2)));
        double _tx = finalTrans(0, 3);
        double _ty = finalTrans(1, 3);
        double _tz = finalTrans(2, 3);

        Pose _current_pose;
        _current_pose.init();
        _current_pose.x = _tx;
        _current_pose.y = _ty;
        _current_pose.z = _tz;
        mat_l.getRPY(_current_pose.roll, _current_pose.pitch, _current_pose.yaw);

        //判断这一帧加入到local map中
        double shift = sqrt(pow(_tx - added_pose.x, 2.0) + pow(_ty - added_pose.y, 2.0));
        if (shift>=min_add_scan_shift){
            *(localMap->map_ptr) += *transformed_scan_ptr;

        //把新的这一帧点云当作新的target
        #ifdef CUDA_FOUND
            gpu_ndt_ptr->setInputTarget(localMap->map_ptr);
        #else
            pcl_ndt.setInputTarget(localMap->map_ptr);
        #endif
            added_pose = _current_pose;
        }
        current_pose = _current_pose;

        //extract sub-map from global map.
		shift = sqrt(pow(current_pose.x - localMap->pose.x, 2.0) + pow(current_pose.y - localMap->pose.y, 2.0));
		if (shift >= region_move_shift) {
			update_region_map();
		}

        ROS_INFO("sequence_number: %d", input_cloud->header.seq);
		ROS_INFO("used %.4f ms", (t2 - t1) / 1000000.);
		ROS_INFO("Number of Scan Points: %u", scan.size());
		ROS_INFO("Number of filtered scan points: %u", filtered_scan_ptr->size());
		ROS_INFO("transformed_scan_ptr: %u", transformed_scan_ptr->size());
		ROS_INFO("local map: %u", localMap->map_ptr->points.size());
		ROS_INFO("NDT has converged %u", has_converged);
		ROS_INFO("Fitness score: %u", fitness_score);
		ROS_INFO("Number of Iterations %d", final_num_iteration);
		ROS_INFO("(x, y, z, roll, pitch, yaw): (%.4f, %.4f, %.4f, %.4f, %.4f, %.4f)", current_pose.x, current_pose.y, current_pose.z, current_pose.roll, current_pose.pitch, current_pose.yaw);
		ROS_INFO("Shift %.4f", shift);

        // 定义并发布位置Pose
        geometry_msgs::PoseStamped pose;
		pose.pose.position.x = current_pose.x;
		pose.pose.position.y = current_pose.y;
		pose.pose.position.z = current_pose.z;
		tf::Quaternion q;
		q.setRPY(current_pose.roll, current_pose.pitch, current_pose.yaw);
		pose.pose.orientation.x = q.x();
		pose.pose.orientation.y = q.y();
		pose.pose.orientation.z = q.z();
		pose.pose.orientation.w = q.w();
		pose.header.frame_id="map";
    }

    void LidarMapping::update_region_map() {
		*(globalMap->map_ptr) += *(localMap->map_ptr);
		double min_x = current_pose.x - region_x_length / 2.0;
		double max_x = current_pose.x + region_x_length / 2.0;
		double min_y = current_pose.y - region_y_length / 2.0;
		double max_y = current_pose.y + region_y_length / 2.0;
		pcl::PointCloud<PointT >::Ptr localMapPtr(new pcl::PointCloud<PointT>);

		// 该处在从global中抽取点云的时候,需要遍历所有的点,当globalMap越来越大,是否会再此处造成速度越来越慢??
		// 注:但该处的速度降低不是导致NDT效率降低的原因,因为我们统计的NDT使用时间不包括这一块
		for (int i = 0; i < globalMap->map_ptr->points.size(); ++i) {
			PointT& p = globalMap->map_ptr->points[i];
			if (p.x >= min_x && p.x <= max_x && p.y >= min_y && p.y <= max_y) {
				localMapPtr->points.push_back(p);
			}
		}
		// filter
		pcl::VoxelGrid<PointT> voxel_grid_filter;
		voxel_grid_filter.setLeafSize(static_cast<float>(voxel_leaf_size), static_cast<float>(voxel_leaf_size),
																	static_cast<float>(voxel_leaf_size));
		voxel_grid_filter.setInputCloud(localMapPtr);
		voxel_grid_filter.filter(*localMap->map_ptr);
		localMap->pose = current_pose;
	}
}