#include "NDTMapping.h"

namespace NDT_MAPPING{
    void NDTMapping::param_initial(ros::NodeHandle &nh, ros::NodeHandle &privateHandle){
        std::cout << "NDTMapping initialization" << std::endl;
        privateHandle.param<std::string>("base_frame", param_base_frame, "");
        privateHandle.param<std::string>("laser_frame", param_laser_frame, "");
        if(param_base_frame == "" || param_laser_frame == ""){
			std::cout << "base_frame: " << param_base_frame << std::endl;
			std::cout << "laser_frame:  " << param_laser_frame << std::endl;
			ROS_ERROR("param: base_link & laser_link unset !");
			return;
		}
        privateHandle.param<double>("tf_timeout",param_tf_timeout,0.05);
		privateHandle.param<bool>("visualize",param_visualize,false);

		privateHandle.param<float>("ndt_resolution",ndt_res,1.0);
		privateHandle.param<double>("ndt_step_size",step_size,0.1);
		privateHandle.param<double>("ndt_trans_eps",trans_eps,0.01);
		privateHandle.param<int>("ndt_max_iter",max_iter,100);
		privateHandle.param<double>("voxel_leaf_size",voxel_leaf_size,0.5);
		privateHandle.param<double>("min_scan_range",min_scan_range,0.01);
		privateHandle.param<double>("max_scan_range",max_scan_range,0.01);
		privateHandle.param<double>("min_add_scan_shift",min_add_scan_shift,200.0);

		privateHandle.param<double>("global_voxel_leafsize",param_global_voxel_leafsize,1.0);

		privateHandle.param<double>("min_update_target_map",param_min_update_target_map,0.);
		privateHandle.param<double>("extract_length",param_extract_length,0.);
		privateHandle.param<double>("extract_width",param_extract_width,0.);
        
        if(param_min_update_target_map == 0. || param_extract_length == 0. || param_extract_width == 0.){
			ROS_ERROR("min_update_target extract_length and width unset !");
			return;
		}

        std::cout << "min_update_target_map:" << param_min_update_target_map << std::endl;
		std::cout << "extract_length: " << param_extract_length << std::endl;
		std::cout << "extract_width: " << param_extract_width << std::endl;
        std::cout << std::endl;

        std::cout << "ndt_res: " << ndt_res << std::endl;
		std::cout << "step_size: " << step_size << std::endl;
		std::cout << "trans_epsilon: " << trans_eps << std::endl;
		std::cout << "max_iter: " << max_iter << std::endl;
		std::cout << "voxel_leaf_size: " << voxel_leaf_size << std::endl;
		std::cout << "min_scan_range: " << min_scan_range << std::endl;
		std::cout << "max_scan_range: " << max_scan_range << std::endl;
		std::cout << "min_add_scan_shift: " << min_add_scan_shift << std::endl;
		std::cout << std::endl;

		privateHandle.param<bool>("use_imu",_use_imu, false);
		privateHandle.param<bool>("use_odom",_use_odom,false);
		privateHandle.param<bool>("imu_upside_down",_imu_upside_down, false);

		std::cout << "use imu: " << _use_imu << std::endl;
		std::cout << "use_odom: " << _use_odom << std::endl;
		std::cout << "reverse imu: " << _imu_upside_down << std::endl;
		std::cout << std::endl;

		privateHandle.param<std::string>("imu_topic",_imu_topic,"/imu_raw");
		privateHandle.param<std::string>("odom_topic",_odom_topic,"/odom_raw");
		privateHandle.param<std::string>("lidar_topic",_lidar_topic,"/velodyne_points");

		std::cout << "imu topic: " << _imu_topic << std::endl;
		std::cout << "odom topic: " << _odom_topic << std::endl;
		std::cout << "lidar topic: " << _lidar_topic << std::endl;
		std::cout << std::endl;

        int method_type;
		privateHandle.param<int>("ndt_method_type",method_type,1);
		_method_type = static_cast<MethodType>(method_type);

		switch (method_type) {
			case 0:
				std::cout << ">> Use PCL NDT <<" << std::endl;
				break;
			case 1:
				std::cout << ">> Use CPU NDT <<" << std::endl;
				break;
			case 2:
				std::cout << ">> Use GPU NDT <<" << std::endl;
				break;
			case 3:
				std::cout << ">> Use OMP NDT <<" << std::endl;
				break;
			default:
				ROS_ERROR("Invalid method type of NDT");
				exit(1);
		}
		std::cout << std::endl;
        tf::StampedTransform transform;
		try{
			ros::Time now = ros::Time::now();
			ROS_INFO("now:%f listen from static_tf and set tf_btol");
			tf_listener.waitForTransform(param_base_frame,param_laser_frame,ros::Time(0),ros::Duration(param_tf_timeout * 10), ros::Duration(param_tf_timeout / 3));
			tf_listener.lookupTransform(param_base_frame,param_laser_frame,ros::Time(0),transform);
			ROS_INFO("success listen from tf");
		}
		catch(const tf::TransformException &ex){
			ROS_ERROR("Error waiting for tf in init: %s",ex.what());
			return;
		}
        
        Eigen::Translation3f tl_btol(transform.getOrigin().getX(),transform.getOrigin().getY(),transform.getOrigin().getZ());
		double roll,pitch,yaw;
		Eigen::AngleAxisf rot_x_btol(roll, Eigen::Vector3f::UnitX());
		Eigen::AngleAxisf rot_y_btol(pitch, Eigen::Vector3f::UnitY());
		Eigen::AngleAxisf rot_z_btol(yaw, Eigen::Vector3f::UnitZ());
		tf_btol = (tl_btol * rot_z_btol * rot_y_btol * rot_x_btol).matrix();
		tf_ltob = tf_btol.inverse();

        previous_pose.x = 0.0;
		previous_pose.y = 0.0;
		previous_pose.z = 0.0;
		previous_pose.roll = 0.0;
		previous_pose.pitch = 0.0;
		previous_pose.yaw = 0.0;

		ndt_pose.x = 0.0;
		ndt_pose.y = 0.0;
		ndt_pose.z = 0.0;
		ndt_pose.roll = 0.0;
		ndt_pose.pitch = 0.0;
		ndt_pose.yaw = 0.0;

		current_pose.x = 0.0;
		current_pose.y = 0.0;
		current_pose.z = 0.0;
		current_pose.roll = 0.0;
		current_pose.pitch = 0.0;
		current_pose.yaw = 0.0;

		current_pose_imu.x = 0.0;
		current_pose_imu.y = 0.0;
		current_pose_imu.z = 0.0;
		current_pose_imu.roll = 0.0;
		current_pose_imu.pitch = 0.0;
		current_pose_imu.yaw = 0.0;

		guess_pose.x = 0.0;
		guess_pose.y = 0.0;
		guess_pose.z = 0.0;
		guess_pose.roll = 0.0;
		guess_pose.pitch = 0.0;
		guess_pose.yaw = 0.0;

		added_pose.x = 0.0;
		added_pose.y = 0.0;
		added_pose.z = 0.0;
		added_pose.roll = 0.0;
		added_pose.pitch = 0.0;
		added_pose.yaw = 0.0;

		diff_x = 0.0;
		diff_y = 0.0;
		diff_z = 0.0;
		diff_yaw = 0.0;

		offset_imu_x = 0.0;
		offset_imu_y = 0.0;
		offset_imu_z = 0.0;
		offset_imu_roll = 0.0;
		offset_imu_pitch = 0.0;
		offset_imu_yaw = 0.0;

		offset_odom_x = 0.0;
		offset_odom_y = 0.0;
		offset_odom_z = 0.0;
		offset_odom_roll = 0.0;
		offset_odom_pitch = 0.0;
		offset_odom_yaw = 0.0;

		offset_imu_odom_x = 0.0;
		offset_imu_odom_y = 0.0;
		offset_imu_odom_z = 0.0;
		offset_imu_odom_roll = 0.0;
		offset_imu_odom_pitch = 0.0;
		offset_imu_odom_yaw = 0.0;

        if(_method_type == MethodType::use_pcl){
			pcl_ndt.setTransformationEpsilon(trans_eps);
			pcl_ndt.setStepSize(step_size);
			pcl_ndt.setResolution(ndt_res);
			pcl_ndt.setMaximumIterations(max_iter);
			// pcl_ndt.setInputSource(filtered_scan_ptr);
		}
		// else if (_method_type == MethodType::use_cpu){
		// 	cpu_ndt.setTransformationEpsilon(trans_eps);
		// 	cpu_ndt.setStepSize(step_size);
		// 	cpu_ndt.setResolution(ndt_res);
		// 	cpu_ndt.setMaximumIterations(max_iter);
		// 	// cpu_ndt.setInputSource(filtered_scan_ptr);
		// }
		else if (_method_type == MethodType::use_gpu){
			gpu_ndt.setTransformationEpsilon(trans_eps);
			gpu_ndt.setStepSize(step_size);
			gpu_ndt.setResolution(ndt_res);
			gpu_ndt.setMaximumIterations(max_iter);
			// gpu_ndt.setInputSource(filtered_scan_ptr);
		}
		// else if (_method_type == MethodType::use_omp){
		// 	omp_ndt.setTransformationEpsilon(trans_eps);
		// 	omp_ndt.setStepSize(step_size);
		// 	omp_ndt.setResolution(ndt_res);
		// 	omp_ndt.setMaximumIterations(max_iter);
		// 	// omp_ndt.setInputSource(filtered_scan_ptr);
		// }
		else{
			ROS_ERROR("Please Define _method_type to conduct NDT");
			exit(1);
		}
    }
    void NDTMapping::imu_odom_calc(ros::Time current_time){

    }
    void NDTMapping::imu_calc(ros::Time current_time){

    }
    void NDTMapping::odom_calc(ros::Time current_time){

    }
    double NDTMapping::warpToPm(double a_num, const double a_max) {
		if(a_num >= a_max){
			a_num -= 2.0*a_max;
		}
		return a_num;
	}

	double NDTMapping::warpToPmPi(double a_angle_rad) {
		return warpToPm(a_angle_rad,M_PI);
	}

    void NDTMapping::imuUpSideDown(const sensor_msgs::Imu::Ptr input){

    }
    double NDTMapping::calcDiffForRadian(const double lhs_rad, const double rhs_rad){

    }
    void NDTMapping::imu_callback(const sensor_msgs::Imu::Ptr &input){
        //imu回调函数。
        
        // TODO: 关注static
        const ros::Time current_time  = input->header.stamp;
        static ros::Time previous_time = current_time;
        const double diff_time = (current_time - previous_time);

        //解析imu
        double imu_roll, imu_pitch, imu_yaw;
        tf::Quaternion imu_orientation;
        tf::quaternionMsgToTF(input->orientation, imu_orientation);
        tf::Matrix3x3(imu_orientation).getRPY(imu_roll, imu_pitch, imu_yaw);
        imu_roll = warpToPmPi(imu_roll);
        imu_pitch = warpToPmPi(imu_pitch);
		imu_yaw = warpToPmPi(imu_yaw);

        static double previous_imu_roll = imu_roll, previous_imu_pitch = imu_pitch, previous_imu_yaw = imu_yaw;
		const double diff_imu_roll = calcDiffForRadian(imu_roll, previous_imu_roll);
		const double diff_imu_pitch = calcDiffForRadian(imu_pitch, previous_imu_pitch);
		const double diff_imu_yaw = calcDiffForRadian(imu_yaw, previous_imu_yaw);
		//

		imu.header = input->header;
		imu.linear_acceleration.x = input->linear_acceleration.x;
        // 实际上我们没用用到y和z方向的加速度
		// imu.linear_acceleration.y = input->linear_acceleration.y;
		// imu.linear_acceleration.z = input->linear_acceleration.z;
		imu.linear_acceleration.y = 0;
		imu.linear_acceleration.z = 0;
        
        if (diff_time != 0)
		{
			imu.angular_velocity.x = diff_imu_roll / diff_time;
			imu.angular_velocity.y = diff_imu_pitch / diff_time;
			imu.angular_velocity.z = diff_imu_yaw / diff_time;
		}
		else
		{
			imu.angular_velocity.x = 0;
			imu.angular_velocity.y = 0;
			imu.angular_velocity.z = 0;
		}

		imu_calc(input->header.stamp);
        // static
		previous_time = current_time;
		previous_imu_roll = imu_roll;
		previous_imu_pitch = imu_pitch;
		previous_imu_yaw = imu_yaw;
    }

    void NDTMapping::odom_callback(const nav_msgs::Odometry::ConstPtr &input){

    }

    void NDTMapping::points_callback(const sensor_msgs::PointCloud2::ConstPtr &input){
        double r;
		pcl::PointXYZI p;
		pcl::PointCloud<pcl::PointXYZI> tmp, scan;
		pcl::PointCloud<pcl::PointXYZI>::Ptr filtered_scan_ptr(new pcl::PointCloud<pcl::PointXYZI>());
		pcl::PointCloud<pcl::PointXYZI>::Ptr transformed_scan_ptr(new pcl::PointCloud<pcl::PointXYZI>());
		tf::Quaternion q;

		Eigen::Matrix4f t_localizer(Eigen::Matrix4f::Identity()); 
		Eigen::Matrix4f t_base_link(Eigen::Matrix4f::Identity()); 

        static tf::TransformBroadcaster br;  
		tf::Transform transform;

        current_scan_time = input->header.stamp;
		pcl::fromROSMsg(*input,tmp);

        for (auto point:tmp.points){
			r = std::sqrt(pow(point.x,2.0) + pow(point.y,2.0));
			if (min_scan_range < r && r < max_scan_range){
				scan.points.push_back(point);
			}
		}

        pcl::PointCloud<pcl::PointXYZI>::Ptr scan_ptr(new pcl::PointCloud<pcl::PointXYZI>(scan));  // scan保存到scan_ptr中
        if(initial_scan_loaded == 0){
			pcl::transformPointCloud(*scan_ptr,*transformed_scan_ptr,tf_btol);  // tf_btol为初始变换矩阵
			map += *transformed_scan_ptr;
			global_map += *transformed_scan_ptr;
			initial_scan_loaded = 1;
		}

        pcl::VoxelGrid<pcl::PointXYZI> voxel_grid_filter;
		voxel_grid_filter.setLeafSize(voxel_leaf_size, voxel_leaf_size, voxel_leaf_size);
		voxel_grid_filter.setInputCloud(scan_ptr);
		voxel_grid_filter.filter(*filtered_scan_ptr);

        pcl::PointCloud<pcl::PointXYZI>::Ptr map_ptr(new pcl::PointCloud<pcl::PointXYZI>(map));

        if(_method_type == MethodType::use_pcl){
			// pcl_ndt.setTransformationEpsilon(trans_eps);
			// pcl_ndt.setStepSize(step_size);
			// pcl_ndt.setResolution(ndt_res);
			// pcl_ndt.setMaximumIterations(max_iter);
			pcl_ndt.setInputSource(filtered_scan_ptr);
		}
		// else if (_method_type == MethodType::use_cpu){
		// 	// cpu_ndt.setTransformationEpsilon(trans_eps);
		// 	// cpu_ndt.setStepSize(step_size);
		// 	// cpu_ndt.setResolution(ndt_res);
		// 	// cpu_ndt.setMaximumIterations(max_iter);
		// 	cpu_ndt.setInputSource(filtered_scan_ptr);
		// }
		else if (_method_type == MethodType::use_gpu){
			// gpu_ndt.setTransformationEpsilon(trans_eps);
			// gpu_ndt.setStepSize(step_size);
			// gpu_ndt.setResolution(ndt_res);
			// gpu_ndt.setMaximumIterations(max_iter);
			gpu_ndt.setInputSource(filtered_scan_ptr);
		}
		// else if (_method_type == MethodType::use_omp){
		// 	// omp_ndt.setTransformationEpsilon(trans_eps);
		// 	// omp_ndt.setStepSize(step_size);
		// 	// omp_ndt.setResolution(ndt_res);
		// 	// omp_ndt.setMaximumIterations(max_iter);
		// 	omp_ndt.setInputSource(filtered_scan_ptr);
		// }
		else{
			ROS_ERROR("Please Define _method_type to conduct NDT");
			exit(1);
		}

        guess_pose.x = previous_pose.x + diff_x;  // 初始时diff_x等都为0
		guess_pose.y = previous_pose.y + diff_y;
		guess_pose.z = previous_pose.z + diff_z;
		guess_pose.roll = previous_pose.roll;
		guess_pose.pitch = previous_pose.pitch;
		guess_pose.yaw = previous_pose.yaw + diff_yaw;

        // 根据是否使用imu和odom,按照不同方式更新guess_pose(xyz,or/and rpy)
		if (_use_imu && _use_odom)
			imu_odom_calc(current_scan_time);
		if (_use_imu && !_use_odom)
			imu_calc(current_scan_time);
		if (!_use_imu && _use_odom)
			odom_calc(current_scan_time);

		// start2 只是为了把上面不同方式的guess_pose都标准化成guess_pose_for_ndt,为了后续操作方便
		pose guess_pose_for_ndt;
		if (_use_imu && _use_odom)
			guess_pose_for_ndt = guess_pose_imu_odom;
		else if (_use_imu && !_use_odom)
			guess_pose_for_ndt = guess_pose_imu;
		else if (!_use_imu && _use_odom)
			guess_pose_for_ndt = guess_pose_odom;
		else
			guess_pose_for_ndt = guess_pose;

        Eigen::AngleAxisf init_rotation_x(static_cast<const float &>(guess_pose_for_ndt.roll), Eigen::Vector3f::UnitX());
		Eigen::AngleAxisf init_rotation_y(static_cast<const float &>(guess_pose_for_ndt.pitch), Eigen::Vector3f::UnitY());
		Eigen::AngleAxisf init_rotation_z(static_cast<const float &>(guess_pose_for_ndt.yaw), Eigen::Vector3f::UnitZ());

		Eigen::Translation3f init_translation(static_cast<const float &>(guess_pose_for_ndt.x),
																					static_cast<const float &>(guess_pose_for_ndt.y),
																					static_cast<const float &>(guess_pose_for_ndt.z));

		Eigen::Matrix4f init_guess =
				(init_translation * init_rotation_z * init_rotation_y * init_rotation_x).matrix() * tf_btol;  // tf_btol

        pcl::PointCloud<pcl::PointXYZI>::Ptr output_cloud(new pcl::PointCloud<pcl::PointXYZI>);
        if (_method_type == MethodType::use_pcl)
		{
			pcl_ndt.align(*output_cloud, init_guess);  // pcl::aligin 需传入转换后的点云(容器),估计变换
			fitness_score = pcl_ndt.getFitnessScore();
			t_localizer = pcl_ndt.getFinalTransformation();  // t_localizer为ndt变换得到的最终变换矩阵(即source和target之间的变换)
			has_converged = pcl_ndt.hasConverged();
			final_num_iteration = pcl_ndt.getFinalNumIteration();
			transformation_probability = pcl_ndt.getTransformationProbability();
		}
		// else if (_method_type == MethodType::use_cpu)
		// {
		// 	cpu_ndt.align(init_guess);            // cpu::align 只需要传入估计变换 --建图的时候传入估计变换,定位matching的时候传入空的单位Eigen
		// 	fitness_score = cpu_ndt.getFitnessScore();
		// 	t_localizer = cpu_ndt.getFinalTransformation();
		// 	has_converged = cpu_ndt.hasConverged();
		// 	final_num_iteration = cpu_ndt.getFinalNumIteration();
		// }
		else if (_method_type == MethodType::use_gpu)
		{
			gpu_ndt.align(init_guess);  // ndt_gpu库的align,不传出配准后的点云 ---用法同cpu_ndt
			fitness_score = gpu_ndt.getFitnessScore();
			t_localizer = gpu_ndt.getFinalTransformation();
			has_converged = gpu_ndt.hasConverged();
			final_num_iteration = gpu_ndt.getFinalNumIteration();
		}
		// else if (_method_type == MethodType::use_omp)
  		// {
		// 	omp_ndt.align(*output_cloud, init_guess);   // omp_ndt.align用法同pcl::ndt
		// 	fitness_score = omp_ndt.getFitnessScore();
		// 	t_localizer = omp_ndt.getFinalTransformation();
		// 	has_converged = omp_ndt.hasConverged();
		// 	final_num_iteration = omp_ndt.getFinalNumIteration();
 	 	// }
        if(final_num_iteration > 20){
			ROS_ERROR("too much iteration !");
			return;
        }
        t_base_link = t_localizer * tf_ltob;
        pcl::transformPointCloud(*scan_ptr, *transformed_scan_ptr, t_localizer);

    }
}