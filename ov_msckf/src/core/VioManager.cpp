/*
 * OpenVINS: An Open Platform for Visual-Inertial Research
 * Copyright (C) 2019 Patrick Geneva
 * Copyright (C) 2019 Kevin Eckenhoff
 * Copyright (C) 2019 Guoquan Huang
 * Copyright (C) 2019 OpenVINS Contributors
 *
 * This program is free software: you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation, either version 3 of the License, or
 * (at your option) any later version.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with this program.  If not, see <https://www.gnu.org/licenses/>.
 */
#include "VioManager.h"
#include "types/Landmark.h"



using namespace ov_core;
using namespace ov_msckf;





VioManager::VioManager() {


    //===================================================================================
    //===================================================================================
    //===================================================================================

    // Load our state options
    StateOptions state_options;
    state_options.do_fej = true; // use_fej
    state_options.imu_avg = true; //use_imuavg
    state_options.use_rk4_integration = true; // use_rk4int
    state_options.do_calib_camera_pose =  true; // calib_cam_extrinsics
    state_options.do_calib_camera_intrinsics = true; // calib_cam_intrinsics
    state_options.do_calib_camera_timeoffset = true; // calib_cam_timeoffset
    state_options.max_clone_size = 11; // max_clones
    state_options.max_slam_features = 50; // max_slam
    state_options.max_aruco_features = 1024; // max_aruco
    state_options.num_cameras = 2; // max_cameras
    dt_statupdelay = 3; // dt_slam_delay

    // Enforce that if we are doing stereo tracking, we have two cameras
    if(state_options.num_cameras < 1) {
        ROS_ERROR("VioManager(): Specified number of cameras needs to be greater than zero");
        ROS_ERROR("VioManager(): num cameras = %d", state_options.num_cameras);
        std::exit(EXIT_FAILURE);
    }

    // Read in what representation our feature is
    std::string feat_rep_str;
    feat_rep_str = "ANCHORED_FULL_INVERSE_DEPTH"; // feat_representation
    std::transform(feat_rep_str.begin(), feat_rep_str.end(),feat_rep_str.begin(), ::toupper);

    // Set what representation we should be using
    if(feat_rep_str == "GLOBAL_3D") state_options.feat_representation = FeatureRepresentation::Representation::GLOBAL_3D;
    else if(feat_rep_str == "GLOBAL_FULL_INVERSE_DEPTH") state_options.feat_representation = FeatureRepresentation::Representation::GLOBAL_FULL_INVERSE_DEPTH;
    else if(feat_rep_str == "ANCHORED_3D") state_options.feat_representation = FeatureRepresentation::Representation::ANCHORED_3D;
    else if(feat_rep_str == "ANCHORED_FULL_INVERSE_DEPTH") state_options.feat_representation = FeatureRepresentation::Representation::ANCHORED_FULL_INVERSE_DEPTH;
    else if(feat_rep_str == "ANCHORED_MSCKF_INVERSE_DEPTH") state_options.feat_representation = FeatureRepresentation::Representation::ANCHORED_MSCKF_INVERSE_DEPTH;
    else {
        ROS_ERROR("VioManager(): invalid feature representation specified = %s", feat_rep_str.c_str());
        ROS_ERROR("VioManager(): the valid types are:");
        ROS_ERROR("\t- GLOBAL_3D");
        ROS_ERROR("\t- GLOBAL_FULL_INVERSE_DEPTH");
        ROS_ERROR("\t- ANCHORED_3D");
        ROS_ERROR("\t- ANCHORED_FULL_INVERSE_DEPTH");
        ROS_ERROR("\t- ANCHORED_MSCKF_INVERSE_DEPTH");
        std::exit(EXIT_FAILURE);
    }

    // Create the state!!
    state = new State(state_options);

    // Timeoffset from camera to IMU
    double calib_camimu_dt;
	calib_camimu_dt = 0.0; // calib_camimu_dt
    Eigen::VectorXd temp_camimu_dt;
    temp_camimu_dt.resize(1);
    temp_camimu_dt(0) = calib_camimu_dt;
    state->calib_dt_CAMtoIMU()->set_value(temp_camimu_dt);
    state->calib_dt_CAMtoIMU()->set_fej(temp_camimu_dt);

    // Global gravity
    Eigen::Matrix<double,3,1> gravity;
    std::vector<double> vec_gravity;
    std::vector<double> vec_gravity_default = {0.0,0.0,9.81};
	vec_gravity = vec_gravity_default; // gravity
    gravity << vec_gravity.at(0),vec_gravity.at(1),vec_gravity.at(2);

    // Debug, print to the console!
    ROS_INFO("FILTER PARAMETERS:");
    ROS_INFO("\t- do fej: %d", state_options.do_fej);
    ROS_INFO("\t- do imu avg: %d", state_options.imu_avg);
    ROS_INFO("\t- calibrate cam to imu: %d", state_options.do_calib_camera_pose);
    ROS_INFO("\t- calibrate cam intrinsics: %d", state_options.do_calib_camera_intrinsics);
    ROS_INFO("\t- calibrate cam imu timeoff: %d", state_options.do_calib_camera_timeoffset);
    ROS_INFO("\t- max clones: %d", state_options.max_clone_size);
    ROS_INFO("\t- max slam: %d", state_options.max_slam_features);
    ROS_INFO("\t- max aruco: %d", state_options.max_aruco_features);
    ROS_INFO("\t- max cameras: %d", state_options.num_cameras);
    ROS_INFO("\t- slam startup delay: %.1f", dt_statupdelay);
    ROS_INFO("\t- feature representation: %s", feat_rep_str.c_str());


    // Debug print initial values our state use
    ROS_INFO("STATE INIT VALUES:");
    ROS_INFO("\t- calib_camimu_dt: %.4f", calib_camimu_dt);
    ROS_INFO("\t- gravity: %.3f, %.3f, %.3f", vec_gravity.at(0), vec_gravity.at(1), vec_gravity.at(2));


    //===================================================================================
    //===================================================================================
    //===================================================================================

    // Debug message
    ROS_INFO("=====================================");
    ROS_INFO("CAMERA PARAMETERS:");

    // Loop through through, and load each of the cameras
    for(int i=0; i<state->options().num_cameras; i++) {

        // If our distortions are fisheye or not!
        bool is_fisheye;
		is_fisheye = false; // cami_is_fisheye
        state->get_model_CAM(i) = is_fisheye;

        // If the desired fov we should simulate
        std::vector<int> matrix_wh;
        std::vector<int> matrix_wd_default = {752,480};
		matrix_wh = matrix_wd_default; // cami_wh
        std::pair<int,int> wh(matrix_wh.at(0),matrix_wh.at(1));

        // Camera intrinsic properties
        Eigen::Matrix<double,8,1> cam_calib;
		if (i==0) {
			std::vector<double> matrix_k, matrix_d;
			std::vector<double> matrix_k_default = {458.654,457.296,367.215,248.375};
			std::vector<double> matrix_d_default = {-0.28340811,0.07395907,0.00019359,1.76187114e-05};
			matrix_k = matrix_k_default; // cami_k
			matrix_d = matrix_d_default; // cami_d
			cam_calib << matrix_k.at(0),matrix_k.at(1),matrix_k.at(2),matrix_k.at(3),matrix_d.at(0),matrix_d.at(1),matrix_d.at(2),matrix_d.at(3);
		} else if (i==1) {
			std::vector<double> matrix_k, matrix_d;
			std::vector<double> matrix_k_default = {457.587,456.134,379.999,255.238};
			std::vector<double> matrix_d_default = {-0.28368365,0.07451284,-0.00010473,-3.55590700e-05};
			matrix_k = matrix_k_default; // cami_k
			matrix_d = matrix_d_default; // cami_d
			cam_calib << matrix_k.at(0),matrix_k.at(1),matrix_k.at(2),matrix_k.at(3),matrix_d.at(0),matrix_d.at(1),matrix_d.at(2),matrix_d.at(3);
		} else {
			std::vector<double> matrix_k, matrix_d;
			std::vector<double> matrix_k_default = {458.654,457.296,367.215,248.375};
			std::vector<double> matrix_d_default = {-0.28340811,0.07395907,0.00019359,1.76187114e-05};
			matrix_k = matrix_k_default; // cami_k
			matrix_d = matrix_d_default; // cami_d
			cam_calib << matrix_k.at(0),matrix_k.at(1),matrix_k.at(2),matrix_k.at(3),matrix_d.at(0),matrix_d.at(1),matrix_d.at(2),matrix_d.at(3);
		}

        // Save this representation in our state
        state->get_intrinsics_CAM(i)->set_value(cam_calib);
        state->get_intrinsics_CAM(i)->set_fej(cam_calib);

        // Our camera extrinsics transform
        Eigen::Matrix4d T_CtoI;
        std::vector<double> matrix_TCtoI;
		std::vector<double> matrix_TtoI_default = {1,0,0,0,0,1,0,0,0,0,1,0,0,0,0,1};
        std::vector<double> matrix_TtoI_default_1 = {0.0148655429818, -0.999880929698, 0.00414029679422, -0.0216401454975, 0.999557249008, 0.0149672133247, 0.025715529948, -0.064676986768, -0.0257744366974, 0.00375618835797, 0.999660727178, 0.00981073058949, 0.0, 0.0, 0.0, 1.0};
		std::vector<double> matrix_TtoI_default_2 = {0.0125552670891, -0.999755099723, 0.0182237714554, -0.0198435579556, 0.999598781151, 0.0130119051815, 0.0251588363115, 0.0453689425024, -0.0253898008918, 0.0179005838253, 0.999517347078, 0.00786212447038, 0.0, 0.0, 0.0, 1.0};

        // Read in from ROS, and save into our eigen mat
		if (i==0) {
			matrix_TCtoI = matrix_TtoI_default_1; // T_CitoI
		} else if (i == 1) {
			matrix_TCtoI = matrix_TtoI_default_2; // T_CitoI
		} else {
			matrix_TCtoI = matrix_TtoI_default; // T_CitoI
		}

		T_CtoI << matrix_TCtoI.at(0),matrix_TCtoI.at(1),matrix_TCtoI.at(2),matrix_TCtoI.at(3),
                matrix_TCtoI.at(4),matrix_TCtoI.at(5),matrix_TCtoI.at(6),matrix_TCtoI.at(7),
                matrix_TCtoI.at(8),matrix_TCtoI.at(9),matrix_TCtoI.at(10),matrix_TCtoI.at(11),
                matrix_TCtoI.at(12),matrix_TCtoI.at(13),matrix_TCtoI.at(14),matrix_TCtoI.at(15);

        // Load these into our state
        Eigen::Matrix<double,7,1> cam_eigen;
        cam_eigen.block(0,0,4,1) = rot_2_quat(T_CtoI.block(0,0,3,3).transpose());
        cam_eigen.block(4,0,3,1) = -T_CtoI.block(0,0,3,3).transpose()*T_CtoI.block(0,3,3,1);
        state->get_calib_IMUtoCAM(i)->set_value(cam_eigen);
        state->get_calib_IMUtoCAM(i)->set_fej(cam_eigen);

        // Append to our maps for our feature trackers
        camera_fisheye.insert({i,is_fisheye});
        camera_calib.insert({i,cam_calib});
        camera_wh.insert({i,wh});

        // Debug printing
        cout << "cam_" << i << "wh:" << endl << wh.first << " x " << wh.second << endl;
        cout << "cam_" << i << "K:" << endl << cam_calib.block(0,0,4,1).transpose() << endl;
        cout << "cam_" << i << "d:" << endl << cam_calib.block(4,0,4,1).transpose() << endl;
        cout << "T_C" << i << "toI:" << endl << T_CtoI << endl << endl;

    }

    // Debug message
    ROS_INFO("=====================================");

    //===================================================================================
    //===================================================================================
    //===================================================================================

    // Load config values for our feature initializer
    FeatureInitializerOptions featinit_options;
	featinit_options.max_runs = 20; // fi_max_runs
	featinit_options.init_lamda = 1e-3; // fi_init_lamda
	featinit_options.max_lamda = 1e10; // fi_max_lamda
	featinit_options.min_dx = 1e-6; // fi_min_dx
	featinit_options.min_dcost = 1e-6; // fi_min_dcost
	featinit_options.lam_mult = 10; // fi_lam_mult
	featinit_options.min_dist = 0.25; // fi_min_dist
	featinit_options.max_dist = 40; // fi_max_dist
	featinit_options.max_baseline = 40; // fi_max_baseline
	featinit_options.max_cond_number = 1000; // fi_max_cond_number

    // Debug, print to the console!
    ROS_INFO("FEATURE INITIALIZER PARAMETERS:");
    ROS_INFO("\t- max runs: %d", featinit_options.max_runs);
    ROS_INFO("\t- init lambda: %e", featinit_options.init_lamda);
    ROS_INFO("\t- max lambda: %.4f", featinit_options.max_lamda);
    ROS_INFO("\t- min final dx: %.4f", featinit_options.min_dx);
    ROS_INFO("\t- min delta cost: %.4f", featinit_options.min_dcost);
    ROS_INFO("\t- lambda multiple: %.4f", featinit_options.lam_mult);
    ROS_INFO("\t- closest feature dist: %.4f", featinit_options.min_dist);
    ROS_INFO("\t- furthest feature dist: %.4f", featinit_options.max_dist);
    ROS_INFO("\t- max baseline ratio: %.4f", featinit_options.max_baseline);
    ROS_INFO("\t- max condition number: %.4f", featinit_options.max_cond_number);

    // Parameters for our extractor
    int num_pts, fast_threshold, grid_x, grid_y, min_px_dist;
    double knn_ratio;
    bool use_klt, use_aruco, do_downsizing;
	use_klt = true; // use_klt
	//use_klt = false; // dont use_klt
	use_aruco = false; // use_aruco
	num_pts = 150; // num_pts
	fast_threshold = 15; // fast_threshold
	grid_x = 5; // grid_x
	grid_y = 3; // grid_y
	min_px_dist = 10; // min_px_dist
	knn_ratio = 0.7; // knn_ratio
	do_downsizing = true; // downsize_aruco

    // Debug, print to the console!
    ROS_INFO("TRACKING PARAMETERS:");
    ROS_INFO("\t- use klt: %d", use_klt);
    ROS_INFO("\t- use aruco: %d", use_aruco);
    ROS_INFO("\t- max track features: %d", num_pts);
    ROS_INFO("\t- max aruco tags: %d", state->options().max_aruco_features);
    ROS_INFO("\t- grid size: %d x %d", grid_x, grid_y);
    ROS_INFO("\t- fast threshold: %d", fast_threshold);
    ROS_INFO("\t- min pixel distance: %d", min_px_dist);
    ROS_INFO("\t- downsize aruco image: %d", do_downsizing);


    //===================================================================================
    //===================================================================================
    //===================================================================================

    // Our noise values for inertial sensor
    Propagator::NoiseManager imu_noises;
	imu_noises.sigma_w = 1.6968e-04; // gyroscope_noise_density
	imu_noises.sigma_a = 2.0000e-3; // accelerometer_noise_density
	imu_noises.sigma_wb = 1.9393e-05; // gyroscope_random_walk
	imu_noises.sigma_ab = 3.0000e-03; // accelerometer_random_walk


    // Debug print out
    ROS_INFO("PROPAGATOR NOISES:");
    ROS_INFO("\t- sigma_w: %.4f", imu_noises.sigma_w);
    ROS_INFO("\t- sigma_a: %.4f", imu_noises.sigma_a);
    ROS_INFO("\t- sigma_wb: %.4f", imu_noises.sigma_wb);
    ROS_INFO("\t- sigma_ab: %.4f", imu_noises.sigma_ab);

    // Load inertial state initialize parameters
    double init_window_time, init_imu_thresh;
	init_window_time = 0.75; // init_window_time
	init_imu_thresh = 1.5; // init_imu_thresh

    // Debug print out
    ROS_INFO("INITIALIZATION PARAMETERS:");
    ROS_INFO("\t- init_window_time: %.4f", init_window_time);
    ROS_INFO("\t- init_imu_thresh: %.4f", init_imu_thresh);


    // Read in update parameters
    UpdaterOptions msckf_options;
    UpdaterOptions slam_options;
    UpdaterOptions aruco_options;
	msckf_options.sigma_pix = 1; // up_msckf_sigma_px
	msckf_options.chi2_multipler = 1; // up_msckf_chi2_multipler
	slam_options.sigma_pix = 1; // up_slam_sigma_px
	slam_options.chi2_multipler = 5; // up_slam_chi2_multipler
	aruco_options.sigma_pix = 1; // up_aruco_sigma_px
	aruco_options.chi2_multipler = 5; // up_aruco_chi2_multipler

    // If downsampling aruco, then double our noise values
    aruco_options.sigma_pix = (do_downsizing) ? 2*aruco_options.sigma_pix : aruco_options.sigma_pix;

    ROS_INFO("MSCKFUPDATER PARAMETERS:");
    ROS_INFO("\t- sigma_pxmsckf: %.4f", msckf_options.sigma_pix);
    ROS_INFO("\t- sigma_pxslam: %.4f", slam_options.sigma_pix);
    ROS_INFO("\t- sigma_pxaruco: %.4f", aruco_options.sigma_pix);
    ROS_INFO("\t- chi2_multipler msckf: %d", msckf_options.chi2_multipler);
    ROS_INFO("\t- chi2_multipler slam: %d", slam_options.chi2_multipler);
    ROS_INFO("\t- chi2_multipler aruco: %d", aruco_options.chi2_multipler);


    //===================================================================================
    //===================================================================================
    //===================================================================================


    // Lets make a feature extractor
    if(use_klt) {
        trackFEATS = new TrackKLT(num_pts,state->options().max_aruco_features,fast_threshold,grid_x,grid_y,min_px_dist);
        trackFEATS->set_calibration(camera_calib, camera_fisheye);
    } else {
        trackFEATS = new TrackDescriptor(num_pts,state->options().max_aruco_features,fast_threshold,grid_x,grid_y,knn_ratio);
        trackFEATS->set_calibration(camera_calib, camera_fisheye);
    }

    // Initialize our aruco tag extractor
    if(use_aruco) {
        trackARUCO = new TrackAruco(state->options().max_aruco_features,do_downsizing);
        trackARUCO->set_calibration(camera_calib, camera_fisheye);
    }

    // Initialize our state propagator
    propagator = new Propagator(imu_noises,gravity);

    // Our state initialize
    initializer = new InertialInitializer(gravity,init_window_time, init_imu_thresh);

    // Make the updater!
    updaterMSCKF = new UpdaterMSCKF(msckf_options, featinit_options);
    updaterSLAM = new UpdaterSLAM(slam_options, aruco_options, featinit_options);

    total_images = 0;
    counted_images = 0;
    total_tracking_time = 0.0;
    total_filter_time = 0.0;
    total_frame_time = 0.0;
}




void VioManager::feed_measurement_imu(double timestamp, Eigen::Vector3d wm, Eigen::Vector3d am) {

    // Push back to our propagator
    propagator->feed_imu(timestamp,wm,am);

    // Push back to our initializer
    if(!is_initialized_vio) {
        initializer->feed_imu(timestamp, wm, am);
    }

}





void VioManager::feed_measurement_monocular(double timestamp, cv::Mat& img0, size_t cam_id) {

    // Start timing
    rT1 =  boost::posix_time::microsec_clock::local_time();

    // Feed our trackers
    trackFEATS->feed_monocular(timestamp, img0, cam_id);

    // If aruoc is avalible, the also pass to it
    if(trackARUCO != nullptr) {
        trackARUCO->feed_monocular(timestamp, img0, cam_id);
    }
    rT2 =  boost::posix_time::microsec_clock::local_time();

    // If we do not have VIO initialization, then try to initialize
    // TODO: Or if we are trying to reset the system, then do that here!
    if(!is_initialized_vio) {
        is_initialized_vio = try_to_initialize();
        if(!is_initialized_vio) return;
    }

    // Call on our propagate and update function
    do_feature_propagate_update(timestamp);


}


void VioManager::feed_measurement_stereo(double timestamp, cv::Mat& img0, cv::Mat& img1, size_t cam_id0, size_t cam_id1) {

    // Start timing
    rT1 =  boost::posix_time::microsec_clock::local_time();

    // Feed our trackers
    trackFEATS->feed_stereo(timestamp, img0, img1, cam_id0, cam_id1);

    // If aruoc is avalible, the also pass to it
    if(trackARUCO != nullptr) {
        trackARUCO->feed_stereo(timestamp, img0, img1, cam_id0, cam_id1);
    }
    rT2 =  boost::posix_time::microsec_clock::local_time();

    // If we do not have VIO initialization, then try to initialize
    // TODO: Or if we are trying to reset the system, then do that here!
    if(!is_initialized_vio) {
        is_initialized_vio = try_to_initialize();
        if(!is_initialized_vio) return;
    }

    // Call on our propagate and update function
    do_feature_propagate_update(timestamp);

}



void VioManager::feed_measurement_simulation(double timestamp, const std::vector<int> &camids, const std::vector<std::vector<std::pair<size_t,Eigen::VectorXf>>> &feats) {

    // Start timing
    rT1 =  boost::posix_time::microsec_clock::local_time();

    // Check if we actually have a simulated tracker
    TrackSIM *trackSIM = dynamic_cast<TrackSIM*>(trackFEATS);
    if(trackSIM == nullptr) {
        //delete trackFEATS; //(fix this error in the future)
        trackFEATS = new TrackSIM(state->options().max_aruco_features);
        trackFEATS->set_calibration(camera_calib, camera_fisheye);
        ROS_ERROR("[SIM]: casting our tracker to a TrackSIM object!");
    }

    // Cast the tracker to our simulation tracker
    trackSIM = dynamic_cast<TrackSIM*>(trackFEATS);
    trackSIM->set_width_height(camera_wh);

    // Feed our simulation tracker
    trackSIM->feed_measurement_simulation(timestamp, camids, feats);
    rT2 =  boost::posix_time::microsec_clock::local_time();

    // If we do not have VIO initialization, then return an error
    if(!is_initialized_vio) {
        ROS_ERROR("[SIM]: your vio system should already be initialized before simulating features!!!");
        ROS_ERROR("[SIM]: initialize your system first before calling feed_measurement_simulation()!!!!");
        std::exit(EXIT_FAILURE);
    }

    // Call on our propagate and update function
    do_feature_propagate_update(timestamp);


}


bool VioManager::try_to_initialize() {
        // Returns from our initializer
        double time0;
        Eigen::Matrix<double, 4, 1> q_GtoI0;
        Eigen::Matrix<double, 3, 1> b_w0, v_I0inG, b_a0, p_I0inG;

        // Try to initialize the system
        bool success = initializer->initialize_with_imu(time0, q_GtoI0, b_w0, v_I0inG, b_a0, p_I0inG);

        // Return if it failed
        if (!success) {
			cout << "FAIL!!!!!!!!!" << endl;
            return false;
        }

        // Make big vector (q,p,v,bg,ba), and update our state
        // Note: start from zero position, as this is what our covariance is based off of
        Eigen::Matrix<double,16,1> imu_val;
        imu_val.block(0,0,4,1) = q_GtoI0;
        imu_val.block(4,0,3,1) << 0,0,0;
        imu_val.block(7,0,3,1) = v_I0inG;
        imu_val.block(10,0,3,1) = b_w0;
        imu_val.block(13,0,3,1) = b_a0;
        //imu_val.block(10,0,3,1) << 0,0,0;
        //imu_val.block(13,0,3,1) << 0,0,0;
        state->imu()->set_value(imu_val);
        state->set_timestamp(time0);

        // Else we are good to go, print out our stats
        ROS_INFO("\033[0;32m[INIT]: orientation = %.4f, %.4f, %.4f, %.4f\033[0m",state->imu()->quat()(0),state->imu()->quat()(1),state->imu()->quat()(2),state->imu()->quat()(3));
        ROS_INFO("\033[0;32m[INIT]: bias gyro = %.4f, %.4f, %.4f\033[0m",state->imu()->bias_g()(0),state->imu()->bias_g()(1),state->imu()->bias_g()(2));
        ROS_INFO("\033[0;32m[INIT]: velocity = %.4f, %.4f, %.4f\033[0m",state->imu()->vel()(0),state->imu()->vel()(1),state->imu()->vel()(2));
        ROS_INFO("\033[0;32m[INIT]: bias accel = %.4f, %.4f, %.4f\033[0m",state->imu()->bias_a()(0),state->imu()->bias_a()(1),state->imu()->bias_a()(2));
        ROS_INFO("\033[0;32m[INIT]: position = %.4f, %.4f, %.4f\033[0m",state->imu()->pos()(0),state->imu()->pos()(1),state->imu()->pos()(2));
        return true;

}



void VioManager::do_feature_propagate_update(double timestamp) {

    //===================================================================================
    // State propagation, and clone augmentation
    //===================================================================================

    // Return if the camera measurement is out of order
    if(state->timestamp() >= timestamp) {
        ROS_WARN("image received out of order (prop dt = %3f)",(timestamp-state->timestamp()));
        return;
    }

    // If we have just started up, we should record this time as the current time
    if(startup_time == -1) {
        startup_time = timestamp;
    }

    // Propagate the state forward to the current update time
    // Also augment it with a new clone!
    propagator->propagate_and_clone(state, timestamp);
    rT3 =  boost::posix_time::microsec_clock::local_time();

    // If we have not reached max clones, we should just return...
    // This isn't super ideal, but it keeps the logic after this easier...
    // We can start processing things when we have at least 5 clones since we can start triangulating things...
    if((int)state->n_clones() < std::min(state->options().max_clone_size,5)) {
        ROS_INFO("waiting for enough clone states (%d of %d) ....",(int)state->n_clones(),std::min(state->options().max_clone_size,5));
        return;
    }

    // Return if we where unable to propagate
    if(state->timestamp() != timestamp) {
        ROS_ERROR("[PROP]: Propagator unable to propagate the state forward in time!");
        ROS_ERROR("[PROP]: It has been %.3f since last time we propagated",timestamp-state->timestamp());
        return;
    }

    //===================================================================================
    // MSCKF features and KLT tracks that are SLAM features
    //===================================================================================


    // Now, lets get all features that should be used for an update that are lost in the newest frame
    std::vector<Feature*> feats_lost, feats_marg, feats_slam;
    feats_lost = trackFEATS->get_feature_database()->features_not_containing_newer(state->timestamp());

    // Don't need to get the oldest features untill we reach our max number of clones
    if((int)state->n_clones() > state->options().max_clone_size) {
        feats_marg = trackFEATS->get_feature_database()->features_containing(state->margtimestep());
        if(trackARUCO != nullptr && timestamp-startup_time >= dt_statupdelay) {
            feats_slam = trackARUCO->get_feature_database()->features_containing(state->margtimestep());
        }
    }

    // We also need to make sure that the max tracks does not contain any lost features
    // This could happen if the feature was lost in the last frame, but has a measurement at the marg timestep
    auto it1 = feats_lost.begin();
    while(it1 != feats_lost.end()) {
        if(std::find(feats_marg.begin(),feats_marg.end(),(*it1)) != feats_marg.end()) {
            //ROS_WARN("FOUND FEATURE THAT WAS IN BOTH feats_lost and feats_marg!!!!!!");
            it1 = feats_lost.erase(it1);
        } else {
            it1++;
        }
    }

    // Find tracks that have reached max length, these can be made into SLAM features
    std::vector<Feature*> feats_maxtracks;
    auto it2 = feats_marg.begin();
    while(it2 != feats_marg.end()) {
        // See if any of our camera's reached max track
        bool reached_max = false;
        for (const auto &cams: (*it2)->timestamps){
            if ((int)cams.second.size() > state->options().max_clone_size){
                reached_max = true;
                break;
            }
        }
        // If max track, then add it to our possible slam feature list
        if(reached_max) {
            feats_maxtracks.push_back(*it2);
            it2 = feats_marg.erase(it2);
        } else {
            it2++;
        }
    }

    // Count how many aruco tags we have in our state
    int curr_aruco_tags = 0;
    auto it0 = state->features_SLAM().begin();
    while(it0 != state->features_SLAM().end()) {
        if ((int) (*it0).second->_featid <= state->options().max_aruco_features) curr_aruco_tags++;
        it0++;
    }

    // Append a new SLAM feature if we have the room to do so
    // Also check that we have waited our delay amount (normally prevents bad first set of slam points)
    if(state->options().max_slam_features > 0 && timestamp-startup_time >= dt_statupdelay && (int)state->features_SLAM().size() < state->options().max_slam_features+curr_aruco_tags) {
        // Get the total amount to add, then the max amount that we can add given our marginalize feature array
        int amount_to_add = (state->options().max_slam_features+curr_aruco_tags)-(int)state->features_SLAM().size();
        int valid_amount = (amount_to_add > (int)feats_maxtracks.size())? (int)feats_maxtracks.size() : amount_to_add;
        // If we have at least 1 that we can add, lets add it!
        // Note: we remove them from the feat_marg array since we don't want to reuse information...
        if(valid_amount > 0) {
            feats_slam.insert(feats_slam.end(), feats_maxtracks.end()-valid_amount, feats_maxtracks.end());
            feats_maxtracks.erase(feats_maxtracks.end()-valid_amount, feats_maxtracks.end());
        }
    }

    // Loop through current SLAM features, we have tracks of them, grab them for this update!
    // Note: if we have a slam feature that has lost tracking, then we should marginalize it out
    // Note: if you do not use FEJ, these types of slam features *degrade* the estimator performance....
    for (std::pair<const size_t, Landmark*> &landmark : state->features_SLAM()) {
        if(trackARUCO != nullptr) {
            Feature* feat1 = trackARUCO->get_feature_database()->get_feature(landmark.second->_featid);
            if(feat1 != nullptr) feats_slam.push_back(feat1);
        }
        Feature* feat2 = trackFEATS->get_feature_database()->get_feature(landmark.second->_featid);
        if(feat2 != nullptr) feats_slam.push_back(feat2);
        if(feat2 == nullptr) landmark.second->should_marg = true;
    }

    // Lets marginalize out all old SLAM features here
    // These are ones that where not successfully tracked into the current frame
    // We do *NOT* marginalize out our aruco tags
    StateHelper::marginalize_slam(state);

    // Separate our SLAM features into new ones, and old ones
    std::vector<Feature*> feats_slam_DELAYED, feats_slam_UPDATE;
    for(size_t i=0; i<feats_slam.size(); i++) {
        if(state->features_SLAM().find(feats_slam.at(i)->featid) != state->features_SLAM().end()) {
            feats_slam_UPDATE.push_back(feats_slam.at(i));
            //ROS_INFO("[UPDATE-SLAM]: found old feature %d (%d measurements)",(int)feats_slam.at(i)->featid,(int)feats_slam.at(i)->timestamps_left.size());
        } else {
            feats_slam_DELAYED.push_back(feats_slam.at(i));
            //ROS_INFO("[UPDATE-SLAM]: new feature ready %d (%d measurements)",(int)feats_slam.at(i)->featid,(int)feats_slam.at(i)->timestamps_left.size());
        }
    }

    // Concatenate our MSCKF feature arrays (i.e., ones not being used for slam updates)
    std::vector<Feature*> featsup_MSCKF = feats_lost;
    featsup_MSCKF.insert(featsup_MSCKF.end(), feats_marg.begin(), feats_marg.end());
    featsup_MSCKF.insert(featsup_MSCKF.end(), feats_maxtracks.begin(), feats_maxtracks.end());


    //===================================================================================
    // Now that we have a list of features, lets do the EKF update for MSCKF and SLAM!
    //===================================================================================

    // Pass them to our MSCKF updater
    // We update first so that our SLAM initialization will be more accurate??
    updaterMSCKF->update(state, featsup_MSCKF);
    rT4 =  boost::posix_time::microsec_clock::local_time();

    // Perform SLAM delay init and update
    updaterSLAM->update(state, feats_slam_UPDATE);
    updaterSLAM->delayed_init(state, feats_slam_DELAYED);
    rT5 =  boost::posix_time::microsec_clock::local_time();


    //===================================================================================
    // Update our visualization feature set, and clean up the old features
    //===================================================================================


    // Save all the MSCKF features used in the update
    good_features_MSCKF.clear();
    for(Feature* feat : featsup_MSCKF) {
        good_features_MSCKF.push_back(feat->p_FinG);
        feat->to_delete = true;
    }

    // Remove features that where used for the update from our extractors at the last timestep
    // This allows for measurements to be used in the future if they failed to be used this time
    // Note we need to do this before we feed a new image, as we want all new measurements to NOT be deleted
    trackFEATS->get_feature_database()->cleanup();
    if(trackARUCO != nullptr) {
        trackARUCO->get_feature_database()->cleanup();
    }

    //===================================================================================
    // Cleanup, marginalize out what we don't need any more...
    //===================================================================================

    // First do anchor change if we are about to lose an anchor pose
    updaterSLAM->change_anchors(state);

    // Marginalize the oldest clone of the state if we are at max length
    if((int)state->n_clones() > state->options().max_clone_size) {
        StateHelper::marginalize_old_clone(state);
    }

    // Finally if we are optimizing our intrinsics, update our trackers
    if(state->options().do_calib_camera_intrinsics) {
        // Get vectors arrays
        std::map<size_t, Eigen::VectorXd> cameranew_calib;
        std::map<size_t, bool> cameranew_fisheye;
        for(int i=0; i<state->options().num_cameras; i++) {
            Vec* calib = state->get_intrinsics_CAM(i);
            bool isfish = state->get_model_CAM(i);
            cameranew_calib.insert({i,calib->value()});
            cameranew_fisheye.insert({i,isfish});
        }
        // Update the trackers and their databases
        trackFEATS->set_calibration(cameranew_calib, cameranew_fisheye, true);
        if(trackARUCO != nullptr) {
            trackARUCO->set_calibration(cameranew_calib, cameranew_fisheye, true);
        }
    }
    rT6 =  boost::posix_time::microsec_clock::local_time();

    total_images++;
    double frame_time = (rT6 - rT1).total_microseconds() * 1e-3;
    double tracking_time = (rT2 - rT1).total_microseconds() * 1e-3;
    double filter_time = (rT6 - rT2).total_microseconds() * 1e-3;

    // if (total_images >= 100) {
        counted_images++;
        total_tracking_time += tracking_time;
        total_filter_time += filter_time;
        total_frame_time += frame_time;

        frame_tracking_time.push_back(tracking_time);
        frame_filter_time.push_back(filter_time);
        frame_total_time.push_back(frame_time);

        //cout << "Total time = " << total_time << " ms\n";
        //cout << "Total images = " << total_images << "\n";
        //cout << "Average time per frame = " << total_time / (double) total_images << " ms\n";
        //cout << "Number of OpenCV threads = " << cv::getNumThreads() << "\n";
    // }

    //===================================================================================
    // Debug info, and stats tracking
    //===================================================================================

    // Frame time
    ROS_INFO("\u001b[34m[TIME]: %.4f ms for tracking\u001b[0m", tracking_time);
    ROS_INFO("\u001b[34m[TIME]: %.4f ms for filter\u001b[0m", filter_time);
    ROS_INFO("\u001b[34m[TIME]: %.4f ms for total\u001b[0m", frame_time);

    //// Average time
    ROS_INFO("\u001b[34m[AVERAGE TIME]: %.4f ms for tracking\u001b[0m", total_tracking_time / (double) counted_images);
    ROS_INFO("\u001b[34m[AVERAGE TIME]: %.4f ms for filter\u001b[0m", total_filter_time / (double) counted_images);
    ROS_INFO("\u001b[34m[AVERAGE TIME]: %.4f ms for total\u001b[0m", total_frame_time / (double) counted_images);

    cout << "----\n";

    // Timing information
    ROS_INFO("\u001b[34m[TIME]: %.4f seconds for propagation\u001b[0m",(rT3-rT2).total_microseconds() * 1e-6);
    ROS_INFO("\u001b[34m[TIME]: %.4f seconds for MSCKF update (%d features)\u001b[0m",(rT4-rT3).total_microseconds() * 1e-6, (int)good_features_MSCKF.size());
    if(state->options().max_slam_features > 0)
        ROS_INFO("\u001b[34m[TIME]: %.4f seconds for SLAM update (%d delayed, %d update)\u001b[0m",(rT5-rT4).total_microseconds() * 1e-6, (int)feats_slam_UPDATE.size(), (int)feats_slam_DELAYED.size());
    ROS_INFO("\u001b[34m[TIME]: %.4f seconds for marginalization (%d clones in state)\u001b[0m",(rT6-rT5).total_microseconds() * 1e-6, (int)state->n_clones());


    // Update our distance traveled
    if(state->get_clones().find(timelastupdate) != state->get_clones().end()) {
        Eigen::Matrix<double,3,1> dx = state->imu()->pos() - state->get_clone(timelastupdate)->pos();
        distance += dx.norm();
    }
    timelastupdate = timestamp;

    // Debug, print our current state
    //ROS_INFO("q_GtoI = %.3f,%.3f,%.3f,%.3f | p_IinG = %.3f,%.3f,%.3f | dist = %.2f (meters)",
    //        state->imu()->quat()(0),state->imu()->quat()(1),state->imu()->quat()(2),state->imu()->quat()(3),
    //        state->imu()->pos()(0),state->imu()->pos()(1),state->imu()->pos()(2),distance);
    //ROS_INFO("bg = %.4f,%.4f,%.4f | ba = %.4f,%.4f,%.4f",
    //         state->imu()->bias_g()(0),state->imu()->bias_g()(1),state->imu()->bias_g()(2),
    //         state->imu()->bias_a()(0),state->imu()->bias_a()(1),state->imu()->bias_a()(2));


    // Debug for camera imu offset
    if(state->options().do_calib_camera_timeoffset) {
        ROS_INFO("camera-imu timeoffset = %.5f",state->calib_dt_CAMtoIMU()->value()(0));
    }

    // Debug for camera intrinsics
    if(state->options().do_calib_camera_intrinsics) {
        for(int i=0; i<state->options().num_cameras; i++) {
            Vec* calib = state->get_intrinsics_CAM(i);
            ROS_INFO("cam%d intrinsics = %.3f,%.3f,%.3f,%.3f | %.3f,%.3f,%.3f,%.3f",(int)i,
                     calib->value()(0),calib->value()(1),calib->value()(2),calib->value()(3),
                     calib->value()(4),calib->value()(5),calib->value()(6),calib->value()(7));
        }
    }

    // Debug for camera extrinsics
    if(state->options().do_calib_camera_pose) {
        for(int i=0; i<state->options().num_cameras; i++) {
            PoseJPL* calib = state->get_calib_IMUtoCAM(i);
            ROS_INFO("cam%d extrinsics = %.3f,%.3f,%.3f,%.3f | %.3f,%.3f,%.3f",(int)i,
                     calib->quat()(0),calib->quat()(1),calib->quat()(2),calib->quat()(3),
                     calib->pos()(0),calib->pos()(1),calib->pos()(2));
        }
    }





}
