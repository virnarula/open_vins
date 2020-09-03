#include <functional>

#include <opencv/cv.hpp>
#include <opencv2/core/core.hpp>
#include <opencv2/core/utility.hpp>
#include <opencv2/highgui/highgui.hpp>

#include <math.h>
#include <eigen3/Eigen/Dense>

#include "core/VioManager.h"
#include "state/State.h"

#include "common/plugin.hpp"
#include "common/switchboard.hpp"
#include "common/data_format.hpp"
#include "common/pose_prediction.hpp"
#include "common/phonebook.hpp"

using namespace ILLIXR;
using namespace ov_msckf;

VioManagerOptions create_params()
{
	VioManagerOptions params;

	// Camera #1
	Eigen::Matrix<double, 8, 1> intrinsics_0;
	intrinsics_0 << 458.654, 457.296, 367.215, 248.375, -0.28340811, 0.07395907, 0.00019359, 1.76187114e-05;

    Eigen::Matrix4d T_CtoI_0;
	std::vector<double> matrix_TCtoI_0 = {0.0148655429818, -0.999880929698, 0.00414029679422, -0.0216401454975,
            0.999557249008, 0.0149672133247, 0.025715529948, -0.064676986768,
            -0.0257744366974, 0.00375618835797, 0.999660727178, 0.00981073058949,
            0.0, 0.0, 0.0, 1.0};

	T_CtoI_0 << matrix_TCtoI_0.at(0), matrix_TCtoI_0.at(1), matrix_TCtoI_0.at(2), matrix_TCtoI_0.at(3),
		matrix_TCtoI_0.at(4), matrix_TCtoI_0.at(5), matrix_TCtoI_0.at(6), matrix_TCtoI_0.at(7),
		matrix_TCtoI_0.at(8), matrix_TCtoI_0.at(9), matrix_TCtoI_0.at(10), matrix_TCtoI_0.at(11),
		matrix_TCtoI_0.at(12), matrix_TCtoI_0.at(13), matrix_TCtoI_0.at(14), matrix_TCtoI_0.at(15);

	// Load these into our state
	Eigen::Matrix<double, 7, 1> extrinsics_0;
	extrinsics_0.block(0, 0, 4, 1) = rot_2_quat(T_CtoI_0.block(0, 0, 3, 3).transpose());
	extrinsics_0.block(4, 0, 3, 1) = -T_CtoI_0.block(0, 0, 3, 3).transpose() * T_CtoI_0.block(0, 3, 3, 1);

	params.camera_fisheye.insert({0, false});
	params.camera_intrinsics.insert({0, intrinsics_0});
	params.camera_extrinsics.insert({0, extrinsics_0});
	params.camera_wh.insert({0, {752, 480}});

	// Camera #2
	Eigen::Matrix<double, 8, 1> intrinsics_1;
	intrinsics_1 << 457.587, 456.134, 379.999, 255.238, -0.28368365, 0.07451284, -0.00010473, -3.55590700e-05;

    Eigen::Matrix4d T_CtoI_1;
	std::vector<double> matrix_TCtoI_1 = {0.0125552670891, -0.999755099723, 0.0182237714554, -0.0198435579556,
            0.999598781151, 0.0130119051815, 0.0251588363115, 0.0453689425024,
            -0.0253898008918, 0.0179005838253, 0.999517347078, 0.00786212447038,
            0.0, 0.0, 0.0, 1.0};

	T_CtoI_1 << matrix_TCtoI_1.at(0), matrix_TCtoI_1.at(1), matrix_TCtoI_1.at(2), matrix_TCtoI_1.at(3),
		matrix_TCtoI_1.at(4), matrix_TCtoI_1.at(5), matrix_TCtoI_1.at(6), matrix_TCtoI_1.at(7),
		matrix_TCtoI_1.at(8), matrix_TCtoI_1.at(9), matrix_TCtoI_1.at(10), matrix_TCtoI_1.at(11),
		matrix_TCtoI_1.at(12), matrix_TCtoI_1.at(13), matrix_TCtoI_1.at(14), matrix_TCtoI_1.at(15);

	// Load these into our state
	Eigen::Matrix<double, 7, 1> extrinsics_1;
	extrinsics_1.block(0, 0, 4, 1) = rot_2_quat(T_CtoI_1.block(0, 0, 3, 3).transpose());
	extrinsics_1.block(4, 0, 3, 1) = -T_CtoI_1.block(0, 0, 3, 3).transpose() * T_CtoI_1.block(0, 3, 3, 1);

	params.camera_fisheye.insert({1, false});
	params.camera_intrinsics.insert({1, intrinsics_1});
	params.camera_extrinsics.insert({1, extrinsics_1});
	params.camera_wh.insert({1, {752, 480}});

	// params.state_options.max_slam_features = 0;
	params.state_options.num_cameras = 2;
	params.init_window_time = 0.75;
	params.init_imu_thresh = 1.5;
	params.fast_threshold = 15;
	params.grid_x = 5;
	params.grid_y = 3;
	params.num_pts = 150;
	params.msckf_options.chi2_multipler = 1;
	params.knn_ratio = .7;

	params.state_options.imu_avg = true;
	params.state_options.do_fej = true;
	params.state_options.use_rk4_integration = true;
	params.use_stereo = true;
	params.state_options.do_calib_camera_pose = true;
	params.state_options.do_calib_camera_intrinsics = true;
	params.state_options.do_calib_camera_timeoffset = true;

	params.dt_slam_delay = 3.0;
	params.state_options.max_slam_features = 50;
	params.state_options.max_slam_in_update = 25;
	params.state_options.max_msckf_in_update = 999;

	params.slam_options.chi2_multipler = 1;
	params.slam_options.sigma_pix = 1;

	params.use_aruco = false;

	params.state_options.feat_rep_slam = LandmarkRepresentation::from_string("ANCHORED_FULL_INVERSE_DEPTH");
    params.state_options.feat_rep_aruco = LandmarkRepresentation::from_string("ANCHORED_FULL_INVERSE_DEPTH");


	return params;
}

class slam2 : public plugin {
public:
	/* Provide handles to slam2 */
	slam2(std::string name_, phonebook* pb_)
		: plugin{name_, pb_}
		, sb{pb->lookup_impl<switchboard>()}
		, open_vins_estimator{manager_params}
	{
		_m_pose = sb->publish<pose_type>("slow_pose");
		_m_imu_biases = sb->publish<imu_biases_type>("imu_biases");
		_m_begin = std::chrono::system_clock::now();
		imu_cam_buffer = NULL;

		_m_pose->put(
			new pose_type{
				.sensor_time = std::chrono::time_point<std::chrono::system_clock>{},
				.position = Eigen::Vector3f{0, 0, 0},
				.orientation = Eigen::Quaternionf{1, 0, 0, 0}
			}
		);

#ifdef CV_HAS_METRICS
		cv::metrics::setAccount(new std::string{"-1"});
#endif

	}


	virtual void start() override {
		plugin::start();
		sb->schedule<imu_cam_type>(id, "imu_cam", [&](const imu_cam_type *datum) {
			this->feed_imu_cam(datum);
		});
	}


	std::size_t iteration_no = 0;
	void feed_imu_cam(const imu_cam_type *datum) {
		// Ensures that slam doesnt start before valid IMU readings come in
		if (datum == NULL) {
			assert(previous_timestamp == 0);
			return;
		}

		// This ensures that every data point is coming in chronological order If youre failing this assert, 
		// make sure that your data folder matches the name in offline_imu_cam/plugin.cc
		double timestamp_in_seconds = (double(datum->dataset_time) / NANO_SEC);
		assert(timestamp_in_seconds > previous_timestamp);
		previous_timestamp = timestamp_in_seconds;

		// Feed the IMU measurement. There should always be IMU data in each call to feed_imu_cam
		assert((datum->img0.has_value() && datum->img1.has_value()) || (!datum->img0.has_value() && !datum->img1.has_value()));
		open_vins_estimator.feed_measurement_imu(timestamp_in_seconds, (datum->angular_v).cast<double>(), (datum->linear_a).cast<double>());
		if (open_vins_estimator.initialized()) {
			Eigen::Matrix<double,13,1> state_plus = Eigen::Matrix<double,13,1>::Zero();
			imu_biases_type *biases = new imu_biases_type {
				Eigen::Matrix<double, 3, 1>::Zero(), 
				Eigen::Matrix<double, 3, 1>::Zero(), 
				Eigen::Matrix<double, 3, 1>::Zero(), 
				Eigen::Matrix<double, 3, 1>::Zero(),
				Eigen::Matrix<double, 13, 1>::Zero(),
				// Record the timestamp (in ILLIXR time) associated with this imu sample.
				// Used for MTP calculations.
				datum->time
			};
        	open_vins_estimator.get_propagator()->fast_state_propagate(state, timestamp_in_seconds, state_plus, biases);

			_m_imu_biases->put(biases);
		}

		// std::cout << std::fixed << "Time of IMU/CAM: " << timestamp_in_seconds * 1e9 << " Lin a: " << 
		// 	datum->angular_v[0] << ", " << datum->angular_v[1] << ", " << datum->angular_v[2] << ", " <<
		// 	datum->linear_a[0] << ", " << datum->linear_a[1] << ", " << datum->linear_a[2] << std::endl;

		// If there is not cam data this func call, break early
		if (!datum->img0.has_value() && !datum->img1.has_value()) {
			return;
		} else if (imu_cam_buffer == NULL) {
			imu_cam_buffer = datum;
			return;
		}

#ifdef CV_HAS_METRICS
		cv::metrics::setAccount(new std::string{std::to_string(iteration_no)});
		iteration_no++;
		if (iteration_no % 20 == 0) {
			cv::metrics::dump();
		}
#else
#warning "No OpenCV metrics available. Please recompile OpenCV from git clone --branch 3.4.6-instrumented https://github.com/ILLIXR/opencv/. (see install_deps.sh)"
#endif

		cv::Mat img0{*imu_cam_buffer->img0.value()};
		cv::Mat img1{*imu_cam_buffer->img1.value()};
		double buffer_timestamp_seconds = double(imu_cam_buffer->dataset_time) / NANO_SEC;
		open_vins_estimator.feed_measurement_stereo(buffer_timestamp_seconds, img0, img1, 0, 1);

		// Get the pose returned from SLAM
		state = open_vins_estimator.get_state();
		Eigen::Vector4d quat = state->_imu->quat();
		Eigen::Vector3d pose = state->_imu->pos();

		Eigen::Vector3f swapped_pos = Eigen::Vector3f{float(pose(0)), float(pose(1)), float(pose(2))};
		Eigen::Quaternionf swapped_rot = Eigen::Quaternionf{float(quat(3)), float(quat(0)), float(quat(1)), float(quat(2))};

       	assert(isfinite(swapped_rot.w()));
        assert(isfinite(swapped_rot.x()));
        assert(isfinite(swapped_rot.y()));
        assert(isfinite(swapped_rot.z()));
        assert(isfinite(swapped_pos[0]));
        assert(isfinite(swapped_pos[1]));
        assert(isfinite(swapped_pos[2]));

		if (open_vins_estimator.initialized()) {
			if (isUninitialized) {
				isUninitialized = false;
			}

			_m_pose->put(new pose_type{
				.sensor_time = imu_cam_buffer->time,
				.position = swapped_pos,
				.orientation = swapped_rot,
			});
		}

		// I know, a priori, nobody other plugins subscribe to this topic
		// Therefore, I can const the cast away, and delete stuff
		// This fixes a memory leak.
		// -- Sam at time t1
		// Turns out, this is no longer correct. debbugview uses it
		// const_cast<imu_cam_type*>(imu_cam_buffer)->img0.reset();
		// const_cast<imu_cam_type*>(imu_cam_buffer)->img1.reset();
		imu_cam_buffer = datum;
	}

	virtual ~slam2() override {}

private:
	const std::shared_ptr<switchboard> sb;
	std::unique_ptr<writer<pose_type>> _m_pose;
	std::unique_ptr<writer<imu_biases_type>> _m_imu_biases;
	time_type _m_begin;
	State *state;

	VioManagerOptions manager_params = create_params();
	VioManager open_vins_estimator;

	const imu_cam_type* imu_cam_buffer;
	double previous_timestamp = 0.0;
	bool isUninitialized = true;
};

PLUGIN_MAIN(slam2)
