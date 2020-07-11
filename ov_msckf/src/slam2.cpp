#include <functional>

#include <opencv/cv.hpp>
#include <opencv2/core/core.hpp>
#include <opencv2/highgui/highgui.hpp>

#include <math.h>
#include <eigen3/Eigen/Dense>

#include "core/VioManager.h"
#include "state/State.h"

#include "common/plugin.hpp"
#include "common/switchboard.hpp"
#include "common/data_format.hpp"

using namespace ILLIXR;
using namespace ov_msckf;

VioManagerOptions create_params()
{
	VioManagerOptions params;

	// Camera #1
	Eigen::Matrix<double, 8, 1> intrinsics_0;
	intrinsics_0 << 339.99, 339.99, 322.06, 207.483, 0, 0, 0, 0;

    Eigen::Matrix4d T_CtoI_0;
		std::vector<double> matrix_TCtoI_0 =
					 {-0.01080233, 0.00183858, 0.99993996, 0.01220425,
            -0.99993288, -0.00420947, -0.01079452, 0.0146056,
            0.00418937, -0.99998945, 0.00188393, -0.00113692,
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
	params.camera_wh.insert({0, {672, 376}});

	// Camera #2
	Eigen::Matrix<double, 8, 1> intrinsics_1;
	intrinsics_1 << 339.99, 339.99, 322.06, 207.483, 0, 0, 0, 0;

    Eigen::Matrix4d T_CtoI_1;
		std::vector<double> matrix_TCtoI_1 =
					 {-0.01043535, -0.00191061, 0.99994372, 0.01190459,
            -0.99993668, -0.00419281, -0.01044329, -0.04732387,
            0.00421252, -0.99998938, -0.00186674, -0.00098799,
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
	params.camera_wh.insert({1, {672, 376}});

	params.state_options.do_fej = true;
	params.state_options.imu_avg = false;
	params.state_options.use_rk4_integration = true;
	params.use_stereo = true;
	params.state_options.do_calib_camera_intrinsics = true;
	params.state_options.do_calib_camera_timeoffset = true;
	params.calib_camimu_dt = 0; // was 0
	params.state_options.max_clone_size = 11;
	params.state_options.max_slam_features = 75; // was 25
	params.state_options.max_slam_in_update = 25; // was 25
	params.state_options.max_msckf_in_update = 999; // was 999
	params.state_options.num_cameras = 2;
	params.dt_slam_delay = 3; // was 3
	params.init_window_time = 0.75;
	params.init_imu_thresh = 0.05;
	params.gravity = {0, 0, 9.81};
	params.state_options.feat_rep_msckf = LandmarkRepresentation::from_string("GLOBAL_3D");
	params.state_options.feat_rep_slam = LandmarkRepresentation::from_string("ANCHORED_FULL_INVERSE_DEPTH");
	params.state_options.feat_rep_aruco = LandmarkRepresentation::from_string("ANCHORED_FULL_INVERSE_DEPTH");
	// params.sim_freq_imu = 790;
	// params.sim_freq_cam = 50;

	params.use_klt = true;
	params.num_pts = 250;
	params.fast_threshold = 15;
	params.grid_x = 5;
	params.grid_y = 5;
	params.min_px_dist = 10;
	params.knn_ratio = 0.80;
	// params.downsample_cameras = false;

	params.use_aruco = false;
	// params.numaruco = 1024;
	params.downsize_aruco = true;

	params.msckf_options.sigma_pix = 5;
	params.msckf_options.chi2_multipler = 2;
	params.slam_options.sigma_pix = 5;
	params.slam_options.chi2_multipler = 2;
	params.aruco_options.sigma_pix = 5;
	params.aruco_options.chi2_multipler = 1;
	params.imu_noises.sigma_a = 0.403011031507; // accelerometer noise: 0.403011031507 (github issue), 0.0018 (from ZED, delay in debug)
	params.imu_noises.sigma_ab = 0.00072014; // accelorometer random walk: 0.00072014 (github issue), 0.0006435 (from ZED, delay in debug)
	params.imu_noises.sigma_w = 0.323378786016; // gyroscope noise: 0.323378786016 (github issue), 0.007 * (M_PI / 180) (from ZED, delay in debug)
	params.imu_noises.sigma_wb = 1.9393e-05; // gyroscope random walk:  1.9393e-05 (github issue), 0.0019825 * (M_PI / 180) (from ZED, delay in debug)

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
		_m_begin = std::chrono::system_clock::now();
		imu_cam_buffer = NULL;

		_m_pose->put(new pose_type{std::chrono::time_point<std::chrono::system_clock>{}, Eigen::Vector3f{0, 0, 0}, Eigen::Quaternionf{1, 0, 0, 0}});
	}


	virtual void start() override {
		sb->schedule<imu_cam_type>(get_name(), "imu_cam", [&](const imu_cam_type *datum) {
			this->feed_imu_cam(datum);
		});
	}


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

		cv::Mat img0{*imu_cam_buffer->img0.value()};
		cv::Mat img1{*imu_cam_buffer->img1.value()};
		double buffer_timestamp_seconds = double(imu_cam_buffer->dataset_time) / NANO_SEC;
		open_vins_estimator.feed_measurement_stereo(buffer_timestamp_seconds, *(imu_cam_buffer->img0.value()), *(imu_cam_buffer->img1.value()), 0, 1);

		// Get the pose returned from SLAM
		State *state = open_vins_estimator.get_state();
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
				imu_cam_buffer->time,
				swapped_pos,
				swapped_rot,
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
	time_type _m_begin;

	VioManagerOptions manager_params = create_params();
	VioManager open_vins_estimator;

	const imu_cam_type* imu_cam_buffer;
	double previous_timestamp = 0.0;
	bool isUninitialized = true;
};

PLUGIN_MAIN(slam2)
