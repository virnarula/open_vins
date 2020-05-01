#include <functional>

#include <opencv/cv.hpp>
#include <opencv2/core/core.hpp>
#include <opencv2/highgui/highgui.hpp>

#include <eigen3/Eigen/Dense>

#include "core/VioManager.h"
// #include "core/RosVisualizer.h"
#include "state/State.h"

#include "common/plugin.hh"
#include "common/switchboard.hh"
#include "common/data_format.hh"

using namespace ILLIXR;

using namespace ov_msckf;

VioManagerOptions create_params() {
	VioManagerOptions params;

	Eigen::Matrix<double, 8, 1> intrinsics;
	intrinsics << 458.654, 457.296, 367.215, 248.375, -0.28340811, 0.07395907, 0.00019359, 1.76187114e-05;

	Eigen::Matrix<double, 7, 1> extrinsics;
	extrinsics << 0, 0, 0, 1, 0, 0, 0;

	for (int i = 0; i < 2; i++) {
		params.camera_fisheye.insert({i, false});
		params.camera_intrinsics.insert({i, intrinsics});
		params.camera_extrinsics.insert({i, extrinsics});
		params.camera_wh.insert({i, {752,480}});
	}

	params.state_options.num_cameras = 2;

	return params;
}

class slam2 : public plugin
{
public:
	/* Provide handles to slam2 */
	slam2(phonebook *pb)
		: sb{pb->lookup_impl<switchboard>()}
		, open_vins_estimator{manager_params}
	{

		_m_pose = sb->publish<pose_type>("slow_pose");
		_m_begin = std::chrono::system_clock::now();

		_m_pose->put(new pose_type{std::chrono::system_clock::now(), Eigen::Vector3f{0, 0, 0}, Eigen::Quaternionf{1, 0, 0, 0}});
		sb->schedule<cam_type>("cams", std::bind(&slam2::feed_cam, this, std::placeholders::_1));
		sb->schedule<imu_type>("imu0", std::bind(&slam2::feed_imu, this, std::placeholders::_1));
	}

	void feed_cam(const cam_type *cam_frame)
	{
		const std::lock_guard<std::mutex> lock{_m_mutex};
		// okvis_estimator.addImage(cvtTime(cam_frame->time), cam_frame->id, *cam_frame->img);
		assert(cam_frame->img0);
		assert(cam_frame->img1);
		cv::Mat img0{*cam_frame->img0};
		cv::Mat img1{*cam_frame->img1};
		open_vins_estimator.feed_measurement_stereo(cvtTime(cam_frame->time), img0, img1, 0, 1);
		State *state = open_vins_estimator.get_state();

		Eigen::Vector4d quat = state->_imu->quat();
		Eigen::Vector3d pose = state->_imu->pos();

		Eigen::Vector3f swapped_pos = Eigen::Vector3f{float(pose(0)), float(pose(1)), float(pose(2))};
		Eigen::Quaternionf swapped_rot = Eigen::Quaternionf{float(quat(3)), float(quat(0)), float(quat(1)), float(quat(2))};

		//std::cerr << "cam_frame->time: " << cam_frame->time.time_since_epoch().count() << std::endl;

		if (open_vins_estimator.initialized())
		{
			if (isUninitialized)
			{
				isUninitialized = false;
			}
			_m_pose->put(new pose_type{
				cam_frame->time,
				swapped_pos,
				swapped_rot,
			});
		}
	}

	void feed_imu(const imu_type *imu_reading)
	{
		const std::lock_guard<std::mutex> lock{_m_mutex};
		open_vins_estimator.feed_measurement_imu(cvtTime(imu_reading->time), (imu_reading->angular_v).cast<double>(), (imu_reading->linear_a).cast<double>());
	}

	virtual ~slam2() override {}

private:
	switchboard *const sb;
	std::unique_ptr<writer<pose_type>> _m_pose;
	time_type _m_begin;
	std::mutex _m_mutex;

	VioManagerOptions manager_params = create_params();
	VioManager open_vins_estimator;

	bool isUninitialized = true;

	double cvtTime(time_type t)
	{
		auto diff = t - _m_begin;
		return static_cast<double>(std::chrono::duration_cast<std::chrono::nanoseconds>(diff).count()) / 1000000000.0;
	}
};

PLUGIN_MAIN(slam2)