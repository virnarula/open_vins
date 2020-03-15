#include <functional>

#include "common/component.hh"
#include "common/switchboard.hh"
#include "common/data_format.hh"

#include <opencv/cv.hpp>
#include <opencv2/core/core.hpp>
#include <opencv2/highgui/highgui.hpp>

#include <eigen3/Eigen/Dense>

#include "core/VioManager.h"
#include "core/RosVisualizer.h"
#include "state/State.h"

using namespace ILLIXR;

class slam1 : public component {
public:
	/* Provide handles to slam1 */
	slam1(std::unique_ptr<writer<pose_type>>&& pose)
		: _m_pose{std::move(pose)}
		, _m_begin{std::chrono::system_clock::now()}
	{
		open_vins_estimator = new VioManager();
	}

	void feed_cam(const cam_type* cam_frame) {
		// okvis_estimator.addImage(cvtTime(cam_frame->time), cam_frame->id, *cam_frame->img);
		open_vins_estimator->feed_measurement_stereo(time_buffer, img0_buffer, img1_buffer, 0, 1);
		State* state = open_vins_estimator.get_state();
		// TODO: unpack state 
	}

	void feed_imu(const imu_type* imu_reading) {
		open_vins_estimator->feed_measurement_imu(timem/1000000000.0, imu_reading->angular_v, imu_reading->linear_a);
	}

	virtual void _p_start() override {
		/* All of my work is already scheduled synchronously. Nohting to do here. */
	}

	virtual void _p_stop() override { }

	virtual ~slam1() override { }

private:
	std::unique_ptr<writer<pose_type>> _m_pose;
	time_type _m_begin;

	VioManager open_vins_estimator;
	
	okvis::Time cvtTime(time_type t) {
		auto diff = t - _m_begin;
		auto nanosecs = static_cast<uint64_t>(std::chrono::duration_cast<std::chrono::nanoseconds>(diff).count());
		auto secs = static_cast<uint32_t>(nanosecs / 1000000);
		auto nanosecs_rem = static_cast<uint32_t>(nanosecs - secs * 1000000);
		return {secs, nanosecs_rem};
	}

};

extern "C" component* create_component(switchboard* sb) {
	/* First, we declare intent to read/write topics. Switchboard
	   returns handles to those topics. */
	auto pose_ev = sb->publish<pose_type>("pose");

	auto this_slam1 = new slam1{std::move(pose_ev)};

	sb->schedule<cam_type>("cam0", std::bind(&slam1::feed_cam, this_slam1, std::placeholders::_1));
	sb->schedule<cam_type>("cam1", std::bind(&slam1::feed_cam, this_slam1, std::placeholders::_1));
	sb->schedule<imu_type>("imu0", std::bind(&slam1::feed_imu, this_slam1, std::placeholders::_1));

	return this_slam1;
}
