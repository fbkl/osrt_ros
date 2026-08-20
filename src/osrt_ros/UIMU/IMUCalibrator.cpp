/**
 * -----------------------------------------------------------------------------
 * Copyright 2019-2021 OpenSimRT developers.
 *
 * This file is part of OpenSimRT.
 *
 * OpenSimRT is free software: you can redistribute it and/or modify it under
 * the terms of the GNU General Public License as published by the Free Software
 * Foundation, either version 3 of the License, or (at your option) any later
 * version.
 *
 * OpenSimRT is distributed in the hope that it will be useful, but WITHOUT ANY
 * WARRANTY; without even the implied warranty of MERCHANTABILITY or FITNESS FOR
 * A PARTICULAR PURPOSE.  See the GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License along with
 * OpenSimRT. If not, see <https://www.gnu.org/licenses/>.
 * -----------------------------------------------------------------------------
 */
#include "osrt_ros/UIMU/IMUCalibrator.h"
#include "Exception.h"
#include "geometry_msgs/Quaternion.h"
#include "geometry_msgs/Vector3.h"
#include "osrt_ros/FloatRequest.h"
#include "osrt_ros/FloatResponse.h"
#include "ros/datatypes.h"
#include "ros/init.h"
#include "ros/message_traits.h"
#include "ros/node_handle.h"
#include "ros/time.h"
#include "std_srvs/Empty.h"
#include "std_srvs/EmptyRequest.h"
#include "osrt_ros/Float.h"
#include "tf/exceptions.h"
#include "tf2/LinearMath/Matrix3x3.h"
#include "tf2/LinearMath/Quaternion.h"
#include <SimTKcommon/internal/CoordinateAxis.h>
#include <SimTKcommon/internal/Quaternion.h>
#include <SimTKcommon/internal/Rotation.h>
#include <chrono>
#include <cmath>
#include <sstream>
#include <ros/ros.h>
#include <geometry_msgs/PoseArray.h>
#include <thread>
#include <vector>

using namespace OpenSimRT;
using namespace OpenSim;
;
using namespace std;

const std::string red("\033[0;31m");
const std::string green("\033[1;32m");
const std::string yellow("\033[1;33m");
const std::string cyan("\033[0;36m");
const std::string magenta("\033[0;35m");
const std::string reset("\033[0m");
#define ROS_YE(x) ROS_INFO_STREAM( yellow << x << reset)

inline constexpr auto hash_djb2a(const std::string_view sv) {
	unsigned long hash{ 5381 };
	for (unsigned char c : sv) {
		hash = ((hash << 5) + hash) ^ c;
	}
	return hash;
}

inline constexpr auto operator"" _sh(const char *str, size_t len) {
	return hash_djb2a(std::string_view{ str, len });
}



void IMUCalibrator::setup(const std::vector<std::string>& observationOrder) {
	//	ros::NodeHandle n("~");
	nhandle = ros::NodeHandle("~");
	auto ghandle = ros::NodeHandle();
	nhandle.param<string>("debug_reference_frame",debug_reference_frame,"map");
	std::string tf_prefix;
	nhandle.param<string>("tf_prefix",tf_prefix,"");

	ext_heading_srv = ghandle.serviceClient<osrt_ros::Float>("calibrate_heading", true);

	R_heading = SimTK::Rotation();
	//why?
	R_GoGi1 = SimTK::Rotation();

	// copy observation order list
	imuBodiesObservationOrder = std::vector<std::string>(
			observationOrder.begin(), observationOrder.end());

	for (auto imu_name:imuBodiesObservationOrder) 
	{
		std::string calib_serv_name =tf_prefix+imu_name+"/pose_average/calibrate_pose"; 
		ROS_YE("calibration service name:"<< calib_serv_name);
		autosrv this_srv;
		this_srv.imu = imu_name;
		this_srv.calib_client = ghandle.serviceClient<std_srvs::Empty>(calib_serv_name, true);
		calib_srv.push_back(this_srv);
	}
	// initialize system
	state = model.initSystem();
	model.realizePosition(state);
		sameHeader.frame_id = "opensim_frame";
		sameHeader.stamp = ros::Time::now();

	// Get body orientation in ground at the model's DEFAULT pose.
	//
	// NOTE: this map MUST be keyed by the BASE frame name, not by the raw
	// observation-order label. InverseKinematics::createIMUTasksFromObservationOrder
	// calls frame->findBaseFrame() and puts THAT name into imuTasks[i].body, and
	// that base body is what the SimTK OrientationSensor is attached to. If the
	// observation order names offset frames (an *_imu frame welded to a body),
	// keying by `label` here means calibrateIMUTasks() looks up a key that does
	// not exist and std::map::operator[] silently hands back an identity
	// transform -- which is exactly the bug we are removing.
	for (const auto& label : imuBodiesObservationOrder) {
		const OpenSim::PhysicalFrame* frame = nullptr;
		pub.push_back(nhandle.advertise<geometry_msgs::PoseArray>(label +"/imu_cal",1,true)); //latching topic
		if ((frame = model.findComponent<OpenSim::PhysicalFrame>(label))) {
			const OpenSim::Frame& theBaseFrame = frame->findBaseFrame();
			const std::string baseName = theBaseFrame.getName();
			imuBodiesInGround[baseName] = theBaseFrame.getTransformInGround(state); // R_GoB
			publishTransform(baseName+"true" ,imuBodiesInGround[baseName], sameHeader);
			ROS_INFO_STREAM(cyan << "default-pose body in ground, imu label [" << label
					<< "] -> base frame [" << baseName << "]: "
					<< imuBodiesInGround[baseName] << reset);
		}
		else
			ROS_WARN_STREAM("couldnt find physical frame for label: "<< label);
	}

	// =====================================================================
	// LOUD CHECK: is the model's DEFAULT pose the same as its ZERO pose?
	//
	// calibrateIMUTasks() solves R_BS = ~R_GoB_default * R_GoS, which is only
	// correct if the subject is physically holding the model's DEFAULT pose during
	// the static recording. `initSystem()` above poses the model at each
	// coordinate's default_value, NOT at zero. So if any default_value is non-zero,
	// "default pose" and "all coordinates zero" are two different poses -- and
	// anything that resets to zero (the visualiser, the marker/vicon path) will
	// disagree with this calibration by exactly that difference, silently.
	//
	// Real example this was written for: MOBL_ARMS_41.osim has groundthorax/r_y
	// with default_value -1.5707963 about axis (0,1,0), i.e. a -90 deg YAW on the
	// base body. Cost a long time to find because every symptom pointed at the
	// frame plumbing instead of at the model file.
	//
	// Deliberately emitted at WARN/ERROR: INFO-level output does not survive the
	// paramiko launch path, and ROS_ERROR lands in the stderr/warning log which
	// does get through.
	// =====================================================================
	{
		std::vector<std::string> nonZeroDefaults;
		for (int ci = 0; ci < model.getCoordinateSet().getSize(); ++ci) {
			const OpenSim::Coordinate& c = model.getCoordinateSet().get(ci);
			const double dv = c.getDefaultValue();
			if (std::fabs(dv) < 1e-9) continue;
			const bool isRot =
				(c.getMotionType() == OpenSim::Coordinate::MotionType::Rotational);
			std::stringstream ss;
			ss << c.getName() << " = " << dv;
			if (isRot) ss << " rad (" << dv * 180.0 / SimTK::Pi << " deg)";
			else       ss << " m";
			nonZeroDefaults.push_back(ss.str());
		}

		// What actually matters is NARROWER than "some body is rotated in ground".
		//
		// A non-identity R_GoB is completely normal and is handled correctly by the
		// ~R_GoB factor in calibrateIMUTasks(): iiwa14 has link_0 at 90 deg about X
		// purely from the URDF Z-up -> OpenSim Y-up conversion, and it calibrates
		// fine. Warning on that would be crying wolf.
		//
		// The dangerous case is a non-zero ROTATIONAL default on a joint whose
		// parent is GROUND, because that rotates the entire model relative to the
		// world -- so anything that resets to the zero pose ends up 90-ish degrees
		// away from what calibration assumed. MOBL's groundthorax/r_y is exactly
		// this; iiwa14 and raquegopal have nothing of the kind.
		std::vector<std::string> groundRotations;
		for (int ci = 0; ci < model.getCoordinateSet().getSize(); ++ci) {
			const OpenSim::Coordinate& c = model.getCoordinateSet().get(ci);
			const double dv = c.getDefaultValue();
			if (std::fabs(dv) < 1e-9) continue;
			if (c.getMotionType() != OpenSim::Coordinate::MotionType::Rotational) continue;
			try {
				const OpenSim::Joint& jnt = c.getJoint();
				const OpenSim::Frame& pbase = jnt.getParentFrame().findBaseFrame();
				if (pbase.getName() != model.getGround().getName()) continue;
				std::stringstream ss;
				ss << c.getName() << " on joint [" << jnt.getName() << "] = "
					<< dv * 180.0 / SimTK::Pi << " deg  (parent is GROUND)";
				groundRotations.push_back(ss.str());
			} catch (const std::exception& e) {
				ROS_WARN_STREAM("could not resolve joint for coordinate ["
						<< c.getName() << "]: " << e.what());
			}
		}

		// Context only, never a trigger -- see the note above about iiwa14.
		for (const auto& kv : imuBodiesInGround) {
			const SimTK::Vec4 aa = kv.second.R().convertRotationToAngleAxis();
			const double deg = aa[0] * 180.0 / SimTK::Pi;
			if (std::fabs(deg) < 1e-6) continue;
			ROS_INFO_STREAM(cyan << "default-pose R_GoB: " << kv.first << " rotated "
					<< deg << " deg about (" << aa[1] << ", " << aa[2] << ", " << aa[3]
					<< ") -- normal, handled by ~R_GoB" << reset);
		}

		if (!groundRotations.empty()) {
			ROS_ERROR_STREAM(red
				<< "\n"
				<< "############################################################\n"
				<< "###  THIS MODEL HAS A NON-ZERO INITIAL (DEFAULT) STATE   ###\n"
				<< "###  AND IT WILL OFFSET YOUR CALIBRATION.                ###\n"
				<< "############################################################"
				<< reset);
			for (const auto& s : groundRotations)
				ROS_ERROR_STREAM(red << "###  " << s << reset);
			ROS_ERROR_STREAM(red
				<< "###\n"
				<< "###  calibrateIMUTasks() uses R_BS = ~R_GoB_default * R_GoS.\n"
				<< "###  R_GoB_default is taken at the model's DEFAULT pose (above),\n"
				<< "###  NOT at the all-coordinates-zero pose. If the subject is\n"
				<< "###  standing in the ZERO pose, or if the visualiser/marker path\n"
				<< "###  resets to ZERO, they disagree with this calibration by\n"
				<< "###  exactly the rotations listed above.\n"
				<< "###\n"
				<< "###  Decide which pose is the truth and make everything agree:\n"
				<< "###    - subject matches ZERO    -> zero the default_value in the .osim\n"
				<< "###    - subject matches DEFAULT -> make the visualiser reset to the\n"
				<< "###                                default pose, not to zero\n"
				<< "############################################################"
				<< reset);
		}

		if (!nonZeroDefaults.empty()) {
			std::stringstream ss;
			for (const auto& s : nonZeroDefaults) ss << "\n###    " << s;
			ROS_WARN_STREAM(yellow
				<< "\n### Coordinates with a non-zero default_value ("
				<< nonZeroDefaults.size() << "):" << ss.str()
				<< "\n### These are NOT necessarily a problem -- they only matter if the "
				"subject is not physically holding this pose during the static recording. "
				"Ones below ground level give PER-BODY errors, not a uniform offset, so the "
				"two are distinguishable in the output."
				<< reset);
		} else {
			ROS_INFO_STREAM(green << "Model default pose == zero pose (no non-zero "
					"default_value anywhere). Calibration assumptions hold." << reset);
		}
	}

	auto nh = ros::NodeHandle("~");

	nh.param<bool>("send_start_signal_to_external_heading_calibrator",send_start_signal_to_external_heading_calibrator,false); //. this is incorrect. we should use the string from the_method to match this and start the actual services. I can also bypass a ton of stuff here, which would also make sense for this to be different classes, but well..
}



SimTK::Rotation IMUCalibrator::setGroundOrientationSeq(const double& xDegrees,
		const double& yDegrees,
		const double& zDegrees) {
	auto xRad = SimTK::convertDegreesToRadians(xDegrees);
	auto yRad = SimTK::convertDegreesToRadians(yDegrees);
	auto zRad = SimTK::convertDegreesToRadians(zDegrees);


	//my trusty calculator (https://www.andre-gaschler.com/rotationconverter/) tells me that this is in fact a zyx euler rotation.
	auto R_GoGiX = SimTK::Rotation(SimTK::BodyOrSpaceType::SpaceRotationSequence, xRad,
			SimTK::XAxis, yRad, SimTK::YAxis, zRad, SimTK::ZAxis);
	ROS_DEBUG_STREAM("ground orientation matrix:\n" << R_GoGiX);
	return R_GoGiX;
}

SimTK::Rotation IMUCalibrator::setGroundOrientationFromTF(const std::string& tfname)
{
	SimTK::Quaternion q;
	if (true)
	{
	geometry_msgs::TransformStamped standard_imu_orientation_tf;

	try{
		// target frame, source frame!!!
		//standard_imu_orientation_tf = tfBuffer.lookupTransform("imu_ref_ori", "opensim_frame", ros::Time(0));
		standard_imu_orientation_tf = tfBuffer.lookupTransform(tfname, "opensim_frame", ros::Time(0));
		//coult it be an inverse transform_????
		// doesnt look likeit
		//standard_imu_orientation_tf = tfBuffer.lookupTransform("map", "imu_ref_ori", ros::Time(0));	
	}
	catch(tf::TransformException& ex)
	{
		ROS_ERROR("XXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXcouldnt read standard imu_orientation. reason:%s",ex.what());
		SimTK::Rotation myR;
		return myR;
	}
	q[0] = standard_imu_orientation_tf.transform.rotation.w;
	q[1] = standard_imu_orientation_tf.transform.rotation.x;
	q[2] = standard_imu_orientation_tf.transform.rotation.y;
	q[3] = standard_imu_orientation_tf.transform.rotation.z;
	}
	else{
		ROS_WARN_STREAM("bypassing findint the imu orientation correction because i think i know this already");	
	}
	// default constructor for q already makes it w, x, y, z 1,0,0,0
	SimTK::Rotation myR(q);
	return myR;
}
/**
 * Compute the per-session heading correction and leave it in R_heading.
 *
 * The caller is responsible for folding it in via
 *
 *     R_GoGi1 := R_heading * R_GoGi1
 *
 * BEFORE calling calibrateIMUTasks(). See UIMUnode::start_ik(). Folding it into
 * R_GoGi1 is what guarantees the correction reaches both calibrateIMUTasks() and
 * transform(); applying it to only one side (which is what used to happen --
 * calibrateIMUTasks() used it, transform() did not) bakes a spurious yaw into
 * every solved pose.
 *
 * Why this is needed at all, since it looks like a fudge factor: the heading is
 * NOT recoverable from the static pose. Each sensor gives 3 equations and brings
 * 3 unknowns of its own (its R_BS), so N sensors give 3N equations for 3N+1
 * unknowns -- the leftover being the yaw of R_GoGi. Under-determined by exactly
 * one, for any N. The missing DOF has to be supplied externally, and
 * `imuDirectionAxis` is that external input: it names which principal axis (or
 * its inverse) of the base sensor points along the subject's anterior direction.
 * That is a weak enough claim to be true of a hand-strapped sensor and strong
 * enough to pin the last number.
 *
 * NOTE: this is deliberately NOT folded into the imu_ref_ori TF. That TF encodes
 * the orientation provider's fixed frame convention (z-forward / y-down for a
 * camera) and is authored once per device; the heading depends on where someone
 * physically mounted the base sensor this session. Different lifetimes.
 */
SimTK::Rotation
IMUCalibrator::computeHeadingRotation(const std::string& baseImuName,
		const std::string& imuDirectionAxis) {
	bool negate = false;

		geometry_msgs::TransformStamped some_tf;
		sameHeader.stamp = ros::Time::now();
		some_tf.header = sameHeader;




	double angularDifference = 0.0;
	if (!imuDirectionAxis.empty() && !baseImuName.empty()) {
		// set coordinate direction based on given imu direction axis given as
		// string
		std::string imuAxis = IO::Lowercase(imuDirectionAxis);
		ROS_INFO_STREAM("using imu heading axis of: "<< imuAxis);
		SimTK::CoordinateDirection baseHeadingDirection(SimTK::ZAxis);
		int direction = 1;
		if (imuAxis.front() == '-') direction = -1;
		const char& back = imuAxis.back();
		////////// OKAY, so this is maybe mislabelled_?? and this is the heading of the IMU???
		if (back == 'x')
			baseHeadingDirection =
				SimTK::CoordinateDirection(SimTK::XAxis, direction);
		else if (back == 'y')
			baseHeadingDirection =
				SimTK::CoordinateDirection(SimTK::YAxis, direction);
		else if (back == 'z')
			baseHeadingDirection =
				SimTK::CoordinateDirection(SimTK::ZAxis, direction);
		else { // Throw, invalid specification
			THROW_EXCEPTION("Invalid specification of heading axis '" +
					imuAxis + "' found.");
		}

		// find base imu body index in observation order
		baseBodyIndex = std::distance(
				imuBodiesObservationOrder.begin(),
				std::find(imuBodiesObservationOrder.begin(),
					imuBodiesObservationOrder.end(), baseImuName));

		// GUARD: std::find returns end() when the base body is not in the
		// observation order, which makes baseBodyIndex == size() and the
		// staticPoseQuaternions[baseBodyIndex] below an out-of-bounds read. That
		// yields a garbage non-unit quaternion, and SimTK::Rotation(q) on it either
		// asserts or silently produces a non-orthogonal matrix. Bail out with the
		// identity heading instead -- a wrong heading is recoverable, a corrupted
		// rotation is not.
		if (baseBodyIndex >= (long)imuBodiesObservationOrder.size()) {
			ROS_ERROR_STREAM(red << "base imu [" << baseImuName << "] is NOT in the imu "
					"observation order! No heading correction applied. Check imu_base_body "
					"against imu_observation_order." << reset);
			R_heading = SimTK::Rotation();
			return R_heading;
		}
		if (baseBodyIndex >= (long)staticPoseQuaternions.size()) {
			ROS_ERROR_STREAM(red << "staticPoseQuaternions has only " << staticPoseQuaternions.size()
					<< " entries but the base imu is at index " << baseBodyIndex
					<< ". Calibration data is missing or a method branch failed silently. "
					"No heading correction applied." << reset);
			R_heading = SimTK::Rotation();
			return R_heading;
		}

		// get initial measurement of base imu
		//cout << baseBodyIndex << endl;
		//ROS_DEBUG_STREAM("baseBodyIndex:" << baseBodyIndex );
		for (size_t g= 0; g < staticPoseQuaternions.size();g++)
		{
			//cout << staticPoseQuaternions[g] << endl;
			ROS_INFO_STREAM(green << "Quaternion for imu[" << g << "], ["<< imuBodiesObservationOrder[g] <<"] " << staticPoseQuaternions[g] << reset);
		}
		const auto q0 = staticPoseQuaternions[baseBodyIndex];
		ROS_INFO_STREAM("Basebody q0: "<< q0);

		auto        q0_rotation_matrix =  SimTK::Rotation(q0);
		auto inverseq0_rotation_matrix = ~SimTK::Rotation(q0);

		ROS_INFO_STREAM(magenta << "R_GoGi1 (should be roughly the same as inverse q in the zero heading case): "<< R_GoGi1<<reset);
		ROS_INFO_STREAM(magenta << "R_GiGo1 (actually its ~R_GoGi1 but checking maths because were dumb): "<< ~R_GoGi1<<reset);
		ROS_INFO_STREAM(magenta << "R_GoGi1*R_GiGo1 (this should be the identity, right? but checking maths because were dumb): "<< R_GoGi1*~R_GoGi1<<reset);
		ROS_INFO_STREAM(cyan << "       q0_rotation_matrix: "<<        q0_rotation_matrix<<reset);
		ROS_INFO_STREAM(cyan << "inverseq0_rotation_matrix: "<< inverseq0_rotation_matrix<<reset);
		// all the permutations!
		const auto base_R = R_GoGi1 * SimTK::Rotation(q0);
		//const auto base_R1 = ~R_GoGi1 * SimTK::Rotation(q0);
		//const auto base_R2 = R_GoGi1 * ~SimTK::Rotation(q0);
		//const auto base_R3 = ~R_GoGi1 * ~SimTK::Rotation(q0);
		//const auto base_R4 = SimTK::Rotation(q0) * R_GoGi1;
		//const auto base_R5 = ~SimTK::Rotation(q0) * R_GoGi1;
		//const auto base_R6 = SimTK::Rotation(q0) * ~R_GoGi1;
		//const auto base_R7 = ~SimTK::Rotation(q0) * ~R_GoGi1;

		//const SimTK::Rotation base_R = ~SimTK::Rotation(q0);
		//
		//
		ROS_INFO_STREAM("Okay, what do i think is happening here: R_GoGi1 is the inverse of imu_ref_ori, so if we multiply an imu_ref_ori orientated imu in the front by the rotation of an IMU, this should give us an identity matrix, if they are orientated with a zero heading.\nIf it is at some sort of angle, then this will reflect how much the base IMU should alter the thingymagig. I am going to publish this as a tf with the name base_rotation.");
		ROS_INFO_STREAM("base_R = R_GoGi1 * SimTK::Rotation(q0)" << base_R);

		SimTK::Vec3 trans_base_rotated{1.2,1,1};
		//SimTK::Vec3 trans_base_rotated1{1.2,1+.1,1};
		//SimTK::Vec3 trans_base_rotated2{1.2,1+.2,1};
		//SimTK::Vec3 trans_base_rotated3{1.2,1+.3,1};
		//SimTK::Vec3 trans_base_rotated4{1.2,1+.4,1};
		//SimTK::Vec3 trans_base_rotated5{1.2,1+.5,1};
		//SimTK::Vec3 trans_base_rotated6{1.2,1+.6,1};
		//SimTK::Vec3 trans_base_rotated7{1.2,1+.7,1};

		SimTK::Transform TRotatedBase(base_R, trans_base_rotated);
		//SimTK::Transform TRotatedBase1(base_R1, trans_base_rotated1);
		//SimTK::Transform TRotatedBase2(base_R2, trans_base_rotated2);
		//SimTK::Transform TRotatedBase3(base_R3, trans_base_rotated3);
		//SimTK::Transform TRotatedBase4(base_R4, trans_base_rotated4);
		//SimTK::Transform TRotatedBase5(base_R5, trans_base_rotated5);
		//SimTK::Transform TRotatedBase6(base_R6, trans_base_rotated6);
		//SimTK::Transform TRotatedBase7(base_R7, trans_base_rotated7);
		sameHeader.stamp = ros::Time::now();
		publishTransform("base_rotation", TRotatedBase, sameHeader);
		//publishTransform("base_rotation1", TRotatedBase1, sameHeader);
		//publishTransform("base_rotation2", TRotatedBase2, sameHeader);
		//publishTransform("base_rotation3", TRotatedBase3, sameHeader);
		//publishTransform("base_rotation4", TRotatedBase4, sameHeader);
		//publishTransform("base_rotation5", TRotatedBase5, sameHeader);
		//publishTransform("base_rotation6", TRotatedBase6, sameHeader);
		//publishTransform("base_rotation7", TRotatedBase7, sameHeader);

		// get initial direction from the imu measurement (the axis looking
		// front)
		SimTK::UnitVec3 baseSegmentXheading = base_R(baseHeadingDirection.getAxis());
		if (baseHeadingDirection.getDirection() < 0)
		{
			//this thing here implements the minus sign of the baseHeadingDirection variable which is a parameter we set and it is only used here, it seems.
			baseSegmentXheading = baseSegmentXheading.negate();
		}

		//is y the vertical?
		//
		ROS_INFO_STREAM("baseSegmentXheading with every component::" << baseSegmentXheading);
		//I don't know the right way of doing this
		baseSegmentXheading.set(1, 0); //IS this it?
		auto new_vec = baseSegmentXheading.normalize();
		baseSegmentXheading.set(0,new_vec.get(0));
		baseSegmentXheading.set(1,new_vec.get(1));
		baseSegmentXheading.set(2,new_vec.get(2));

		ROS_INFO_STREAM("baseSegmentXheading with what i think is the vertical component set to zero::" << baseSegmentXheading);
		// get frame of imu body
		const PhysicalFrame* baseFrame = nullptr;
		if (!(baseFrame = model.findComponent<PhysicalFrame>(baseImuName))) {
			ROS_FATAL_STREAM("Could not find imu for base: " << baseImuName);
			THROW_EXCEPTION(
					"Frame of given body name does not exist in the model.");
		}

		// express unit x axis of local body frame to ground frame
		SimTK::Vec3 baseFrameX = SimTK::UnitVec3(1, 0, 0);
		
		// Yes, the base body's frame in ground is generally NOT the identity, and
		// yes, that means there are two headings in play: the sensor's and the
		// model's. The answer to "can I just add them together" is no -- you
		// compare them, and to compare them both have to be projected onto the
		// horizontal plane first, which is what happens below.
		//
		// Note that this comparison is now only a readout. The model's own
		// segment orientation is accounted for properly in calibrateIMUTasks()
		// via the ~R_GoB factor, which is where it always belonged.
		const SimTK::Transform& baseXForm = baseFrame->getTransformInGround(state);


		//publishTransform("baseXForm",baseXForm, sameHeader);	
		SimTK::Vec3 baseFrameXInGround = baseXForm.xformFrameVecToBase(baseFrameX);

		// Project onto the horizontal plane, exactly as was just done above for
		// baseSegmentXheading. BOTH operands of the acos below have to be
		// horizontal unit vectors or the angle is meaningless. This is the same
		// projection-vs-composition error as the original OpenSense heading bug,
		// just on the model side of the comparison instead of the sensor side.
		// (Y is up in OpenSim.)
		baseFrameXInGround[1] = 0;
		if (baseFrameXInGround.norm() < SimTK::SignificantReal)
			ROS_ERROR_STREAM(red << "base body [" << baseImuName << "] has its local X axis vertical "
					"in ground, so its heading is undefined." << reset);
		else
			baseFrameXInGround = baseFrameXInGround.normalize();

		ROS_INFO_STREAM("baseFrameXInGround projected onto the horizontal plane: " << baseFrameXInGround);
		
		

		//publishTransform("opensim_real_base", baseXForm, sameHeader);


		//ROS_YE("baseFrameXInGround" << baseFrameXInGround);

		// compute the angular difference between the model heading and imu
		// heading
		//
		//
		//this is super fishy. let's show this:
		//
		//

		
		angularDifference = acos(~baseSegmentXheading * baseFrameXInGround);

		// compute sign
		auto xproduct = baseFrameXInGround % baseSegmentXheading;
		if (xproduct.get(1) > 0) { angularDifference *= -1; negate=true; }

		ROS_YE("angularDifference: " << angularDifference << " rad ( " << angularDifference/3.14159205*180.0 << " degrees)");
		// set heading rotation (rotation about Y axis)
		R_heading = SimTK::Rotation(angularDifference , SimTK::YAxis);

	} else {
		ROS_WARN("No heading correction is applied. Heading rotation is set to "
				"default");
	}

	///fff.. my angle sign calculation is wrong, so i will use this from opensimrt...
	///this  is awful, i hate it
	//if (negate)
	//{
	//	baseHeadingAngle = -abs(baseHeadingAngle);
	//}
	//else
	//{
	//	baseHeadingAngle = abs(baseHeadingAngle);
	//}

	ROS_INFO_STREAM("heading orientation matrix:\n" << R_heading);
	ros::spinOnce();
	return R_heading;
}

geometry_msgs::Quaternion getAsRosQuaternion(const SimTK::Rotation& R)
{
	SimTK::Quaternion rotation = R.convertRotationToQuaternion();
	geometry_msgs::Quaternion _rotation;
			_rotation.w = rotation[0];
			_rotation.x = rotation[1];
			_rotation.y = rotation[2];
			_rotation.z = rotation[3];
	return _rotation;

}

geometry_msgs::Vector3 getAsRosVec3(const SimTK::Vec3& translation)
{
	geometry_msgs::Vector3 _translation;
			_translation.x = translation[0];
			_translation.y = translation[1];
			_translation.z = translation[2];
	return _translation;


}

geometry_msgs::Transform getAsRosTF(const SimTK::Transform& X_GB)
{
	SimTK::Vec3 translation = X_GB.p();
	SimTK::Rotation R = X_GB.R();
	geometry_msgs::Transform _tf;
	_tf.translation = getAsRosVec3(translation);
	_tf.rotation = getAsRosQuaternion(R);
	return _tf;
}

void IMUCalibrator::publishTransform(const std::string name, const SimTK::Transform X_GB, const std_msgs::Header& header)
{
	geometry_msgs::Transform _tf = getAsRosTF(X_GB);

	geometry_msgs::TransformStamped _tfs;
	_tfs.header = header;
	_tfs.child_frame_id = name;
	_tfs.transform = _tf;

	tb.sendTransform(_tfs);
}

/**
 * Turn the recorded static-pose measurements into the constant sensor-on-segment
 * mounting rotations that InverseKinematics actually wants.
 *
 * The relation SimTK's OrientationSensors assembly condition enforces is
 *
 *     R_GoB(theta) * R_BS  ~=  R_GoS_measured
 *
 * (see InverseKinematics.cpp, addOSensor(..., orientationInB = R_BS, ...)).
 *
 * During the static phase the subject is assumed to be holding the model's
 * DEFAULT pose, so theta == theta_default and R_GoB == imuBodiesInGround[body].R().
 * Solving for the unknown mounting rotation therefore gives
 *
 *     R_BS = ~R_GoB_default * R_GoGi1 * Rotation(q0)
 *
 * The ~R_GoB_default factor is NOT optional. It is the identity only for models
 * whose segment frames happen to be ground-aligned in the default pose (gait2392
 * and friends), which is why dropping it appeared to work for exactly those and
 * silently wrecked every model with rotated segment frames (MoBL-ARMS forearm,
 * anything generated from a URDF).
 *
 * No heading correction is applied here. A heading correction is just a yaw
 * folded into R_GoGi1, so it belongs in the imu_ref_ori TF, and it must appear
 * on the runtime observations (transform()) and here or on neither -- applying
 * it to only one side bakes a spurious yaw into the solved pose.
 */
void IMUCalibrator::calibrateIMUTasks(
		vector<InverseKinematics::IMUTask>& imuTasks) {
	sameHeader.stamp = ros::Time::now();

	if (staticPoseQuaternions.size() != imuTasks.size())
		ROS_ERROR_STREAM(red << "staticPoseQuaternions.size() [" << staticPoseQuaternions.size()
				<< "] != imuTasks.size() [" << imuTasks.size()
				<< "]! This calibration is WRONG. Only the first "
				<< std::min(staticPoseQuaternions.size(), imuTasks.size())
				<< " sensors will be calibrated; the rest keep whatever orientation they "
				"already had." << reset);

	// GUARD: loop to the shorter of the two. The check above used to only warn and
	// then index imuTasks[i] anyway, which is an out-of-bounds WRITE whenever there
	// are more static poses than tasks.
	const size_t n = std::min(staticPoseQuaternions.size(), imuTasks.size());
	for (size_t i = 0; i < n; ++i) {
		const auto& bodyName = imuTasks[i].body; // NOTE: this is the BASE frame name
		const auto& q0 = staticPoseQuaternions[i];

		if (imuBodiesInGround.find(bodyName) == imuBodiesInGround.end())
			ROS_FATAL_STREAM(red << "no default-pose transform stored for body [" << bodyName
					<< "]! R_GoB is being treated as the identity and this calibration WILL be wrong."
					<< reset);

		// sensor orientation in OpenSim ground, as measured during the static pose
		const SimTK::Rotation R_GoS = R_GoGi1 * SimTK::Rotation(q0);

		// body orientation in OpenSim ground, at the model's default pose
		const SimTK::Rotation R_GoB = imuBodiesInGround[bodyName].R();

		// the constant mounting rotation of the sensor on the segment
		const SimTK::Rotation R_BS = ~R_GoB * R_GoS;

		// debug tfs, nudged apart so they are distinguishable in rviz
		SimTK::Vec3 myVec  = imuBodiesInGround[bodyName].p();
		SimTK::Vec3 myVec2 = myVec; myVec2[0] += 0.2;
		publishTransform(bodyName+"R0", SimTK::Transform(R_GoS, myVec),  sameHeader);
		publishTransform(bodyName+"BS", SimTK::Transform(R_BS,  myVec2), sameHeader);

		ROS_DEBUG_STREAM("body [" << bodyName << "]"
				<< "\n R_GoS (measured sensor in ground):\n" << R_GoS
				<< "\n R_GoB (default pose body in ground):\n" << R_GoB
				<< "\n R_BS  (mounting rotation):\n" << R_BS);

		imuTasks[i].orientation = R_BS;
	}
}

void IMUCalibrator::calibrate_ext_signals_sender()
{

	osrt_ros::Float b;
	// I also want to tell every service that they need to start acquiring poses and this needs to be non-blocking!QQ
	ROS_INFO_STREAM("trying to send calibration signals to my fellas imu Quaternion pose average buddies");


	//is this faster? this is 13ms without the spin.
	chrono::high_resolution_clock::time_point t1=chrono::high_resolution_clock::now() ;
	std::vector<std::thread> tts;
	for(autosrv& a_calib_src:calib_srv)
	{
		auto ff = [&](){a_calib_src.calib();};
		tts.push_back(std::thread(ff));
	}
	for (std::thread& t : tts)
		if (t.joinable())
			t.join();
	chrono::high_resolution_clock::time_point t2=chrono::high_resolution_clock::now() ;

	ROS_YE("multiple srv call duration in ms:"<<magenta<<chrono::duration_cast<chrono::milliseconds>(t2-t1).count());

	if(!ext_heading_srv.exists())
		ROS_YE("srv:" <<ext_heading_srv.getService() << " does not exist!!!!!!");
	else
	{
		ROS_INFO_STREAM("trying to call" << ext_heading_srv.getService());
		chrono::high_resolution_clock::time_point tsrv1=chrono::high_resolution_clock::now() ;
		ext_heading_srv.call(b);
		chrono::high_resolution_clock::time_point tsrv2=chrono::high_resolution_clock::now() ;
		ros::spinOnce();
		chrono::high_resolution_clock::time_point tsrv3=chrono::high_resolution_clock::now() ;


		ROS_YE("blames:: service call:"<<green<<chrono::duration_cast<chrono::milliseconds>(tsrv2-tsrv1).count()<<" spinning once"<<chrono::duration_cast<chrono::milliseconds>(tsrv3-tsrv2).count());

		ROS_INFO_STREAM("got heading angle " << b.response.data);
		baseHeadingAngle = -b.response.data*180.0/3.141592;
		ROS_INFO_STREAM("setting heading angle to minus that much, " << baseHeadingAngle);

	}
	chrono::high_resolution_clock::time_point t3=chrono::high_resolution_clock::now() ;

	ROS_YE("heading srv call duration in ms:"<<cyan<<chrono::duration_cast<chrono::milliseconds>(t3-t2).count());

}

void IMUCalibrator::recordNumOfSamples(const size_t& numSamples) {
	if(send_start_signal_to_external_heading_calibrator)
		calibrate_ext_signals_sender();
	else
		impl->recordNumOfSamples(numSamples);
	computeAvgStaticPoseCommon();
}

void IMUCalibrator::recordTime(const double& timeout) {
	if(send_start_signal_to_external_heading_calibrator)
		calibrate_ext_signals_sender();
	else
		impl->recordTime(timeout);
	computeAvgStaticPoseCommon();
}

SimTK::Quaternion IMUCalibrator::getAvgQuaternionFromTopics(std::string imu_name)
{
	SimTK::Quaternion si_q;
	geometry_msgs::QuaternionConstPtr res_q;
	geometry_msgs::Quaternion q;
	auto topic_name = nhandle.resolveName(imu_name+"/avg_pose");
	ROS_DEBUG_STREAM("trying to read: " <<topic_name);
	res_q = ros::topic::waitForMessage<geometry_msgs::Quaternion>(imu_name+"/avg_pose", nhandle);
	if (res_q)
	{
		ROS_DEBUG_STREAM(">>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>");
		ROS_DEBUG_STREAM(res_q);
		q = *res_q;
		ROS_DEBUG_STREAM(q);
		ROS_DEBUG_STREAM(">>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>");
		//the first term of the quaternion is w in simtk:
		//Quaternion 	( 		 )  	[inline]

		//Default constructor produces the ZeroRotation quaternion [1 0 0 0] (not NaN - even in debug mode). 

		si_q[0] = q.w;
		si_q[1] = q.x;
		si_q[2] = q.y;
		si_q[3] = q.z;
		//ROS_WARN("quaternionAverage disabled using one, which should result in an identity matrix rotation");
	}
	else
		ROS_FATAL_STREAM("failed to read avg_pose response for imu" << imu_name);
	return si_q;
}

SimTK::Quaternion IMUCalibrator::getAvgQuaternionFromTF(std::string imu_resolved_name)
{
	SimTK::Quaternion si_q;

	try
	{
		// NOTE: the lookup itself has to live INSIDE the try, it is the thing
		// that throws.
		auto imu_calib_from_tf = tfBuffer.lookupTransform(imu_resolved_name+"_imu",imu_resolved_name,ros::Time(0));

		geometry_msgs::Quaternion q = imu_calib_from_tf.transform.rotation;
		ROS_DEBUG_STREAM(q);
		//the first term of the quaternion is w in simtk:
		//Default constructor produces the ZeroRotation quaternion [1 0 0 0] (not NaN - even in debug mode).
		// NOTE: do NOT redeclare si_q here. It used to shadow the outer one, so
		// this function silently returned the identity quaternion every time.
		si_q[0] = q.w;
		si_q[1] = q.x;
		si_q[2] = q.y;
		si_q[3] = q.z;
	}
	catch (tf2::TransformException &ex)
	{
		ROS_ERROR("Error getting transform from %s:%s",imu_resolved_name.c_str(), ex.what());
	}

	return si_q;
}

void IMUCalibrator::computeAvgStaticPoseCommon()
{
	std::string the_method;
	string tf_prefix;
	nhandle.param<string>("the_method",the_method,"old");
	ROS_INFO_STREAM("Now calculating average static pose");
	// the "topics" and "services" branches below push_back, so without this a
	// second calibration grows the vector to 2N and calibrateIMUTasks() then
	// indexes imuTasks[i] out of bounds.
	staticPoseQuaternions.clear();
	switch(hash_djb2a(the_method)) {
		case "old"_sh:
			std::cout << "You entered \'old\'\n";
			ROS_INFO("This is the default method, using the oldest version of averaging quaternions and internal heading.");
			staticPoseQuaternions = impl->computeAvgStaticPose();
			break;
		case "topics"_sh:
			std::cout << "You entered \'topics\'\n";
			ROS_INFO_STREAM("using external averagingMethod!");
			ROS_WARN_STREAM("This is supposed to be the most accurate version with SVD and internal heading*, but it is slower. (It was not tested with external heading.)");
			publishCalibrationData(); //TODO: this can be parallelised
			for (auto imu_name:imuBodiesObservationOrder) //TODO: this can also be parallelised
			{
				SimTK::Quaternion si_q;
				si_q = getAvgQuaternionFromTopics(imu_name);
				staticPoseQuaternions.push_back(si_q);
			}
			break;
		case "services"_sh:
			std::cout << "You entered \'services\'\n";
			ROS_WARN_STREAM("This is an attempt at making all the calculations external and it sort of works. But use at own risk. Also it requires external heading, or it won't be able to find the TF.");
			nhandle.param<string>("tf_prefix",tf_prefix,"");
			for (auto imu_name:imuBodiesObservationOrder) //TODO: this can also be parallelised
			{
				SimTK::Quaternion si_q;
				string imu_resolved_name = tf_prefix+imu_name; 
				si_q = getAvgQuaternionFromTF(imu_resolved_name);
				staticPoseQuaternions.push_back(si_q);
			}
			break;
		default:
			ROS_ERROR_STREAM("Method "<<the_method <<" not recognized!\n");
			break;
	}
	//let's compare the results:
	if (false)
	{
		ROS_DEBUG_STREAM("\n===== Calibration results ============");
		auto old_avg_response_list = impl->computeAvgStaticPose();
		for (size_t i=0;i<old_avg_response_list.size(); i++)
		{
			ROS_DEBUG_STREAM("\nOLD:" <<old_avg_response_list[i] <<
					"\nNEW;" <<staticPoseQuaternions[i]);
		}
		ROS_DEBUG_STREAM("\n===== End of calibration results =====");
	}
}

void IMUCalibrator::publishCalibrationData()
{
	auto ans =  impl->getTableData();
	int i = 0;
	for (auto qT:ans) // so this is not necessarily aligned. it should be a smart thing, like a map
	{ 	
		ROS_DEBUG_STREAM("iterating over imus[" << i << "]: " << imuBodiesObservationOrder[i] );
		geometry_msgs::PoseArray p_msg;

		for (auto q:qT)
		{
			//ROS_DEBUG_STREAM_ONCE("Is the Quaternion order correct? The SVD method shouldn't care, but maybe that could be wrong?");
			geometry_msgs::Pose pp;
			pp.orientation.w = q[0];
			pp.orientation.x = q[1];
			pp.orientation.y = q[2];
			pp.orientation.z = q[3];
			ROS_DEBUG_STREAM(q);
			p_msg.poses.push_back(pp);
		}
		pub[i].publish(p_msg);
		i++;
	}


}
