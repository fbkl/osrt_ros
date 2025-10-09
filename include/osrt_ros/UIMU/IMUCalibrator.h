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
 *
 * @file IMUCalibrator.h
 *
 * @brief Calibration of the IMU data required before using the IK module.
 *
 * @author Filip Konstantinos <filip.k@ece.upatras.gr>
 */
#pragma once

#include "InverseKinematics.h"
#include "UIMUInputDriver.h"
#include "Utils.h"
#include "ros/service_client.h"
#include "std_srvs/Empty.h"
#include "tf/transform_broadcaster.h"
#include "tf2_ros/buffer.h"
#include "tf2_ros/transform_listener.h"
#include <Simulation/Model/Model.h>
#include <boost/function/function_fwd.hpp>
#include <string>
#include <string_view>
#include <type_traits>
#include <tf2_ros/transform_broadcaster.h>

#include <ros/ros.h>
#include <vector>
namespace OpenSimRT {

	class autosrv
	{
		public:
			ros::ServiceClient calib_client;
			void calib()
			{
				auto a = std_srvs::Empty();
				if(!calib_client.exists())
					ROS_WARN_STREAM("srv:" << calib_client.getService() << " does not exist!!!!!!");
				else
				{
					ROS_INFO_STREAM("trying to call" << calib_client.getService());
					calib_client.call(a);
					ros::spinOnce();
				}
			}
			std::string imu;

	};

	/**
	 * Class used to calibrate IMUData for solving the IK module with orientation
	 * data. Initially, it calibrates the IK tasks based on the data from the
	 * initial pose of the subject that corresponds to the osim model pose. The
	 * subject should stand in pose for a number of seconds or a specific number of
	 * samples acquired from stream. After that the calibrator is used to transform
	 * the received data in the dynamic simulation based on the initial
	 * measurements. The transformations applied are the transformation of the
	 * ground-reference of the sensors to the OpenSim's ground reference frame, and
	 * the a heading transformation of the subject so that the heading axis of the
	 * subject coinsides with anterior axis in OpenSim (X-axis).
	 */
	class  IMUCalibrator {
		public:
			std::vector<SimTK::Quaternion> staticPoseQuaternions; // static pose data
			tf::TransformBroadcaster tb;
			tf2_ros::Buffer tfBuffer;
			tf2_ros::TransformListener tfListener;
			std::vector<ros::Publisher> pub;
			ros::NodeHandle nhandle;
			ros::ServiceClient ext_heading_srv;

			std::string debug_reference_frame;
			bool old_method_of_getting_averaged = false;
			bool send_start_signal_to_external_heading_calibrator = false;
			long baseBodyIndex;
			//std::vector<ros::Subscriber> avg_pose_subs;

			SimTK::Quaternion getAvgQuaternionFromTopics(std::string imu_name);
			SimTK::Quaternion getAvgQuaternionFromTF(std::string imu_resolved_name);

			double baseHeadingAngle=0;

			std::vector<autosrv> calib_srv;
			void calibrate_ext_signals_sender();
			/**
			 * Construct a calibrator object. The constructor uses the Type-Erasure
			 * pattern/idiom that is based on automatic type deduction to remove the
			 * dependency on the IMUData type of the input driver. The only requirement
			 * is that the IMUData type of the driver MUST have a member function
			 * `getQuaternion()` to receive quaternion estimations for each IMU sensor.
			 */
			IMUCalibrator(const OpenSim::Model& otherModel,
					const UIMUInputDriver* const driver,
					const std::vector<std::string>& observationOrder)
				// instantiate the DriverErasure object by forwarding the input
				// driver in its contructor.
				: tfListener(tfBuffer), model(*otherModel.clone()),
				impl(new DriverErasure(
							std::forward<const UIMUInputDriver* const>(driver))) {
					setup(observationOrder);
				}

			SimTK::Rotation setGroundOrientationFromTF(const std::string& tfname);
			/**
			 * Set the rotation sequence of the axes (in degrees) that form the
			 * transformation between the sensor's reference frame and the OpenSim's
			 * ground reference frame.
			 */
			SimTK::Rotation setGroundOrientationSeq(const double& xDegrees,
					const double& yDegrees,
					const double& zDegrees);
			/**0
			 * Compute the transformation for the heading correction based on the
			 * measurements acquired during the static phase.
			 */
			SimTK::Rotation computeHeadingRotation(const std::string& baseImuName,
					const std::string& imuDirectionAxis);
			/**
			 * Calibrate the IK IMUTasks prior the construction of the IK module.
			 */
			void calibrateIMUTasks(std::vector<InverseKinematics::IMUTask>& imuTasks);

			/**
			 * Time duration (in seconds) of the static phase during calibration.
			 */
			void recordTime(const double& timeout);

			/**
			 * Time duration (in number number of acquired samples) of the static
			 * phase during calibration.
			 */
			void recordNumOfSamples(const size_t& numSamples);

			/**
			 * Calibrate NGIMU data acquired from stream and create the IK input.
			 */
			void publishCalibrationData();
			void computeAvgStaticPoseCommon();
			SimTK::Array_<SimTK::Rotation> transform(const std::vector<UIMUData>& imuData) {
				long i=0;
				SimTK::Array_<SimTK::Rotation> imuObservations;
				for (const auto& data : imuData) {
					const auto& q = data.getQuaternion();
					SimTK::Rotation R;
					//if (i == baseBodyIndex)
					R = R_heading *R_GoGi1* SimTK::Rotation(q);
					//else
					//	R = R_GoGi1 * ~SimTK::Rotation(q);
					imuObservations.push_back(R);
					i++;
				}
				return imuObservations;
			}
			SimTK::Rotation R_GoGi1;    // ground-to-ground transformation
			void publishTransform(const std::string name, const SimTK::Transform X_GB, const std_msgs::Header& header);
    			std_msgs::Header sameHeader;
		

		private:
			bool externalAveragingMethod = false;
			/**
			 * Type erasure on imu InputDriver types. Base class. Provides an interface
			 * for the functionality of derived classes.
			 *
			 *TODO: IHATETHISSOMUCH!!!! REMOVE
			 *
			 */
			class DriverErasure {
				public:
					/* some logic for checking if calibration data was received! */
					std::mutex calibration_mtx;
					std::condition_variable cv_calibration_done;
					bool data_ready = false;

					DriverErasure(const UIMUInputDriver* const driver) : m_driver(driver) {
					}
					virtual std::vector<std::vector<SimTK::Quaternion>> getTableData() {
						std::vector<std::vector<SimTK::Quaternion>> table;
						int n = initIMUDataTable.size();    // num of recorded frames
						int m = initIMUDataTable[0].size(); // num of imu devices
						for (int j = 0; j< m ; ++j)
						{
							std::vector<SimTK::Quaternion> thisImuTable;
							for (int i = 0; i< n; ++i)
							{	
								thisImuTable.push_back(initIMUDataTable[i][j].getQuaternion());
							}
							table.push_back(thisImuTable);
						}	
						return table;
					}
					virtual void recordTime(const double& timeout)  {
						{
							std::lock_guard<std::mutex> lock(calibration_mtx);
							initIMUDataTable.clear(); // if you want to do something fancy, remove this and then just create another service to allow to record multiple calibrations, for instance. no idea if this makes any sense though.
							std::cout << "Recording Static Pose..." << std::endl;
							const auto start = std::chrono::steady_clock::now();
							while (std::chrono::duration_cast<std::chrono::seconds>(
									std::chrono::steady_clock::now() - start)
								.count() < timeout) {
							// get frame measurements. `getData()` is common to all input
							// drivers
								initIMUDataTable.push_back(m_driver->getData());
							}
							data_ready = true;
						}
						cv_calibration_done.notify_one();
					}

					virtual void recordNumOfSamples(const size_t& numSamples)  {
						{
							std::lock_guard<std::mutex> lock(calibration_mtx);
							initIMUDataTable.clear();
							std::cout << "Recording Static Pose..." << std::endl;
							size_t i = 0;
							while (i < numSamples) {
								// get frame measurements. `getData()` is common to all input
								// drivers
								std::cout << "am i stuck here?" << std::endl;
								std::vector<UIMUData> aa = m_driver->getData();
								for(auto uimudata_i:aa)
									std::cout << uimudata_i << ", ";
								std::cout <<  std::endl;
								initIMUDataTable.push_back(m_driver->getData());
								++i;
							}
							data_ready = true;
						}
						cv_calibration_done.notify_one();
					}

					/**
					 * Compute average of 3D rotations. Given a list of IMUData containing
					 * the quaternion measurements, computes the average quaternion error
					 * from the first sample and adds it back to the first sample to return
					 * the average quaternions.
					 *
					 * Source:
					 * https://math.stackexchange.com/questions/1984608/average-of-3d-rotations
					 */
					virtual std::vector<SimTK::Quaternion> computeAvgStaticPose()  {
						std::unique_lock<std::mutex> lock(calibration_mtx);
						while(!data_ready)
						{
							cv_calibration_done.wait(lock);
						}

						int n = initIMUDataTable.size();    // num of recorded frames
						int m = initIMUDataTable[0].size(); // num of imu devices
						auto avgQuaternionErrors =
							std::vector<SimTK::Quaternion>(m, SimTK::Quaternion());
						auto avgQuaternions(avgQuaternionErrors);

						ROS_INFO_STREAM("OLD: Using norm average of rotations");


						// Quaternion product for each imu
						for (int j = 0; j < m; ++j)
							for (int i = 0; i < n; ++i) {
								// NOTE: requires `.getQuaternion()` function of the IMUData
								// type.
								const auto& q = initIMUDataTable[i][j].getQuaternion();
								avgQuaternionErrors[j] = avgQuaternionErrors[j] * (~q * q);
							}

						// convet to axis angle, devide with n, and convert back to
						// quaternions
						for (auto& q : avgQuaternionErrors) {
							q.setQuaternionFromAngleAxis(
									q.convertQuaternionToAngleAxis().scalarDivide(
										double(n)));
						}

						// add the error back to the first sample
						for (int j = 0; j < m; ++j) {
							// NOTE: requires `.getQuaternion()` function of the IMUData
							// type.
							const auto& q0 = initIMUDataTable[0][j].getQuaternion();
							const auto& qe = avgQuaternionErrors[j];
							avgQuaternions[j] = qe * q0;
						}

						data_ready = false; // seems weird, but we need to reset it so that we can calibrate again
						return avgQuaternions;
					}
					virtual void clearCalibration()  
					{
						initIMUDataTable.clear();
					}
				private:
					SimTK::ReferencePtr<const UIMUInputDriver> m_driver;
					std::vector<std::vector<UIMUData>> initIMUDataTable;
			};

			/**
			 * Supplamentary method used in the IMUCalibrator constructor.
			 */
			void setup(const std::vector<std::string>& observationOrder);

			OpenSim::Model model;
			SimTK::State state;
			std::unique_ptr<DriverErasure>
				impl; // pointer to DriverErasureBase class
			std::map<std::string, SimTK::Transform> imuBodiesInGround; // R_GB per body
			std::vector<std::string> imuBodiesObservationOrder;       // imu order
			SimTK::Rotation R_heading; // heading correction
	};
} // namespace OpenSimRT
