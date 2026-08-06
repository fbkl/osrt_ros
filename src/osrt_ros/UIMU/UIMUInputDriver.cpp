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
#include "osrt_ros/UIMU/UIMUInputDriver.h"
#include "Exception.h"
#include <iostream>
#include <string>
#include <vector>
#include "ros/node_handle.h"
#include "ros/ros.h"
#include "osrt_ros/UIMU/TfServer.h"
#include "osrt_ros/UIMU/ImuSubscriberServer.h"
#include "osrt_ros/UIMU/CometaServer.h"

using namespace OpenSimRT;
;



enum server_code {
	eTf,
	eSubscriber,
	eCometa,
	eUnknown,
};

server_code hashit (std::string const& inString) {
	if (inString == "cometa") return eCometa;
	if (inString == "subscriber") return eSubscriber;
	if (inString == "tf") return eTf;
	return eUnknown;
}


//TODO: this is already threaded because ROS is threaded. Remove this thread. remove the table which just grown and no one reads, since you have it inside a thread!!!!!!!!!!!!!!!!

UIMUInputDriver::UIMUInputDriver(std::vector<std::string> imuObservationOrder, const std::string tf_frame_prefix, const double& sendRate)
	: rate(sendRate), terminationFlag(false) {
		ROS_INFO_STREAM("Starting UIMUInputDriver interface.");
		imu_names = imuObservationOrder;
		if (imuObservationOrder.size() == 0)
			ROS_FATAL("no imuObservationOrder provided. why didn't you?");
		std::string wholeObservationOrderStr;
		for(auto imu_name:imuObservationOrder)
		{
			wholeObservationOrderStr+=imu_name+",";
		}
		ROS_INFO_STREAM("Using observationOrder of:" << wholeObservationOrderStr);


		ros::NodeHandle nh("~");
		std::string world_tf_reference;
		nh.param<std::string>("imu_tf_world_reference", world_tf_reference, "/map");

		//how to set enum though
		std::string orientation_server_type;
		nh.param<std::string>("orientation_server_type", orientation_server_type, "tf");

		int server_type = hashit(orientation_server_type); 

		switch (server_type){
			//case tf_server
			case eTf:
				{
					ROS_INFO_STREAM("Using tf_frame_prefix: " << tf_frame_prefix);
					server = new TfServer(imuObservationOrder, tf_frame_prefix);
					//i dont need this...
					auto bbb = dynamic_cast<TfServer*>(server);
					if (bbb)
					{
						bbb->set_world_reference(world_tf_reference);
					}
					break;
				}
				//case imusubscriber_server
			case eSubscriber:
				{	
					ROS_INFO_STREAM("Using imu subscriber interface... ");
					server = new ImuSubscriberServer();
					auto ccc = dynamic_cast<ImuSubscriberServer*>(server);
					if (ccc)
					{
						ccc->set_world_reference(world_tf_reference);
					}
					break;
				}
				//case cometa server
			case eCometa:
				{
					ROS_ERROR("cometa interface not implemented");
					break;
				}
			case eUnknown:
				{
					ROS_ERROR_STREAM("unknown server type:" << orientation_server_type);
					break;

				}
		}
	}

// i maybe want to start the server!

UIMUInputDriver::~UIMUInputDriver() { t.join(); }

void UIMUInputDriver::startListening() {
	static auto f = [&]() {
		try {
			int i = 0;
			std::cout << "Rate: " << rate << std::endl ;
			ROS_INFO_STREAM("Rate: " << rate ) ;
			for (;;) {
				if (shouldTerminate())
					THROW_EXCEPTION("shouldTerminate called. File stream terminated.");
				{
					std::lock_guard<std::mutex> lock(mu);
					// get something from the udp stream
					//TODO: rosdebug
					ROS_DEBUG_STREAM( "Acquired lock. receiving.");
					if (! server->receive()){
						ROS_INFO_STREAM( "Received goodbye message!" );
						terminationFlag = true;
						break;
					}
					std::vector<double> output = server->output;
					ROS_DEBUG_STREAM("Received.");

					// there is no table, so this will be empty
					//std::stringstream s(server.buffer);
					//time = output[0]; // probably a double
					//SimTK::readUnformatted<SimTK::Vector>(s, frame);// I will keep
					//OpenSim::TimeSeriesTable table;

					ROS_DEBUG_STREAM("read input size from OrientationProvider" << output.size());
					//table.appendRow(output[0], output.begin()+1, output.end()); // superflex!
					//ROS_DEBUG_STREAM( "added to table alright." );
					//table.getMatrix()[0]; // OpenSim::TimeSeriesTable
					//this will crash because table was not initialized.
					//time = table.getIndependentColumn()[i];
					time = output[0];
					SimTK::RowVector newFrame(output.size()-1); 
					for (size_t j= 0; j<output.size()-1;j++)
						newFrame[j] = output[j+1];
					frame =  newFrame;
					//frame = table.getMatrix()[i];
					ROS_DEBUG_STREAM("FRAME" << frame );
					newRow = true;
					i++;
				}
				cond.notify_one();

				// artificial delay, if I remove this it gets slower. if you want to remove this really, you have to remove this whole thread. 
				std::this_thread::sleep_for(std::chrono::milliseconds(
							static_cast<int>(1 / rate * 1000)));
			}
			terminationFlag = true;
			cond.notify_one();

		} catch (const std::exception& e) {
			std::cout << "Failed in acquiring thread." << e.what() << std::endl;

			terminationFlag = true;
			cond.notify_one();
			return;
		}
	};
	t = std::thread(f);
	// this is threaded, so this guy should work too!
	ROS_INFO_STREAM( "Acquisition thread created!");
}

bool UIMUInputDriver::shouldTerminate() {
	return terminationFlag.load();
}

void UIMUInputDriver::shouldTerminate(bool flag) {
	terminationFlag = flag;
	cond.notify_one();
}

UIMUInputDriver::IMUDataList
UIMUInputDriver::fromVector(const SimTK::Vector& v) const {
	IMUDataList list;
	UIMUData data;
	int n = UIMUData::size();

	//std::cout << v << std::endl;
	//std::cout << "size of UIMUData: " << n  << std::endl;
	//std::cout << "size of v: " << v.size() << std::endl;

	for (int i = 0; i < v.size(); i += n) {
		data.fromVector(v(i, n));
		list.push_back(data);
		//i have 8 right now so this should tell me 8
		//std::cout << i << "number of vectors I pushed back" << std::endl;
	}
	return list;
}

UIMUInputDriver::IMUDataList
UIMUInputDriver::getData() const {
	std::unique_lock<std::mutex> lock(mu);
	cond.wait(lock,
			[&]() { return (newRow == true) || terminationFlag.load(); });
	newRow = false;
	ROS_DEBUG_STREAM("frame inside getData (used for calibration): " << frame);
	return fromVector((~frame).getAsVector());
}

std::pair<double, std::vector<UIMUData>> UIMUInputDriver::getFrame() {
	auto temp = getFrameAsVector();
	return std::make_pair(temp.first, fromVector(temp.second));
}

std::pair<double, SimTK::Vector> UIMUInputDriver::getFrameAsVector() const {
	std::unique_lock<std::mutex> lock(mu);
	cond.wait(lock,
			[&]() { return (newRow == true) || terminationFlag.load(); });
	newRow = false;

	return std::make_pair(time, (~frame).getAsVector());
}

OpenSim::TimeSeriesTable UIMUInputDriver::initializeLogger() const {
	std::vector<std::string> suffixes = {
		"_q1",       "_q2",       "_q3",      "_q4",        "_ax",
		"_ay",       "_az",       "_gx",      "_gy",        "_gz",
		"_mx",       "_my",       "_mz",      "_barometer", "_linAcc_x",
		"_linAcc_y", "_linAcc_z", "_altitude"};

	return setLogger(suffixes);
}

OpenSim::TimeSeriesTable UIMUInputDriver::initializeCalibrationValuesLogger() const {
	std::vector<std::string> suffixes = {
		"_q1",       "_q2",       "_q3",      "_q4"};

	return setLogger(suffixes);
}

OpenSim::TimeSeriesTable UIMUInputDriver::setLogger(std::vector<std::string> suffixes ) const 
{
	// create column names for each combination of imu names and measurement
	// suffixes
	std::vector<std::string> columnNames;
	for (const auto& imu : imu_names) {
		for (const auto& suffix : suffixes) {
			columnNames.push_back(imu + suffix);
		}
	}

	// return table
	OpenSim::TimeSeriesTable q;
	q.setColumnLabels(columnNames);
	return q;
}

