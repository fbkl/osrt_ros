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
 * @file UIMUInputDriver.h
 *
 * @brief Concrete implementation of the IMUInputDriver to stream NGIMU data
 * from file in a separate thread.
 *
 * @author Filip Konstantinos <filip.k@ece.upatras.gr>
 */
#pragma once
#include "UIMUData.h"
#include <Common/TimeSeriesTable.h>
#include <condition_variable>
#include <thread>
#include "OrientationProvider.h"
#include <vector>
#include "CircularBuffer.h"

#include <map>
#define CIRCULAR_BUFFER_SIZE 1

namespace OpenSimRT {
template <typename T> class ListenerAdapter;

/**
 * @brief xio NGIMU Input driver for streaming data from file.
 */
class  UIMUInputDriver {
    friend class ListenerAdapter<UIMUData>;
 public:
    /**
     */
    using IMUDataList = std::vector<UIMUData>;
    UIMUInputDriver(const double& sendRate = 5);
    UIMUInputDriver(const int port,
                             const double& sendRate);
    UIMUInputDriver(const std::vector<std::string> imuObservationOrder, const std::string tf_frame_prefix = "no_prefix" ,const double& sendRate = 5);
    ~UIMUInputDriver(); // dtor

    /**
     * Implements the startListening of the base class. Create a thread that
     * streams the data from file at a constant rate.
     */
    virtual void startListening();

    /**
     * Determine if the stream from file has ended.
     */
    bool shouldTerminate();

    /**
     * Set the termination flag;
     */
    void shouldTerminate(bool flag);

    /**
     * Get data from file as a list of UIMUData. Implements the stopListening
     * of the base class.
     */
    virtual IMUDataList getData() const;

    /**
     * Get data from file as a std::pair containing the time and all the sensor
     * values from the table as a std::vector<UIMUData>. (i.e., time and row
     * from table)
     */
    std::pair<double, IMUDataList> getFrame();

    /**
     * Get data from file as a std::pair containing the time and all the sensor
     * values from the table as a SimTK::Vector. (i.e., time and row from table)
     */
    std::pair<double, SimTK::Vector> getFrameAsVector() const;

    // Using orientation provider pointer to choose server implementation
    OrientationProvider* server;

    OpenSim::TimeSeriesTable initializeLogger() const;
    OpenSim::TimeSeriesTable setLogger(std::vector<std::string> suffixes) const;
    OpenSim::TimeSeriesTable initializeCalibrationValuesLogger() const;

    // buffers
    SimTK::RowVector frame;
    double time;

    std::vector<std::string> imu_names;

 protected:
    IMUDataList fromVector(const SimTK::Vector&) const;

    // hide it from public since it does nothing
    void stopListening() {}
    /**
     */
    std::vector<std::shared_ptr<ListenerAdapter<UIMUData>>> listeners;

    /**
     * A map with thread-safe buffers to store IMU data from each port.
     */
    mutable std::map<int,
                     std::unique_ptr<CircularBuffer<CIRCULAR_BUFFER_SIZE, UIMUData>>>
            buffer;

 private:
   //OpenSim::TimeSeriesTable table;
    //std::vector<double> output;

    double rate;


    // thread related variables
    std::thread t;
    mutable std::mutex mu;
    mutable std::condition_variable cond;
    std::atomic_bool terminationFlag;
    mutable bool newRow = false;
};
} // namespace OpenSimRT

