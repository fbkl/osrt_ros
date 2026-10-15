/**
 * @author      : $USER ($USER@frkle-Predator-PT515-52)
 * @file        : pointCalibrator.h
 * @created     : Thursday Sep 17, 2026 13:57:41 UTC
 */

#ifndef POINTCALIBRATOR_H
#define POINTCALIBRATOR_H
#include "osrt_ros/UIMU/pointGetters.h"

class PointCalibrator
{
	public:
		PointCalibrator(OpenSim::Model* model_, GetPoint* pointGetter_): 
			model(model_),
			pointGetter(pointGetter_)
		{
			// so we have the order and it is the same because it is the same freaking object, so that's cool
		



		};
		std::vector<TransObs> calibSamples;
		
		SimTK::Array_<SimTK::Vec3> mt;

		// Wait for `numSamples` samples the point getter says are actually new.
		//
		// The comment that used to live here said get_new_data() blocks and always
		// returns new data. That stopped being true when the condition_variable wait
		// in GetPoint was commented out to protect the ik rate: it now returns
		// whatever is in the buffer immediately. Called straight after
		// startListening(), as recordCalibrationPose does, that buffer is still the
		// default-constructed frame -- an EMPTY array -- and all ten "samples" were
		// the same empty thing. computeAvgStaticPose then indexed [0] on it and took
		// the process down with a SimTK out-of-range at Array.h:555.
		//
		// Returns false if it gave up, so the caller can skip the calibration rather
		// than average nothing.
		bool recordNumOfSamples(const size_t& numSamples, double timeout_s = 5.0)
		{
			ros::Rate poll(200);
			const ros::Time deadline = ros::Time::now() + ros::Duration(timeout_s);
			while (calibSamples.size() < numSamples && ros::ok())
			{
				if (ros::Time::now() > deadline)
				{
					ROS_ERROR_STREAM("point calibration gave up after " << timeout_s
							<< " s with only " << calibSamples.size() << " of "
							<< numSamples << " samples. Is anything publishing the "
							"marker positions?");
					return false;
				}
				auto sample = pointGetter->get_new_data();
				// .second is the point getter's "this is a fresh sample" flag. It only
				// became trustworthy once data_stale stopped being cleared on every
				// poll tick; before that it meant "the poller ran".
				if (!sample.second || sample.first.first.empty())
				{
					poll.sleep();
					continue;
				}
				calibSamples.push_back(sample.first);
			}
			return calibSamples.size() == numSamples;
		}

		// false means mt was not computed and must not be read.
		bool computeAvgStaticPose()
		{
			//oh god TransObs is a pair of lists of vec3, accuracy
			//and this is supposedly in the order of mmList, i think, from the pointGetter
			if (calibSamples.empty())
			{
				ROS_ERROR("point calibration has no samples, refusing to average nothing");
				return false;
			}

			auto numSamples = calibSamples.size();
			auto numSensors = calibSamples[0].first.size();
			if (numSensors == 0)
			{
				ROS_ERROR("point calibration samples carry no markers at all");
				return false;
			}

			std::vector<size_t> counts(numSensors,numSamples);
			SimTK::Array_<SimTK::Vec3> _mt(numSensors, SimTK::Vec3(0));
			for (const auto& isamp:calibSamples)
			{
				SimTK::Array_<SimTK::Vec3> actualObservations = isamp.first;
				SimTK::Array_<SimTK::Real> accuracyOfObservations = isamp.second;
				// bound by the sample, not by markerList: those two disagreeing is
				// exactly how you walk off the end of an array.
				if (actualObservations.size() != numSensors ||
				    accuracyOfObservations.size() != numSensors)
				{
					ROS_ERROR_STREAM("point calibration sample has "
							<< actualObservations.size() << " positions and "
							<< accuracyOfObservations.size() << " accuracies, expected "
							<< numSensors << " of each. Skipping it.");
					for (size_t i = 0; i < numSensors; ++i)
						counts[i]--;
					continue;
				}
				for (size_t i = 0; i < numSensors; ++i)
				{
					//claude wants it weighed.. i also hope that vector scalar multiplication are correctly defined
					if (!std::isnan(accuracyOfObservations[i]))
						_mt[i]+=actualObservations[i]*accuracyOfObservations[i]; 
					else // it is a nan
						counts[i]--;
				}
			}
			for (size_t i = 0; i < numSensors; ++i)
			{
				if (counts[i] ==0) 
					ROS_ERROR_STREAM("marker " << (i < pointGetter->markerNames.size() ?
								pointGetter->markerNames[i] : std::to_string(i))
							<< " has zero valid observations!!!");
				else
					_mt[i]/=counts[i];
			}
			mt = _mt;
			return true;
		}

	OpenSim::Model* model;
	GetPoint* pointGetter;
	//something to save the observation order maybe the state?
};

#endif /* end of include guard POINTCALIBRATOR_H */

