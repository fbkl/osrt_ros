#include "osrt_ros/UIMU/UIMUnode.h"



void UIMUnode::get_params()
{
	ros::NodeHandle nh("~");
	nh.param<std::string>("tf_frame_prefix",tf_frame_prefix,"not_set");
	// imu calibration settings

	nh.param<std::string>("imu_direction_axis", imuDirectionAxis, "");

	nh.param<std::string>("imu_base_body", imuBaseBody, "");

	nh.param<double>("imu_ground_rotation_x", xGroundRotDeg1, 0.0);
	nh.param<double>("imu_ground_rotation_y", yGroundRotDeg1, 0.0);
	nh.param<double>("imu_ground_rotation_z", zGroundRotDeg1, 0.0);

	nh.param<bool>("use_position_markers", usePositionMarkers, false);
	nh.param<bool>("use_orientation_markers", useOrientationMarkers, true);


	nh.getParam("imu_observation_order", imuObservationOrder);
	if (imuObservationOrder.size() == 0 && useOrientationMarkers) 
	{
		ROS_FATAL("IMU observation order not defined!");
		throw(std::invalid_argument("imuObservationOrder not defined."));
	}
	else
	{
		for(const auto& a:imuObservationOrder) ROS_INFO_STREAM(a);
	}

	ROS_INFO_STREAM("Adding tf_frame_prefix [" << tf_frame_prefix<< "] to tfs to be read.");


	// driver send rate
	nh.param<double>("rate", rate, 0.0);
	r = new ros::Rate(rate);
	nh.param<bool>("visualise", visualiseIt, true);
	// subject data
	nh.param<std::string>("model_file", modelFile, "");
	ROS_INFO_STREAM("Using modelFile:" << modelFile);
	nh.param<std::string>("logger_filename_ik", loggerFileNameIK, "test_ik");
	nh.param<std::string>("logger_filename_imus", loggerFileNameIMUs, "test_imus");

	nh.param<int>("calib_samples", num_calib_samples, 10);

	if (!usePositionMarkers && !useOrientationMarkers)
		ROS_FATAL("You need at least one form of input marker to compute IK!!!");


	nh.param<bool>("filter_output",publish_filtered, true);
	nh.param<double>("cutoff_freq", cutoffFreq, 0.0);
	nh.param<int>("memory", memory, 0);
	nh.param<int>("spline_order", splineOrder, 0);
	nh.param<int>("delay", delay, 0);

	if( usePositionMarkers)
	{
		std::string getter_type;
		nh.param<std::string>("point_getter_type",getter_type,"");

		ROS_INFO_STREAM("Using position Markers![" << getter_type <<"]");
		//needs a param to get if it is the TF input or a marker input

		if (getter_type == "tf")
			pointGetter = new GetPointFromSomeTF;
		if (getter_type == "marker")
			pointGetter = new GetPointFromMarkers;
		if (getter_type != "marker" && getter_type != "tf")
			throw(std::invalid_argument("invalid positional getter type."));

	}

	ROS_DEBUG_STREAM("Finished getting params.");


}
void UIMUnode::registerType(Object* muscleModel) //do I even need this?
{

	Object::registerType(*muscleModel);

}
void UIMUnode::reconfigure_callback(osrt_ros::UIMUConfig &config, uint32_t level){

	std::unique_lock<std::mutex> lock(ik_mtx, std::try_to_lock);
	if (!lock.owns_lock()) { ROS_WARN("busy calibrating, or doing something else that takes a long time, ignored"); return; }


	ROS_INFO("IMU Ground Orientation reconfigure request %s, (%f, %f, %f)", config.imu_direction_axis_param.c_str(), config.imu_ground_rotation_x, config.imu_ground_rotation_y,config.imu_ground_rotation_z);

	if (clb_is_ready)
	{
		imuDirectionAxis = config.imu_direction_axis_param;
		xGroundRotDeg1 = config.imu_ground_rotation_x;
		yGroundRotDeg1 = config.imu_ground_rotation_y;
		zGroundRotDeg1 = config.imu_ground_rotation_z;
		ROS_WARN_STREAM("imu_ground_rotation_x/y/z are NO LONGER USED: the nominal R_GoGi1 "
				"comes exclusively from the imu_ref_ori TF, change that TF instead. "
				"imu_direction_axis DOES still matter: it supplies the heading DOF, which "
				"is not recoverable from the static pose and is folded into R_GoGi1.");
		calibrate_ik();
	}
	else
		ROS_WARN("IMU Ground Orientation reconfigure request warning: calibrator not yet defined.");

}
void UIMUnode::reconfigure_heading_callback(osrt_ros::headingConfig &config, uint32_t level){

	std::unique_lock<std::mutex> lock(ik_mtx, std::try_to_lock);
	if (!lock.owns_lock()) { ROS_WARN("busy calibrating, or doing something else that takes a long time, ignored"); return; }


	ROS_INFO("Heading Reconfigure request base IMU heading angle: %f", config.base_imu_heading);

	if (clb_is_ready)
	{
		clb->baseHeadingAngle = config.base_imu_heading;
		//I need to change the things that are related to the heading here!
		calibrate_ik();

	}
	else
		ROS_WARN("Heading Reconfigure request warning: calibrator not yet defined.");

}
void UIMUnode::define_tasks()
{

	ROS_YE("DEFINING TASKS!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!1");
	// marker tasks
	if (usePositionMarkers) //not sure what this does, some interface for VICON .trc files. we are not using it here.
	{
		ROS_DEBUG_STREAM("Setting up markerTasks");
		vector<string> markerObservationOrder;
		for (auto some_marker_name:pointGetter->markerNames)
			ROS_YE("AR positional marker name: "<<some_marker_name);

		InverseKinematics::createMarkerTasksFromMarkerNames(* model, pointGetter->markerNames, markerTasks,
				markerObservationOrder);
	}

	// imu tasks
	if (useOrientationMarkers)
	{
		ROS_DEBUG_STREAM("Setting up imuTasks");
		InverseKinematics::createIMUTasksFromObservationOrder(
				*model, imuObservationOrder, imuTasks);
	}

	ROS_DEBUG_STREAM("Starting driver");
	{
		string imuObservationOrderStr;
		for(auto a:imuObservationOrder)
		{
			imuObservationOrderStr +=a+", ";
		}
		ROS_INFO_STREAM("Using imu observation order: " << imuObservationOrderStr);
	}
}
void UIMUnode::calibrate_ik()
{
	chrono::high_resolution_clock::time_point t1=chrono::high_resolution_clock::now() ;

	if (useOrientationMarkers)
	{
		// Ground-to-ground rotation: maps the IMU/VIO global reference frame Gi
		// into OpenSim's ground Go. tf2's lookupTransform(target, source) returns
		// R_target_source, so looking up (imu_ref_ori <- opensim_frame) gives
		// R_GiGo; we want R_GoGi, hence the inversion.
		//
		// This is the single place the heading correction belongs: it is just a yaw
		// premultiplied here, see below.
		clb->R_GoGi1 = ~clb->setGroundOrientationFromTF("imu_ref_ori");

		clb->sameHeader.stamp = ros::Time::now();
		clb->publishTransform("R_GoGi1_nominal", SimTK::Transform(clb->R_GoGi1, SimTK::Vec3{1.1,1,-1}), clb->sameHeader);
		ROS_INFO_STREAM("R_GoGi1 nominal (device convention only, from the imu_ref_ori TF):\n" << clb->R_GoGi1);

		// The heading is NOT recoverable from the static pose. Counting it: each
		// sensor contributes 3 equations (R_GoB_i * R_BS_i == R_GoGi * q0_i) and 3
		// unknowns (its own R_BS_i), so N sensors give 3N equations for 3N+1
		// unknowns -- the extra one being the yaw of R_GoGi. Under-determined by
		// exactly one, always, no matter how many sensors you add. The missing DOF
		// therefore HAS to come from outside, and `imu_direction_axis` is that
		// external input: it asserts which principal axis of the base sensor points
		// along the subject's anterior direction.
		clb->computeHeadingRotation(imuBaseBody, imuDirectionAxis, actualModelState);

		// Fold the per-session yaw into the ground rotation, ONCE, before calibrating.
		//
		// This ordering is load-bearing. R_GoGi1 is what calibrateIMUTasks() solves
		// R_BS against AND what transform() applies to every runtime sample, so
		// folding here is what keeps the two sides consistent by construction. Do
		// NOT be tempted to move the heading into the imu_ref_ori TF instead: that
		// TF encodes the orientation provider's fixed frame convention (z-forward,
		// y-down for a camera) and is authored once, whereas the heading depends on
		// where the operator physically strapped the base sensor THIS session.
		// Different lifetimes; they cannot share a home.
		clb->R_GoGi1 = clb->R_heading * clb->R_GoGi1;

		clb->publishTransform("R_GoGi1", SimTK::Transform(clb->R_GoGi1, SimTK::Vec3{1.1,1,-1.2}), clb->sameHeader);
		ROS_INFO_STREAM("R_GoGi1 effective (device convention * heading):\n" << clb->R_GoGi1);

		clb->calibrateIMUTasks(imuTasks);
	}

	if (usePositionMarkers)
	{
		// here we need to consume mt, calculate the position it should be, vs what it is, and then update the model 
		ROS_YE("positional markers calibration is a wip,,, is it even running?");

		// we want to calibrate IK in the position in which it is in right now, but for the 
		// positional markers, this maybe messes up the model. Anyway, doing it like this
		SimTK::Vector savedQ = actualModelState.getQ();

		//model.set() = pclb->model;
		// i used a make_unique, i think, so it should work by default

		// If it could not average, mt is empty or stale and reading it is how the
		// node died before. Keep the model markers where they are and carry on; a
		// wrong calibration is worse than none.
		if (!pclb->computeAvgStaticPose())
		{
			ROS_ERROR("skipping positional marker calibration, model markers left at their authored positions");
		}
		else
			for (size_t i = 0; i < pclb->mt.size(); ++i) 
			{
				SimTK::Vec3 p_G = pclb->mt[i];
				std::string markername = pointGetter->markerNames[i];
				ROS_INFO_STREAM("marker"<< markername <<" pos we measured: " << p_G);
				OpenSim::Marker& opensimMarker = model->updMarkerSet().get(markername); 
				ROS_INFO_STREAM("marker"<< markername <<" pos it had before: " << opensimMarker.get_location());
				//magically sets the marker is this enough?
				auto p_F = model->getGround().findStationLocationInAnotherFrame(actualModelState, p_G, opensimMarker.getParentFrame());

				ROS_INFO_STREAM("marker"<< markername <<" pos it will have now: " << p_F);

				opensimMarker.set_location(p_F);
			}
		ROS_YE("positional markers calibration is wip,,, we sorta finished bro!");
		// this is the place, this is the time. the model which is unique needs to be initialized, and it was to be here and it has to be now!
		// is this fine? i have no clue, claude says so. i will keep this comment here because suspish

		//laut claude we only need to reset the model here, so:
		actualModelState = model->initSystem();           // fresh default state
		actualModelState.updQ() = savedQ;                 // put it back
		model->realizePosition(actualModelState);
	}

	resetIK(); 

	//TODO: publish correct ROS topics
	ROS_INFO_STREAM("Done with calibrate_ik");
	chrono::high_resolution_clock::time_point t2=chrono::high_resolution_clock::now() ;

	ROS_YE(bar << "calibrate_ik call duration in ms:"<<magenta<<chrono::duration_cast<chrono::milliseconds>(t2-t1).count()<<bar <<reset);
}

void UIMUnode::resetIK()
{
	ROS_DEBUG_STREAM("Setting up IK");
	auto* fresh = new InverseKinematics(model.get(), actualModelState, markerTasks, imuTasks, SimTK::Infinity, 1e-5);
	std::swap(ik, fresh);
	delete(fresh);

	qRawLogger = ik->initializeLogger(); // not sure this is correct
	output.labels = qRawLogger.getColumnLabels();
}

SimTK::RowVector UIMUnode::fromVectorOfSimTKQuaternionsToARowVector(std::vector<SimTK::Quaternion> vv)
{
	std::vector<double> serialized;

	for (auto q:vv)
	{
		ROS_INFO_STREAM("Weird serializer (quaternion): " << q[0] <<","<<q[1]<<","<<q[2]<<","<<q[3]);
		serialized.push_back(q[0]);
		serialized.push_back(q[1]);
		serialized.push_back(q[2]);
		serialized.push_back(q[3]);
	}

	SimTK::RowVector serializedv(serialized.size());
	for (int ii = 0; ii <serializedv.size(); ii++)
	{
		serializedv[ii] = serialized[ii];
	}
	return serializedv;
}
void UIMUnode::clearLogger(TimeSeriesTable &t) //TODO: move it somewhere nice. maybe make loggers a wrapper class
{
	for (size_t iii= 0; iii<t.getNumRows();iii++)
		t.removeRow(0);
	if (t.getNumRows() == 0)
		ROS_INFO_STREAM("logger cleared");
	else
		ROS_WARN_STREAM("couldnt clear logger table!");
}
void UIMUnode::recordCalibrationPose()
{
	ROS_INFO_STREAM("Recording calibration pose. NOT USING A DEFAULT POSITION BTW; WE ARE USING THE POSITION IN WHICH THE STATE IS IN (or at least this is what i want)!");

	if (useOrientationMarkers)
	{
		ROS_DEBUG_STREAM("clb samples");
		clb->setup(actualModelState);
		clearLogger(imuCalibrationLogger);
		ROS_INFO_STREAM("After clearing table: Number of rows in table is: " << imuCalibrationLogger.getNumRows());
		clb->recordNumOfSamples(num_calib_samples); 
		clb_is_ready = true;
		imuCalibrationLogger.appendRow(0, fromVectorOfSimTKQuaternionsToARowVector(clb->staticPoseQuaternions)); //if this is the time, maybe we want to add the calibration time here as well. Also, maybe we don't want to clear the calibration, or clear only after saving? TODO: think about this
		ROS_INFO_STREAM("After appending clb samples to table: Number of rows in table is: " << imuCalibrationLogger.getNumRows());
	}
	if (usePositionMarkers)
	{
		pclb->calibSamples.clear();
		if (!pclb->recordNumOfSamples(num_calib_samples))
			ROS_ERROR("positional marker calibration did not get the samples it asked for");
		// erm some logging
		ROS_WARN("Calibration for Position Markers is not yet logging!");

	}
}	
bool UIMUnode::calibrationSrv(std_srvs::Empty::Request &req, std_srvs::Empty::Response &res)
{
	ROS_INFO_STREAM("Calibration service called!");
	std::unique_lock<std::mutex> lock(ik_mtx);

	recordCalibrationPose();
	//I need to restart ik again as well
	calibrate_ik();
	return true;
}
void UIMUnode::onInit()
{
	get_params();
	//f = boost::bind(&UIMUnode::reconfigure_callback, this, _1,_2);
	//server.setCallback(f);
	time_pub = nh.advertise<std_msgs::Int64>("time",1);
	time_ik_pub = nh.advertise<std_msgs::Int64>("time_ik",1);

	Ros::CommonNode::onInit(0); //we are not reading from anything, we are a source
	string all_labels;
	ros::NodeHandle nh("~");
	ROS_INFO_STREAM("Setting plottable_outputs topic advertisers...");
	for (auto l:output.labels)
	{

		plottable_outputs.push_back(nh.advertise<std_msgs::Float64>("joints/"+l,1));
		all_labels+=l+", ";
	}
	ROS_INFO_STREAM("Publisher labels: "<<all_labels);
	//I want to start the service after we set the labels, otherwise it might reply with an empty message.
	// setup model
	ROS_DEBUG_STREAM("Setting up model.");
	model = std::make_unique<OpenSim::Model>(modelFile);
	OpenSimUtils::removeActuators(*model);
	actualModelState = model->initSystem();
	model->realizePosition(actualModelState);
	if(publish_filtered)
	{
		//filter
		ROS_DEBUG_STREAM("Setting up filter");
		LowPassSmoothFilter::Parameters ikFilterParam;
		ikFilterParam.numSignals = model->getNumCoordinates();
		ikFilterParam.memory = memory;
		ikFilterParam.delay = delay;
		ikFilterParam.cutoffFrequency = cutoffFreq;
		ikFilterParam.splineOrder = splineOrder;
		ikFilterParam.calculateDerivatives = true;
		ROS_DEBUG_STREAM("filter parameters set.");
		ikfilter = new LowPassSmoothFilter(ikFilterParam);
		// initialize filtered loggers
		ROS_DEBUG_STREAM("getting columnNames from model");
		auto columnNames = OpenSimRT::OpenSimUtils::getCoordinateNamesInMultibodyTreeOrder(*model);
		string columnNamesStr = "";
		for(auto cn:columnNames)
		{
			columnNamesStr+=cn+", ";
		}
		ROS_DEBUG_STREAM("got columnNamesStr: " << columnNamesStr);
		qLogger.setColumnLabels(columnNames);
		qDotLogger.setColumnLabels(columnNames);
		qDDotLogger.setColumnLabels(columnNames);
		ROS_DEBUG_STREAM("columnNames for loggers set.");
		initializeLoggers("qLogger",&qLogger);
		initializeLoggers("qDotLogger",&qDotLogger);
		initializeLoggers("qDDotLogger",&qDDotLogger);

	}

	calibrationService = nh.advertiseService("calibrate", &UIMUnode::calibrationSrv, this);

	if (useOrientationMarkers){
		ROS_DEBUG_STREAM("Staring UIMUInputDriver with tf_frame_prefix: [" << tf_frame_prefix << "] and rate: [" << rate <<"]" );
		driver = new UIMUInputDriver(imuObservationOrder,tf_frame_prefix,rate*2); //uses tf server
											  //so now we are updating this at double the rate of the main loop because we are no longer waiting for the conditional, 
		driver->startListening();
		imuLogger = driver->initializeLogger();
		initializeLoggers(loggerFileNameIMUs,&imuLogger);
		imuCalibrationLogger = driver->initializeCalibrationValuesLogger();
		initializeLoggers("calib", &imuCalibrationLogger);

		// calibrator
		ROS_DEBUG_STREAM("Setting up IMUCalibrator");
		clb = new IMUCalibrator(model.get(), driver, imuObservationOrder);
	}

	if (usePositionMarkers && pointGetter)
	{
		ROS_INFO_STREAM("Setting up PointCalibrator");
		pclb = new PointCalibrator(model.get(),pointGetter);
		pointGetter->rate = new ros::Rate(rate*2); // no longer waiting for the conditional so same problem as above
							   //okay, this TRC logger is stupid. it measures frames and assumes no jitter
							   //also the rate is not the rate that pointgetters will get the frames, but the rate at which they will be logged. and that is defined by the append thing in the run loop. so it is the normal rate
							   //and it is a string, because this is bs and they know it
		posLogger.addTableMetaData("DataRate",std::to_string(rate));
		posLogger.addTableMetaData("Units",std::string{"m"});

		posLogger.setColumnLabels(pointGetter->markerNames);
		initializeLoggers("posLog",&posLogger);
		pointGetter->startListening();


	}

	ROS_INFO_STREAM("onInit define_tasks >>>>>>>");

	define_tasks();

	{
		std::lock_guard<std::mutex> lock(ik_mtx);
		resetIK();
	}
	initializeLoggers(loggerFileNameIK,&qRawLogger); 

	// Future frederico and Claude: we are not calibrating IK without any data. it makes no sense.
	//calibrate_ik();

	set_ready(); // ready to calibrate i guess...

	ROS_INFO_STREAM(cyan << "onInit finished just fine.");
}

void UIMUnode::run() {

	ROS_DEBUG_STREAM("started to run");
	bool keep_running=true;
	int ij = 0;
	int runs_to_log= 4;
	try { // main loop
		opensimrt_msgs::CommonTimed msg;
		std_msgs::Header h;
		h.frame_id = "subject";
		std::pair<TransObs,bool> markerObservations;
		OriData imuData;
		double this_time=-1.0, last_time = -1.1; //it should never happen that the time remains as -1.0, the initialization should make sure that either userOri or usePos is always true.
		SimTK::Array_<SimTK::Rotation> transformedOris;
		double ori_time, pos_time; // we should keep track of them
		while (keep_running) {
			//ROS_YE("=======================================================================================================");
			if (ij>=runs_to_log ) {
				ij = 0; 
				msg.events = opensimrt_msgs::Events();
				keep_running = ros::ok();
				addEvent("afterrosok"+std::to_string(ij),msg);
			}
			ij++;
			bool have_ik;
			{
				std::lock_guard<std::mutex> it_wont_race_i_think_but_locking_is_safer(ik_mtx);
				have_ik = ( ik!= nullptr);
			}
			if (!have_ik) { 
				ROS_WARN_THROTTLE(5, "not calibrated yet, call ~/calibrate");
				ros::spinOnce();
				r->sleep(); 
				continue; 
			}


			addEvent("run_start"+std::to_string(ij),msg);


			h.stamp = ros::Time::now();
			msg.header = h;

			if (useOrientationMarkers)
			{
				// get input from imus
				ROS_DEBUG_STREAM("Getting frame:");
				imuData = driver->getFrame();
				ori_time = imuData.time;
				this_time = ori_time;
				transformedOris = clb->transform(imuData.data);
				addEvent("got_ori_frame"+std::to_string(ij),msg);
			}
			if (usePositionMarkers) //not sure what this does, some interface for VICON .trc files. we are not using it here.
			{
				ROS_DEBUG_STREAM("Getting marker frame:");
				markerObservations = pointGetter->get_new_data();
				// so maybe we want to have also another list with the marker qualities
				pos_time = pointGetter->frame_time; // the time of the frame we just got, not of whatever arrived since
				this_time = pos_time;

				for(int32_t i = 0; i< pointGetter->markerNames.size(); i++)
				{
					//	auto someIx = ik->markerAssemblyConditions->getMarkerIx(pointGetter->markerNames[i]);
					//ik->markerAssemblyConditions->changeMarkerWeight(someIx,markerObservations.second[i]);

				}
			}

			if (useOrientationMarkers && usePositionMarkers)
			{
				// we have 2 different sources of time now, so we probably should log this somehow
				this_time = (ori_time + pos_time)/2.0;

			}

			//numFrames++;

			// solve ik

			//for(int iii = 0 ; iii< markerObservations.first.size();iii++)
			//	ROS_INFO_STREAM(markerObservations.first[iii]);
			addEvent("got_data"+std::to_string(ij),msg);

			//we need to lock this because ik may be calibrating, and if it is deleted when it tries to run it will crash.

			InverseKinematics::Output pose;
			bool ori_new = useOrientationMarkers && !imuData.stale;
			bool pos_new = usePositionMarkers && markerObservations.second;
			bool data_is_new = ori_new || pos_new ;

			addEvent("beforspinonce"+std::to_string(ij),msg);
			ros::spinOnce(); // we spin every loop, if you want to get paranoid and % it again, be my guest, but i think this is fast. anyway, you should be able to measure it.
			addEvent("afterspinonce"+std::to_string(ij),msg);

			if (!data_is_new)
			{
				continue;
			}
			// if the time is the same, this thing will crash, so we need to prevent it from happening
			if (last_time >= this_time) 
			{
				ROS_WARN("The clock is stale or running back in time. Since we already check for staleness, this means we are getting new data with an equal or older timestamp. This shouldn't happen. If it is happening all the time, you should recheck this code.");
				continue; //the greater part shouldnt happen, but somehow it did! o.O
			}
			last_time = this_time;


			{
				std::lock_guard<std::mutex> lock(ik_mtx);
				pose = ik->solve(
						{this_time, markerObservations.first.first, transformedOris });
			}
			addEvent("ik"+std::to_string(ij),msg);
			//sumDelayMS += chrono::duration_cast<chrono::milliseconds>(t2 - t1).count();

			//msg.data.push_back(pose.t);
			msg.data = std::vector<double>(); //TODO: we should change this in Osb::update_pose, i think 
			Osb::update_pose(msg, pose.t, pose.q); //this is from opensimbridge, another package that shouldnt exist TODO: this is not working properly, it is appending to the end and this creates a huge msg.data... not sure if this was intended, but now that we dont restart msg every frame, this is a problem. 

			/*
			   if(plottable_outputs.size()>0) // we don't have the labels here, this is stupid
			   for (const auto& joint_angle:pose.q)
			   {
			   ROS_DEBUG_STREAM("some joint_angle: "<<joint_angle << " will be sent to topic: " << plottable_outputs[i].getTopic());
			   std_msgs::Float64 j_msg;
			   j_msg.data = joint_angle*180/3.14159265;
			   plottable_outputs[i].publish(j_msg);
			   }
			   else ROS_ERROR_ONCE("TODO: you should have the labels, we are creating them, the initialization order is wrong, please create the topics after reading the model");
			   */
			pub.publish(msg); 
			addEvent("afternormal_pub"+std::to_string(ij),msg);
			if(publish_filtered)
			{
				auto ikFiltered = ikfilter->filter({pose.t, pose.q});
				auto q = ikFiltered.x;
				auto qDot = ikFiltered.xDot;
				auto qDDot = ikFiltered.xDDot;
				ROS_DEBUG_STREAM("Filter ran ok");
				if (!ikFiltered.isValid) {
					ROS_DEBUG_STREAM("filter results are NOT valid");
					continue; }
				ROS_DEBUG_STREAM("Filter results are valid");
				addEvent("afterfilter"+std::to_string(ij),msg);

				opensimrt_msgs::PosVelAccTimed msg_filtered = Osb::get_as_ik_filtered_msg(h, ikFiltered.t, q, qDot, qDDot);
				pub_filtered.publish(msg_filtered); // not working
				addEvent("afterfilter_pub"+std::to_string(ij),msg);

				//adding the data to the loggers
				if (isRecording())
				{
					qLogger.appendRow(pose.t,~q);
					qDotLogger.appendRow(pose.t,~qDot);
					qDDotLogger.appendRow(pose.t,~qDDot);
				}
			}
			// record
			if (isRecording())
			{
				ROS_YE("Recording!");
				if(useOrientationMarkers)
					imuLogger.appendRow(ori_time, driver->frame);//
				if(usePositionMarkers)
				{
					posLogger.appendRow(pos_time,markerObservations.first.first );

				}
				qRawLogger.appendRow(pose.t, ~pose.q);
			}

			//ros::spinOnce();
			if(false) // this looks like it is free running, but it isn't. it is being gated by driver->getFrame() and we also have the pointGetter doing 2 reads to make sure we don't lose a single sample. 
			{
				r->sleep();
				addEvent("afterrate"+std::to_string(ij),msg);
			}
			{
				//r->sleep();
				addEvent("afternorate"+std::to_string(ij),msg);


			}
		}
	} catch (std::exception& e) {
		cout << e.what() << endl;

		driver->shouldTerminate(true);
		pointGetter->shouldTerminate(true);
	}

	//cout << "Mean delay: " << (double) sumDelayMS / numFrames << " ms" << endl;

	//CSVFileAdapter::write( qRawLogger, loggerFileNameIK);
	//CSVFileAdapter::write( imuLogger, loggerFileNameIMUs);

	// // store results
	// STOFileAdapter::write(
	//         qRawLogger, subjectDir + "real_time/inverse_kinematics/qRaw_imu.sto");
}






