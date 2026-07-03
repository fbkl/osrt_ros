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
	ROS_INFO("IMU Ground Orientation reconfigure request %s, (%f, %f, %f)", config.imu_direction_axis_param.c_str(), config.imu_ground_rotation_x, config.imu_ground_rotation_y,config.imu_ground_rotation_z);

	if (clb_is_ready)
	{
		imuDirectionAxis = config.imu_direction_axis_param;
		xGroundRotDeg1 = config.imu_ground_rotation_x;
		yGroundRotDeg1 = config.imu_ground_rotation_y;
		zGroundRotDeg1 = config.imu_ground_rotation_z;
		start_ik();
	}
	else
		ROS_WARN("IMU Ground Orientation reconfigure request warning: calibrator not yet defined.");

}
void UIMUnode::reconfigure_heading_callback(osrt_ros::headingConfig &config, uint32_t level){
	ROS_INFO("Heading Reconfigure request base IMU heading angle: %f", config.base_imu_heading);

	if (clb_is_ready)
	{
		clb->baseHeadingAngle = config.base_imu_heading;
		//I need to change the things that are related to the heading here!
		start_ik();

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

		InverseKinematics::createMarkerTasksFromMarkerNames(model, pointGetter->markerNames, markerTasks,
				markerObservationOrder);
	}

	// imu tasks
	if (useOrientationMarkers)
	{
		ROS_DEBUG_STREAM("Setting up imuTasks");
		InverseKinematics::createIMUTasksFromObservationOrder(
				model, imuObservationOrder, imuTasks);
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
void UIMUnode::start_ik()
{
	chrono::high_resolution_clock::time_point t1=chrono::high_resolution_clock::now() ;
	ROS_DEBUG_STREAM("setGroundOrientationSeq");
	if (false)
	{
		clb->R_GoGi1 = clb->setGroundOrientationSeq(xGroundRotDeg1, yGroundRotDeg1, zGroundRotDeg1);
		ROS_INFO("Setting ground orientation from params");
	}
	else
	{
		auto R_GoGi2 = clb->setGroundOrientationSeq(xGroundRotDeg1, yGroundRotDeg1, zGroundRotDeg1);
		Vec3 trans_p{1,1,1};
		Vec3 trans_p2{1.1,1,1};
		SimTK::Transform TX(R_GoGi2,trans_p);
		SimTK::Transform TX2(~R_GoGi2,trans_p2);
		clb->sameHeader.stamp = ros::Time::now();
		clb->publishTransform("imu_R_GoGi_original", TX, clb->sameHeader);
		clb->publishTransform("imu_R_GiGo_original", TX2, clb->sameHeader);
		Vec3 trans_p0{1.1,1,-1};
		clb->R_GoGi1 = ~clb->setGroundOrientationFromTF("imu_ref_ori"); // why the double inversion here? well, because we want to multiply this by the orientations from the imus and get the canonical rotation that they apply. or something, idk.
		SimTK::Transform TX0(clb->R_GoGi1,trans_p0);
		clb->publishTransform("imu_ref_ori_inv", TX0, clb->sameHeader);
		ROS_YE("UNTESTED!!! setting ground orientation from TF what i defined:"<< clb->R_GoGi1 << "\nwhat was before (R_GoGi_original from imu_ground_rotation_XYZ parameter defined magic numbers)" << R_GoGi2 );
	}
	ROS_DEBUG_STREAM("heading");
	clb->computeHeadingRotation(imuBaseBody, imuDirectionAxis);

	//std::cout << boost::stacktrace::stacktrace() << std::endl;
	clb->calibrateIMUTasks(imuTasks);
	ROS_DEBUG_STREAM("Setting up IMUCalibrator");

	if (useOrientationMarkers){	
		ROS_DEBUG_STREAM("setGroundOrientationSeq");
		if (false)
		{
			clb->R_GoGi1 = clb->setGroundOrientationSeq(xGroundRotDeg1, yGroundRotDeg1, zGroundRotDeg1);
			ROS_INFO("Setting ground orientation from params");
		}
		else
		{
			auto R_GoGi2 = clb->setGroundOrientationSeq(xGroundRotDeg1, yGroundRotDeg1, zGroundRotDeg1);
			Vec3 trans_p{1,1,1};
			Vec3 trans_p2{1.1,1,1};
			SimTK::Transform TX(R_GoGi2,trans_p);
			SimTK::Transform TX2(~R_GoGi2,trans_p2);
			clb->sameHeader.stamp = ros::Time::now();
			clb->publishTransform("imu_R_GoGi_original", TX, clb->sameHeader);
			clb->publishTransform("imu_R_GiGo_original", TX2, clb->sameHeader);
			Vec3 trans_p0{1.1,1,-1};
			clb->R_GoGi1 = ~clb->setGroundOrientationFromTF("imu_ref_ori");
			SimTK::Transform TX0(~clb->R_GoGi1,trans_p0);
			clb->publishTransform("imu_ref_ori_inv", TX0, clb->sameHeader);
			ROS_WARN_STREAM("UNTESTED!!! setting ground orientation from TF what i defined:"<< clb->R_GoGi1 << "\nwhat was before" << R_GoGi2 );
		}
		ROS_DEBUG_STREAM("heading");
		clb->computeHeadingRotation(imuBaseBody, imuDirectionAxis);
		//std::cout << boost::stacktrace::stacktrace() << std::endl;
		clb->calibrateIMUTasks(imuTasks);
		ROS_DEBUG_STREAM("Setting up IMUCalibrator");
	}
	// initialize ik (lower constraint weight and accuracy -> faster tracking)
	ROS_DEBUG_STREAM("Setting up IK");
	ik = new InverseKinematics(model, markerTasks, imuTasks, SimTK::Infinity, 1e-5);
	qRawLogger = ik->initializeLogger();
	initializeLoggers(loggerFileNameIK,&qRawLogger);

	//TODO: publish correct ROS topics
	output.labels = qRawLogger.getColumnLabels();
	ROS_INFO_STREAM("Done with start_ik");
	chrono::high_resolution_clock::time_point t2=chrono::high_resolution_clock::now() ;

	ROS_YE(bar << "start_ik call duration in ms:"<<magenta<<chrono::duration_cast<chrono::milliseconds>(t2-t1).count()<<bar <<reset);
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
void UIMUnode::doCalibrate()
{
	ROS_DEBUG_STREAM("clb samples");
	clearLogger(imuCalibrationLogger);
	ROS_INFO_STREAM("After clearing table: Number of rows in table is: " << imuCalibrationLogger.getNumRows());
	clb->recordNumOfSamples(10); //TODO:PARAM!
	clb_is_ready = true;
	imuCalibrationLogger.appendRow(0, fromVectorOfSimTKQuaternionsToARowVector(clb->staticPoseQuaternions)); //if this is the time, maybe we want to add the calibration time here as well. Also, maybe we don't want to clear the calibration, or clear only after saving? TODO: think about this
	ROS_INFO_STREAM("After appending clb samples to table: Number of rows in table is: " << imuCalibrationLogger.getNumRows());
}
bool UIMUnode::calibrationSrv(std_srvs::Empty::Request &req, std_srvs::Empty::Response &res)
{
	ROS_INFO_STREAM("Calibration service called!");
	doCalibrate();
	//I need to restart ik again as well
	start_ik();
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
	model = OpenSim::Model(modelFile);
	OpenSimUtils::removeActuators(model);
	if(publish_filtered)
	{
		//filter
		ROS_DEBUG_STREAM("Setting up filter");
		LowPassSmoothFilter::Parameters ikFilterParam;
		ikFilterParam.numSignals = model.getNumCoordinates();
		ikFilterParam.memory = memory;
		ikFilterParam.delay = delay;
		ikFilterParam.cutoffFrequency = cutoffFreq;
		ikFilterParam.splineOrder = splineOrder;
		ikFilterParam.calculateDerivatives = true;
		ROS_DEBUG_STREAM("filter parameters set.");
		ikfilter = new LowPassSmoothFilter(ikFilterParam);
		// initialize filtered loggers
		ROS_DEBUG_STREAM("getting columnNames from model");
		auto columnNames = OpenSimRT::OpenSimUtils::getCoordinateNamesInMultibodyTreeOrder(model);
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


	if (useOrientationMarkers){
		calibrationService = nh.advertiseService("calibrate", &UIMUnode::calibrationSrv, this);
		ROS_DEBUG_STREAM("Staring UIMUInputDriver with tf_frame_prefix: [" << tf_frame_prefix << "] and rate: [" << rate <<"]" );
		driver = new UIMUInputDriver(imuObservationOrder,tf_frame_prefix,rate); //uses tf server
		driver->startListening();
		imuLogger = driver->initializeLogger();
		initializeLoggers(loggerFileNameIMUs,&imuLogger);
		imuCalibrationLogger = driver->initializeCalibrationValuesLogger();
		initializeLoggers("calib", &imuCalibrationLogger);

		// calibrator
		ROS_DEBUG_STREAM("Setting up IMUCalibrator");
		clb = new IMUCalibrator(model, driver, imuObservationOrder);
	doCalibrate(); //Maybe i dont want to do this in the initialization
	}



	define_tasks();

	ik = new InverseKinematics(model, markerTasks, imuTasks, SimTK::Infinity, 1e-5);
	//start_ik();
	// mean delay
	ROS_DEBUG_STREAM("onInit finished just fine.");
}

void UIMUnode::run() {

	ROS_DEBUG_STREAM("started to run");
	//ros::AsyncSpinner spinner(4);
	//spinner.start();
	try { // main loop
		int i = 0; // we dont need to react to service calls and other things every loop, we can have it wait, like 200ms or so, since this can be an expensive call,,, let's see if that improves the running times 
		while (ros::ok()) {
			ROS_YE("=======================================================================================================");
			opensimrt_msgs::CommonTimed msg;
			std_msgs::Header h;
			h.stamp = ros::Time::now();
			h.frame_id = "subject";
			msg.header = h;

			TransObs markerObservations;
			std::pair<double, std::vector<OpenSimRT::UIMUData>> imuData;
			double this_time=-1.0; //it should never happen that the time remains as -1.0, the initialization should make sure that either userOri or usePos is always true.
			if (useOrientationMarkers)
			{
				// get input from imus
				ROS_DEBUG_STREAM("Getting frame:");
				imuData = driver->getFrame();
				this_time = imuData.first;
			}
			if (usePositionMarkers) //not sure what this does, some interface for VICON .trc files. we are not using it here.
			{
				ROS_DEBUG_STREAM("Getting marker frame:");
				markerObservations = pointGetter->get_translations();
				this_time = pointGetter->last_time;
				// so maybe we want to have also another list with the marker qualities
				

				for(int32_t i = 0; i< pointGetter->markerNames.size(); i++)
				{
					auto someIx = ik->markerAssemblyConditions->getMarkerIx(pointGetter->markerNames[i]);
//ik->markerAssemblyConditions->changeMarkerWeight(someIx,markerObservations.second[i]);
			
				}
			}
			ROS_DEBUG_STREAM("Solving inverse kinematics:" );
			numFrames++;

			// solve ik
			chrono::high_resolution_clock::time_point t1;
			t1 = chrono::high_resolution_clock::now();

			if (last_time == this_time)
			{
				ROS_WARN_ONCE("run() rate exceeds data update rate.");
				ros::spinOnce();
				r->sleep();
				continue;
			}
			auto pose = ik->solve(
					{this_time, markerObservations.first, clb->transform(imuData.second)});
			last_time = this_time;
			addEvent("ik",msg);
			chrono::high_resolution_clock::time_point t2;
			t2 = chrono::high_resolution_clock::now();
			sumDelayMS += chrono::duration_cast<chrono::milliseconds>(t2 - t1)
				.count();
			ROS_DEBUG_STREAM( "pose is:" << pose.q);

			//msg.data.push_back(pose.t);
			Osb::update_pose(msg, pose.t, pose.q);
			double Dt = pose.t-previousTime;
			double jitter = Dt-previousDt;

			ROS_DEBUG_STREAM("jitter(us):" << jitter*1000000);
			ROS_DEBUG_STREAM("delta_t   :" << Dt);
			ROS_DEBUG_STREAM("T (pose.t):" << pose.t);

			if(plottable_outputs.size()>0) // we don't have the labels here, this is stupid
				for (const auto& joint_angle:pose.q)
				{
					ROS_DEBUG_STREAM("some joint_angle: "<<joint_angle << " will be sent to topic: " << plottable_outputs[i].getTopic());
					std_msgs::Float64 j_msg;
					j_msg.data = joint_angle*180/3.14159265;
					plottable_outputs[i].publish(j_msg);
				}
			else ROS_ERROR("TODO: you should have the labels, we are creating them, the initialization order is wrong, please create the topics after reading the model");

			pub.publish(msg); //not working
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
				opensimrt_msgs::PosVelAccTimed msg_filtered = Osb::get_as_ik_filtered_msg(h, ikFiltered.t, q, qDot, qDDot);
				pub_filtered.publish(msg_filtered); // not working

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
					imuLogger.appendRow(pose.t, driver->frame);//
				qRawLogger.appendRow(pose.t, ~pose.q);
			}
			previousTime = pose.t;
			previousDt = Dt;
			std_msgs::Int64 time_ik_msg;
			time_ik_msg.data = chrono::duration_cast<chrono::microseconds>(t2 - t1).count();
			time_ik_pub.publish(time_ik_msg);

			chrono::high_resolution_clock::time_point t3;
			t3 = chrono::high_resolution_clock::now();
			std_msgs::Int64 time_msg;
			time_msg.data = std::chrono::duration_cast<std::chrono::microseconds>(t3 -t1).count();
			time_pub.publish(time_msg);

			i++;
			ros::spinOnce();
			r->sleep();
		}
	} catch (std::exception& e) {
		cout << e.what() << endl;

		driver->shouldTerminate(true);
	}

	cout << "Mean delay: " << (double) sumDelayMS / numFrames << " ms" << endl;

	//CSVFileAdapter::write( qRawLogger, loggerFileNameIK);
	//CSVFileAdapter::write( imuLogger, loggerFileNameIMUs);

	// // store results
	// STOFileAdapter::write(
	//         qRawLogger, subjectDir + "real_time/inverse_kinematics/qRaw_imu.sto");
}






