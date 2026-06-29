#ifndef ORIENTATION_PROVIDER_H_FBK20220627
#define ORIENTATION_PROVIDER_H_FBK20220627

#include <string>
#include <vector>

#include <tf/transform_broadcaster.h>
#include <tf/transform_listener.h>

class OrientationProvider
{
 public:
	 OrientationProvider();
	 ~OrientationProvider();
	 virtual bool receive() = 0;
	 std::vector<double> output;
	void onInit();

	void get_params();
	std::string imuDirectionAxis;
	std::string imuBaseBody;
	double xGroundRotDeg, yGroundRotDeg, zGroundRotDeg;
	std::vector<std::string> imuObservationOrder;
	double rate;
	
	std::string subjectDir, modelFile;
    
	static tf::TransformBroadcaster br;

};
#endif
