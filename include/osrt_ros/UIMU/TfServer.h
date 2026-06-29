#ifndef TFSERVER_H_FBK20220627
#define TFSERVER_H_FBK20220627

#include <string>
#include <vector>
/*#include <stdio.h>
#include <stdlib.h>
#include <unistd.h>
#include <string.h>
#include <sys/types.h>
#include <sys/socket.h>
#include <arpa/inet.h>
#include <netinet/in.h>*/
// trying to make a class out of the example so I can use this upd server 

#include <ros/ros.h>
#include <tf/transform_listener.h>
#include "OrientationProvider.h"

class TfServer: public OrientationProvider {
 public:
	TfServer(std::vector<std::string> tf_names = {"a", "b", "c"}, std::string tf_frame_prefix = "not_set");
	~TfServer();
	bool receive() override;

	//std::vector<double> output;
	void readTransform(std::string);
	std::vector<double> readTransformIntoOpensim(std::string);
	std::vector<std::string> tf_strs;
	std::string world_tf_reference = "/map";
	void set_world_reference(std::string);
	void set_tfs(std::vector<std::string> tf_names, std::string tf_frame_prefix);
	std::map<std::string,tf::StampedTransform> last_transforms;
	tf::TransformListener listener;
	

};
#endif
