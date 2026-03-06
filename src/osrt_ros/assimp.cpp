#include "osrt_ros/meshasstl.h"

int main(int argc, char* argv[])
{

	std::string inmesh_filename = "/srv/host_data/foot_r.obj";
	std::string outmesh_filename = "/srv/host_data/foot_r.stl" ;

	writeMeshAsStl(inmesh_filename, outmesh_filename);	
	
	std::string inmesh_filename2 = "/srv/host_data/ulna.vtp";
	std::string outmesh_filename2 = "/srv/host_data/test2.stl" ;

	return writeMeshAsStl(inmesh_filename2, outmesh_filename2);	
}

