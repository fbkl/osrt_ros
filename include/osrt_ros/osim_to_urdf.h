/**
 * @author      : $USER (osruser [at]dbefbc0c19ed)
 * @file        : some_tfs.cpp
 * @date     : Tuesday Nov 14, 2023 16:01:58 UTC
 */
#ifndef OSIMTOURDF_H

#define OSIMTOURDF_H

#include <OpenSim/OpenSim.h>
#include <tinyxml2.h>
#include <iostream>
#include <fstream>

namespace OsimToUrdf{
struct OsimLinkVisual {
	std::string name;
	std::string mesh_filename;
	SimTK::Vec3 mesh_offset;
	SimTK::Vec3 mesh_scale;
	SimTK::Vec4 mesh_ori;
	//SimTK::Vec4 mesh_color;
	SimTK::Vec3 mesh_color;
	double mesh_opacity;
	SimTK::Vec3 mesh_rpy;
};
struct OsimLink {
	std::string name;
	std::vector<OsimLinkVisual> visuals;
};

struct OsimJoint {
	std::string name;
	std::string parent;
	std::string child;
	SimTK::Vec3 location_in_parent;
	SimTK::Vec3 location_in_child;
};

SimTK::String writeVec3(SimTK::Vec3 myvec);

SimTK::String writeVec4(SimTK::Vec4 myvec);

std::string removeExtension(const std::string& filename);


// Utility to redirect cout --- doesnt work, i cant capture the stupid stuff that opensim says!
class CoutSilencer {
	public:
		CoutSilencer() {
			// Save original buffer
			oldCoutBuf = std::cout.rdbuf();
			nullStream.open("/dev/null");
        		std::cout.rdbuf(nullStream.rdbuf());
			std::cerr.rdbuf(nullStream.rdbuf());
		}

		~CoutSilencer() {
			// Restore original buffer
			std::cout.rdbuf(oldCoutBuf);
		}

	private:
		std::ofstream nullStream;
		std::streambuf* oldCoutBuf;
};

tinyxml2::XMLDocument* create_model(std::string osim_path, std::string additional_path);
}
#endif /* end of include guard OSIMTOURDF_H */

