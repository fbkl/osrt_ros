/**
 * @author      : $USER (osruser [at]dbefbc0c19ed)
 * @file        : some_tfs.cpp
 * @date     : Tuesday Nov 14, 2023 16:01:58 UTC
 */
#include <osrt_ros/osim_to_urdf.h>


int main(int argc, char* argv[]) {

		//OsimToUrdf::CoutSilencer c; // when we close scope it will call the destructor
		if (argc < 2) {
			std::cerr << "Usage: " << argv[0] << " model.osim\n";
			return 1;
		}

		std::string osim_path = argv[1];
	tinyxml2::XMLDocument* urdf = OsimToUrdf::create_model(osim_path, "/srv/data/geometry_v3.3");

	if (true)
	{// Save to file
		tinyxml2::XMLError eResult = urdf->SaveFile("/tmp/converted_model.urdf");
		if (eResult != tinyxml2::XML_SUCCESS) {
			std::cerr << "Failed to write URDF file.\n";
			return 1;
		}
	}
	else
	{
		tinyxml2::XMLPrinter printer;
		urdf->Print(&printer);
		std::cout << printer.CStr();

	}
	std::cout << "URDF written to converted_model.urdf\n";
	return 0;
}

