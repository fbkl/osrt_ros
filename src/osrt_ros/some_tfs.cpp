/**
 * @author      : $USER (osruser [at]dbefbc0c19ed)
 * @file        : some_tfs.cpp
 * @date     : Tuesday Nov 14, 2023 16:01:58 UTC
 */

#include <OpenSim/OpenSim.h>
#include <tinyxml2.h>
#include <iostream>
#include <fstream>

using namespace OpenSim;
using namespace SimTK;
using namespace tinyxml2;
using namespace std;

struct OsimLinkVisual {
    std::string name;
    std::string mesh_filename;
    Vec3 mesh_offset;
    Vec4 mesh_ori;
};
struct OsimLink {
    std::string name;
    std::vector<OsimLinkVisual> visuals;
};

struct OsimJoint {
    std::string name;
    std::string parent;
    std::string child;
    Vec3 location_in_parent;
    Vec3 location_in_child;
};

int main(int argc, char* argv[]) {
    if (argc < 2) {
        std::cerr << "Usage: " << argv[0] << " model.osim\n";
        return 1;
    }

    std::string osim_path = argv[1];

    // Load model
    Model model(osim_path);

    std::string model_name = model.getName();
    model.finalizeFromProperties();

    std::vector<OsimLink> links;
    std::vector<OsimJoint> joints;

    const OpenSim::BodySet& bodies = model.getBodySet();
    // Extract links (Bodies)
    for (int i = 0; i< bodies.getSize(); ++i) {
	const OpenSim::Body& body = bodies.get(i);
        OsimLink link;
        link.name = body.getName();
	
	const OpenSim::PhysicalFrame* frame = model.findComponent<OpenSim::PhysicalFrame>("/bodyset/"+link.name);

        if (frame) {
	    int num_meshes = frame->getProperty_attached_geometry().size();
            for (int j= 0; j< num_meshes ; ++j)
	    {
		    auto mesh = dynamic_cast<const Mesh*>(&frame->get_attached_geometry(j));
		    if (mesh) {
			    OsimLinkVisual this_viz;
			    //this_viz.mesh_filename = "package://"+ model_name+ "/description/meshes/" + mesh->get_mesh_file();
			    this_viz.mesh_filename = "/srv/data/geometry_v3.3/" + mesh->get_mesh_file();
			    auto& meshFrame = mesh->getFrame();
			    const SimTK::Transform T_offset_parent = meshFrame.findTransformInBaseFrame();
			    this_viz.mesh_offset = T_offset_parent.p();
			    link.visuals.push_back(this_viz);
		    }
	    }
	}

	links.push_back(link);
    }

    // Extract joints
    const auto& jointSet = model.getJointSet();
    for (int i =0 ; i< jointSet.getSize(); ++i) {
	    OsimJoint j;
	    const OpenSim::Joint& joint = jointSet.get(i); 
	    j.name = joint.getName();
	    auto& f_parent = joint.getParentFrame();
	    auto& f_child = joint.getChildFrame();
	    auto& f_real_parent = f_parent.findBaseFrame();
	    auto& f_real_child = f_child.findBaseFrame();
	    j.parent = f_real_parent.getName();
	    j.child = f_real_child.getName();

	    const SimTK::Transform T_offset_parent = f_parent.findTransformInBaseFrame();
	    const SimTK::Transform T_offset_child = f_child.findTransformInBaseFrame();
	    j.location_in_parent = T_offset_parent.p();
	    j.location_in_child = T_offset_child.p();

	    joints.push_back(j);
    }

    // Create URDF XML
    tinyxml2::XMLDocument urdf;

    XMLDeclaration* decl = urdf.NewDeclaration();
    urdf.InsertFirstChild(decl);

    XMLElement* robot = urdf.NewElement("robot");
    robot->SetAttribute("name", "converted_model");
    urdf.InsertEndChild(robot);

    // Add links
    for (const auto& link : links) {
	    XMLElement* link_elem = urdf.NewElement("link");
	    link_elem->SetAttribute("name", link.name.c_str());

	    for (const auto& visual_i: link.visuals)
	    if (!visual_i.mesh_filename.empty()) {
		    XMLElement* visual = urdf.NewElement("visual");
		    XMLElement* geometry = urdf.NewElement("geometry");
		    XMLElement* mesh = urdf.NewElement("mesh");

		    mesh->SetAttribute("filename", visual_i.mesh_filename.c_str());

		    geometry->InsertEndChild(mesh);
		    visual->InsertEndChild(geometry);

		    // Optional: Add origin
		    XMLElement* origin = urdf.NewElement("origin");
		    origin->SetAttribute("xyz",
				    (std::to_string(visual_i.mesh_offset[0]) + " " +
				     std::to_string(visual_i.mesh_offset[1]) + " " +
				     std::to_string(visual_i.mesh_offset[2])).c_str());
		    origin->SetAttribute("rpy", "0 0 0");
		    visual->InsertEndChild(origin);

		    link_elem->InsertEndChild(visual);
	    }

	    robot->InsertEndChild(link_elem);
    }

    // Add joints
    for (const auto& joint : joints) {
	    XMLElement* joint_elem = urdf.NewElement("joint");
	    joint_elem->SetAttribute("name", joint.name.c_str());
	    joint_elem->SetAttribute("type", "floating"); // Simplified joint type

	    XMLElement* parent = urdf.NewElement("parent");
	    parent->SetAttribute("link", joint.parent.c_str());

	    XMLElement* child = urdf.NewElement("child");
	    child->SetAttribute("link", joint.child.c_str());

	    XMLElement* origin = urdf.NewElement("origin");
	    origin->SetAttribute("xyz",
			    (std::to_string(joint.location_in_parent[0]) + " " +
			     std::to_string(joint.location_in_parent[1]) + " " +
			     std::to_string(joint.location_in_parent[2])).c_str());
	    origin->SetAttribute("rpy", "0 0 0");

	    joint_elem->InsertEndChild(parent);
	    joint_elem->InsertEndChild(child);
	    joint_elem->InsertEndChild(origin);

	    robot->InsertEndChild(joint_elem);
    }

    // Save to file
    XMLError eResult = urdf.SaveFile("converted_model.urdf");
    if (eResult != XML_SUCCESS) {
	    std::cerr << "Failed to write URDF file.\n";
	    return 1;
    }

    std::cout << "URDF written to converted_model.urdf\n";
    return 0;
}

