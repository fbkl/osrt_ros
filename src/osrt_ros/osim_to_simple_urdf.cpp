/**
 * @author      : $USER (osruser [at]dbefbc0c19ed)
 * @file        : osim_to_simple_urdf.cpp
 * @date     : Tuesday Nov 14, 2023 16:01:58 UTC
 */

#include <osrt_ros/osim_to_urdf.h>
#include <osrt_ros/meshasstl.h>
#include <ros/ros.h>
#include <filesystem>


SimTK::String OsimToUrdf::writeVec3(SimTK::Vec3 myvec)
{
	return std::to_string(myvec[0]) + " " +
		std::to_string(myvec[1]) + " " +
		std::to_string(myvec[2]);
}

SimTK::String OsimToUrdf::writeVec4(SimTK::Vec4 myvec)
{
	return std::to_string(myvec[0]) + " " +
		std::to_string(myvec[1]) + " " +
		std::to_string(myvec[2]) + " " +
		std::to_string(myvec[3]);
}

std::string OsimToUrdf::removeExtension(const std::string& filename) {
	size_t last_dot = filename.find_last_of('.');
	if (last_dot == std::string::npos) return filename; // no extension
	return filename.substr(0, last_dot);
}

tinyxml2::XMLDocument* OsimToUrdf::create_model(std::string osim_path, std::string additional_path)
{
	// Create URDF XML
	tinyxml2::XMLDocument* urdf = new tinyxml2::XMLDocument();
	{


		ROS_WARN("OsimToUrdf::create_model reached: I GOT HERE,, HEY BUDDY");

		// Load model
		OpenSim::Model model(osim_path);

		std::string model_name = model.getName();
		model.finalizeFromProperties();

		std::vector<OsimToUrdf::OsimLink> links;
		std::vector<OsimToUrdf::OsimJoint> joints;

		const OpenSim::BodySet& bodies = model.getBodySet();
		OpenSim::ModelVisualizer::addDirToGeometrySearchPaths(additional_path);
		// Extract links (Bodies)
		for (int i = 0; i< bodies.getSize(); ++i) {
			const OpenSim::Body& body = bodies.get(i);
			OsimToUrdf::OsimLink link;
			link.name = body.getName();

			ROS_DEBUG_STREAM("BODY: "<< link.name   << "what is going on?");


			const OpenSim::PhysicalFrame* frame = model.findComponent<OpenSim::PhysicalFrame>("/bodyset/"+link.name);

			if (frame) {
				ROS_DEBUG_STREAM("Frame: "<< link.name   << "is valid. Going to iterate over attached geometries:");
				int num_meshes = frame->getProperty_attached_geometry().size();
				for (int j= 0; j< num_meshes ; ++j)
				{
					auto mesh = dynamic_cast<const OpenSim::Mesh*>(&frame->get_attached_geometry(j));
					if (mesh) {
						OsimToUrdf::OsimLinkVisual this_viz;

						this_viz.mesh_scale = mesh->get_scale_factors();

						std::filesystem::path meshName =  mesh->get_mesh_file();
//std::filesystem::path p("c:/dir/dir/file.ext");

						ROS_INFO_STREAM("Parsing mesh: " << meshName.filename());
						
						//this_viz.mesh_filename = "/srv/data/geometry_v3.3/" + mesh->get_mesh_file(); //sadly we cant load vtp files directly into rviz so we need to convert them beforehand to stl
						//
						//auto inmesh = SimTK::Pathname::getAbsolutePathnameUsingSpecifiedWorkingDirectory(osim_path, mesh->get_mesh_file());
						std::string inmesh = "";
						SimTK::Array_<std::string> attempts;
						bool isAbsolutePath = false;
						if (OpenSim::ModelVisualizer::findGeometryFile(model, meshName.filename(), isAbsolutePath, attempts))
						{
							inmesh = attempts.back();
						}
						else {
							ROS_WARN_STREAM("Looked for the meshes in the related directories but couldn't find it. Is the Geometries directory present?");
							for (auto& attempt:attempts) ROS_WARN_STREAM( "Looked for meshes in: " << attempt );

						}

						ROS_INFO_STREAM( "The actual file, hopefully: " << inmesh << "" );

						std::string outmesh = "/tmp/" + removeExtension(meshName) + ".dae";
						//std::string outmesh = "/tmp/" + meshName.stem().string() + ".stl";

						if (writeMeshAsStl(inmesh, outmesh) !=0 ) std::cerr << "failed to convert mesh" << inmesh << std::endl;

						this_viz.mesh_filename = "file://"+outmesh ; // they seem to use the same meshes, idk
											     //this_viz.mesh_filename = "package://model_meshes/"+ model_name+ "/" + mesh->get_mesh_file(); // they seem to use the same meshes, idk
						auto& meshFrame = mesh->getFrame();
						const SimTK::Transform T_offset_parent = meshFrame.findTransformInBaseFrame();
						this_viz.mesh_offset = T_offset_parent.p();

						this_viz.mesh_color = mesh->get_Appearance().get_color();
						this_viz.mesh_opacity = mesh->get_Appearance().get_opacity();
						auto angles_ = T_offset_parent.R().convertRotationToBodyFixedXYZ(); 
						ROS_WARN_STREAM("" << angles_);
						this_viz.mesh_rpy = angles_;


						link.visuals.push_back(this_viz);
					}
				}
			}


			// After your existing frame mesh loop, still inside the body loop:
			//auto offset_frames = body.findListT<OpenSim::PhysicalOffsetFrame>();
			auto offset_frames = body.getComponentList<OpenSim::PhysicalOffsetFrame>();
			for (const auto& pof : offset_frames) {
				int num_pof_meshes = pof.getProperty_attached_geometry().size();
				for (int j = 0; j < num_pof_meshes; ++j) {
					auto mesh = dynamic_cast<const OpenSim::Mesh*>(&pof.get_attached_geometry(j));
					if (mesh) {
						OsimToUrdf::OsimLinkVisual this_viz;
						this_viz.mesh_scale = mesh->get_scale_factors();
						std::string meshName = mesh->get_mesh_file();

						std::string inmesh = "";
						SimTK::Array_<std::string> attempts;
						bool isAbsolutePath = false;
						if (OpenSim::ModelVisualizer::findGeometryFile(model, meshName, isAbsolutePath, attempts))
							inmesh = attempts.back();
						else
							ROS_WARN_STREAM("Couldn't find mesh for PhysicalOffsetFrame geometry: " << meshName);

						std::string outmesh = "/tmp/" + removeExtension(meshName) + ".stl";
						if (writeMeshAsStl(inmesh, outmesh) != 0)
							std::cerr << "failed to convert mesh " << inmesh << std::endl;

						this_viz.mesh_filename = "file://" + outmesh;

						const SimTK::Transform T_offset = pof.findTransformInBaseFrame();
						auto angles_ = T_offset.R().convertRotationToBodyFixedXYZ(); 
						ROS_WARN_STREAM("" << angles_);
						this_viz.mesh_offset = T_offset.p();
						this_viz.mesh_rpy = angles_;
						this_viz.mesh_color = mesh->get_Appearance().get_color();
						this_viz.mesh_opacity = mesh->get_Appearance().get_opacity();
						link.visuals.push_back(this_viz);
					}
				}
			}








			links.push_back(link);
		}

		// Extract joints
		const auto& jointSet = model.getJointSet();
		for (int i =0 ; i< jointSet.getSize(); ++i) {
			OsimToUrdf::OsimJoint j;
			const OpenSim::Joint& joint = jointSet.get(i); 
			j.name = joint.getName();
			auto& f_parent = joint.getParentFrame();
			auto& f_child = joint.getChildFrame();
			auto& f_real_parent = f_parent.findBaseFrame();
			auto& f_real_child = f_child.findBaseFrame();
			j.parent = f_real_parent.getName();
			if (j.parent == "ground")
				continue;
			j.child = f_real_child.getName();

			const SimTK::Transform T_offset_parent = f_parent.findTransformInBaseFrame();
			const SimTK::Transform T_offset_child = f_child.findTransformInBaseFrame();
			j.location_in_parent = T_offset_parent.p();
			j.location_in_child = T_offset_child.p();

			joints.push_back(j);
		}


		tinyxml2::XMLDeclaration* decl = urdf->NewDeclaration();
		urdf->InsertFirstChild(decl);

		tinyxml2::XMLElement* robot = urdf->NewElement("robot");
		robot->SetAttribute("name", model_name.c_str());
		urdf->InsertEndChild(robot);

		// Add links
		int k = 0;
		for (const auto& link : links) {
			tinyxml2::XMLElement* link_elem = urdf->NewElement("link");
			link_elem->SetAttribute("name", link.name.c_str());


			for (const auto& visual_i: link.visuals)
				if (!visual_i.mesh_filename.empty()) {
					tinyxml2::XMLElement* visual = urdf->NewElement("visual");
					tinyxml2::XMLElement* geometry = urdf->NewElement("geometry");
					tinyxml2::XMLElement* mesh = urdf->NewElement("mesh");

					mesh->SetAttribute("filename", visual_i.mesh_filename.c_str());

					mesh->SetAttribute("scale", writeVec3(visual_i.mesh_scale).c_str());

					geometry->InsertEndChild(mesh);
					visual->InsertEndChild(geometry);

					// Optional: Add origin
					tinyxml2::XMLElement* origin = urdf->NewElement("origin");
					origin->SetAttribute("xyz", writeVec3(visual_i.mesh_offset).c_str());
					//origin->SetAttribute("rpy", "0 0 0");
					origin->SetAttribute("rpy", writeVec3(visual_i.mesh_rpy).c_str());
					visual->InsertEndChild(origin);

					// It will look red for some reason and that upsets me.
					tinyxml2::XMLElement* material = urdf->NewElement("material");
					material->SetAttribute("name", ("bone"+std::to_string(++k)).c_str());

					tinyxml2::XMLElement* color = urdf->NewElement("color");
					//color->SetAttribute("rgba", "0.292156862745098 0.819607843137255 0.933333333333333 1" );
					color->SetAttribute("rgba", (writeVec3(visual_i.mesh_color) + " " + std::to_string(visual_i.mesh_opacity)).c_str() );
					//color->SetAttribute("rgba", (writeVec3(visual_i.mesh_color) + " 1").c_str() );
					material->InsertEndChild(color);

					visual->InsertEndChild(material);

					link_elem->InsertEndChild(visual);
				}

			robot->InsertEndChild(link_elem);
		}

		// Add joints
		for (const auto& joint : joints) {
			tinyxml2::XMLElement* joint_elem = urdf->NewElement("joint");
			joint_elem->SetAttribute("name", joint.name.c_str());
			joint_elem->SetAttribute("type", "floating"); // Simplified joint type

			tinyxml2::XMLElement* parent = urdf->NewElement("parent");
			parent->SetAttribute("link", joint.parent.c_str());

			tinyxml2::XMLElement* child = urdf->NewElement("child");
			child->SetAttribute("link", joint.child.c_str());

			tinyxml2::XMLElement* origin = urdf->NewElement("origin");
			origin->SetAttribute("xyz", writeVec3(joint.location_in_parent).c_str());
			origin->SetAttribute("rpy", "0 0 0");

			joint_elem->InsertEndChild(parent);
			joint_elem->InsertEndChild(child);
			joint_elem->InsertEndChild(origin);

			robot->InsertEndChild(joint_elem);
		}
	}
	return urdf;

}

