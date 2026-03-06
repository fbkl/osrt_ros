#include <assimp/Exporter.hpp>
#include <assimp/Importer.hpp>
#include <assimp/scene.h>
#include <assimp/postprocess.h>
#include <iostream>
#include <vtkSmartPointer.h>
#include <vtkXMLPolyDataReader.h>
#include <vtkGenericDataObjectReader.h>
#include <vtkPLYWriter.h>

std::string getExtension(const std::string& path)
{
	auto pos = path.rfind(".");
	if (pos == std::string::npos) return "";
	return path.substr(pos+1);
}

int writeMeshAsStl(const std::string& inmesh_filename, const std::string& outmesh_filename)
{
	std::cout << "in: " << inmesh_filename << "out:" << outmesh_filename <<std::endl;
	std::string intermediate_mesh_name = outmesh_filename +".ply";

	if(getExtension(inmesh_filename) == "vtp")
	{
		std::cout << "vtk: "<< std::endl;
		auto reader = vtkSmartPointer<vtkXMLPolyDataReader>::New();
		//auto reader = vtkSmartPointer<vtkGenericDataObjectReader>::New();
		reader->SetFileName(inmesh_filename.c_str());
		reader->Update();

		auto writer = vtkSmartPointer<vtkPLYWriter>::New();
		writer->SetFileName(intermediate_mesh_name.c_str());
		writer->SetInputConnection(reader->GetOutputPort());
		writer->Write();
	}
	else intermediate_mesh_name = inmesh_filename;

		std::cout << "Assimp: "<< std::endl;
		Assimp::Importer importer;
		Assimp::Exporter exporter;
		std::string mesh_format = "collada";
		const aiScene* scene = importer.ReadFile(intermediate_mesh_name, aiProcess_Triangulate);
		if (!scene)
		{
			std::cerr <<"import error" << importer.GetErrorString()<<std::endl;
			return 1;
		}
		if (exporter.Export(scene, mesh_format, outmesh_filename ) != AI_SUCCESS)
		{
			std::cerr << "export failed" << exporter.GetErrorString() << std::endl;
			return 1;
		}
	return 0;
}



