#include <assimp/Exporter.hpp>
#include <assimp/Importer.hpp>
#include <assimp/scene.h>
#include <assimp/postprocess.h>
#include <iostream>
#include <vtkSmartPointer.h>
#include <vtkXMLPolyDataReader.h>
#include <vtkGenericDataObjectReader.h>
//#include <vtkOBJWriter.h> //nope, vtk 8.2 and up only. i think we are on 7.XX lol
#include <vtkSTLWriter.h>
//#include <vtkPLYWriter.h>
#include <vtkTriangleFilter.h>
#include <vtkPolyDataNormals.h>
#include <boost/filesystem.hpp>

int writeMeshAsStl(const std::string& inmesh_filename, const std::string& outmesh_filename)
{
	std::cout << "in: " << inmesh_filename << "\nout:" << outmesh_filename <<std::endl;
	auto mmesh_name = boost::filesystem::path(outmesh_filename).stem();
	auto mmesh_dir  = boost::filesystem::path(outmesh_filename).parent_path();
	std::string intermediate_mesh_name = (mmesh_dir / mmesh_name).string()+".stl"; //+".ply";

	std::string mesh_format = "collada";
	if(boost::filesystem::path(outmesh_filename).extension().string() == ".stl")
	{
		mesh_format = "stl";

	} 

	if(boost::filesystem::path(inmesh_filename).extension().string() == ".vtp")
	{
		std::cout << "vtk: "<< std::endl;
		auto reader = vtkSmartPointer<vtkXMLPolyDataReader>::New();
		//auto reader = vtkSmartPointer<vtkGenericDataObjectReader>::New();
		reader->SetFileName(inmesh_filename.c_str());
		reader->Update();
		
		
		auto tri = vtkSmartPointer<vtkTriangleFilter>::New();
		tri->SetInputConnection(reader->GetOutputPort());

		auto normals = vtkSmartPointer<vtkPolyDataNormals>::New();
		normals->SetInputConnection(tri->GetOutputPort());
		normals->ConsistencyOn();
		normals->AutoOrientNormalsOn();
		normals->SplittingOff();
		


		//auto writer = vtkSmartPointer<vtkOBJWriter>::New(); // we need at least vtk 8.2 for this afff....
		auto writer = vtkSmartPointer<vtkSTLWriter>::New();
		//auto writer = vtkSmartPointer<vtkPLYWriter>::New();
		writer->SetFileName(intermediate_mesh_name.c_str());
		//writer->SetInputConnection(reader->GetOutputPort());
		writer->SetInputConnection(normals->GetOutputPort());
		writer->Write();
		if (mesh_format == "stl")
		{
			std::cout << "final mesh created: " << intermediate_mesh_name << std::endl;
			return 0;
		}
		std::cout << "intermediate mesh created: " << intermediate_mesh_name << std::endl;
		
	}
	else intermediate_mesh_name = inmesh_filename;

	std::cout << "Assimp: "<< std::endl;
	Assimp::Importer importer;
	Assimp::Exporter exporter;
	const aiScene* scene = importer.ReadFile(intermediate_mesh_name, aiProcess_Triangulate |
			//aiProcess_GenSmoothNormals|
			aiProcess_JoinIdenticalVertices |
			aiProcess_PreTransformVertices |
			aiProcess_GenNormals
			//| aiProcess_PreTransformVertices
			//|
			//aiProcess_FlipWindingOrder
			);
	if (!scene)
	{
		std::cerr <<"import error: " << importer.GetErrorString()<<std::endl;
		return 1;
	}
	// for manually inverting normals because idk, it's not rendering right
	if (false)
		for (unsigned int m = 0; m < scene->mNumMeshes; ++m)
		{
			aiMesh* mesh = scene->mMeshes[m];

			if (mesh->HasNormals()&& false)
			{
				for (unsigned int v = 0; v < mesh->mNumVertices; ++v)
				{
					mesh->mNormals[v].x *= -1.0f;
					mesh->mNormals[v].y *= -1.0f;
					mesh->mNormals[v].z *= -1.0f;
				}
			}
			if (mesh->HasTangentsAndBitangents()&& false)
			{
				for (unsigned int v = 0; v < mesh->mNumVertices; ++v)
				{
					mesh->mTangents[v] *= -1.0f;
					mesh->mBitangents[v] *= -1.0f;
				}
			}

			for (unsigned int f = 0; f < mesh->mNumFaces; f++)
			{
				aiFace& face = mesh->mFaces[f];

				if(face.mNumIndices == 3&& false)
				{
					std::swap(face.mIndices[1],
							face.mIndices[2]);
				}
			}

		}
	/*for (unsigned int m = 0; m < scene->mNumMeshes; m++)
	  {
	  aiMesh* mesh = scene->mMeshes[m];

	  for (unsigned int v = 0; v < mesh->mNumVertices; v++)
	  {
	  mesh->mNormals[v] = aiVector3D(0,0,0);
	  }
	  }
	  */

	//the dae files are missing the materials, so it is not working. too lazy to fix this now, more important fires to put out
	if (exporter.Export(scene, mesh_format, outmesh_filename ) != AI_SUCCESS)
	{
		std::cerr << "export failed: " << exporter.GetErrorString() << std::endl;
		return 1;
	}
	std::cout << "wrote sucessfully" << outmesh_filename << std::endl;
	return 0;
}



