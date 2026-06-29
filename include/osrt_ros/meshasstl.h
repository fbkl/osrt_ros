#include <assimp/Exporter.hpp>
#include <assimp/Importer.hpp>
#include <assimp/scene.h>
#include <assimp/postprocess.h>
#include <iostream>
#include <vtkSmartPointer.h>
#include <vtkXMLPolyDataReader.h>
#include <vtkGenericDataObjectReader.h>
#include <vtkSTLWriter.h>
#include <vtkPLYWriter.h>
#include <vtkTriangleFilter.h>
#include <vtkPolyDataNormals.h>

std::string getExtension(const std::string& path)
{
	auto pos = path.rfind(".");
	if (pos == std::string::npos) return "";
	return path.substr(pos+1);
}

int writeMeshAsStl(const std::string& inmesh_filename, const std::string& outmesh_filename)
{
	std::cout << "in: " << inmesh_filename << "out:" << outmesh_filename <<std::endl;
	std::string intermediate_mesh_name = outmesh_filename; //+".ply";

	if(getExtension(inmesh_filename) == "vtp")
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



		auto writer = vtkSmartPointer<vtkSTLWriter>::New();
		//auto writer = vtkSmartPointer<vtkPLYWriter>::New();
		writer->SetFileName(intermediate_mesh_name.c_str());
		//writer->SetInputConnection(reader->GetOutputPort());
		writer->SetInputConnection(normals->GetOutputPort());
		writer->Write();
		return 0;
	}
	else intermediate_mesh_name = inmesh_filename;

	std::cout << "Assimp: "<< std::endl;
	Assimp::Importer importer;
	Assimp::Exporter exporter;
	std::string mesh_format = "collada";
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
		std::cerr <<"import error" << importer.GetErrorString()<<std::endl;
		return 1;
	}
	// for manually inverting normals because idk, it's not rendering right
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

	if (exporter.Export(scene, mesh_format, outmesh_filename ) != AI_SUCCESS)
	{
		std::cerr << "export failed" << exporter.GetErrorString() << std::endl;
		return 1;
	}
	return 0;
}



