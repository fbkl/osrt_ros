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

// --- Diagnostic switches for isolating the iiwa14 mesh-orientation bug (2026-07-29) ---
// Flip these, rebuild, and compare in rviz. Meant to be temporary - remove once the
// root cause is confirmed and folded into a real fix.
//
// MESHCONV_SKIP_OBJ_INTERMEDIATE: when 1, osim_to_simple_urdf.cpp bypasses the .obj
// baked into the .osim by urdf_to_osim.cpp and converts the ORIGINAL iiwa_description
// .dae straight to .stl instead. Isolates whether the double Assimp round-trip
// (dae->obj->stl) is corrupting geometry, vs. something inherent to leaving .dae at all.
#ifndef MESHCONV_SKIP_OBJ_INTERMEDIATE
#define MESHCONV_SKIP_OBJ_INTERMEDIATE 0
#endif

// MESHCONV_APPLY_PRETRANSFORM_VERTICES: when 0, drops aiProcess_PreTransformVertices
// from the generic Assimp import pass below. That flag bakes any scene-graph node
// transform into vertex data on import; it's worth ruling in/out on its own.
#ifndef MESHCONV_APPLY_PRETRANSFORM_VERTICES
#define MESHCONV_APPLY_PRETRANSFORM_VERTICES 1
#endif

// MESHCONV_APPLY_DAE_YUP_CORRECTION: when 1, manually rotates -90deg about X for
// .dae sources before export. rviz's own COLLADA loader reads a mesh's <up_axis> tag
// and applies exactly this correction for Y_UP files (all of iiwa_description's link
// meshes are Y_UP) - a correction that never happens once a mesh leaves .dae format,
// which this pipeline always does. This replicates that correction manually so it
// survives the conversion to .obj/.stl.
#ifndef MESHCONV_APPLY_DAE_YUP_CORRECTION
#define MESHCONV_APPLY_DAE_YUP_CORRECTION 0
#endif

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
	unsigned int import_flags = aiProcess_Triangulate |
			//aiProcess_GenSmoothNormals|
			aiProcess_JoinIdenticalVertices |
			aiProcess_GenNormals
			//|
			//aiProcess_FlipWindingOrder
			;
#if MESHCONV_APPLY_PRETRANSFORM_VERTICES
	import_flags |= aiProcess_PreTransformVertices;
#endif
	const aiScene* scene = importer.ReadFile(intermediate_mesh_name, import_flags);
	if (!scene)
	{
		std::cerr <<"import error: " << importer.GetErrorString()<<std::endl;
		return 1;
	}

#if MESHCONV_APPLY_DAE_YUP_CORRECTION
	// rviz's own COLLADA loader reads <up_axis> and applies this same correction
	// (Y_UP -> Z_UP) for .dae display; that never happens once we've left .dae format,
	// so replicate it manually here before export.
	if (boost::filesystem::path(intermediate_mesh_name).extension().string() == ".dae")
	{
		aiMatrix4x4 yup_to_zup;
		aiMatrix4x4::RotationX(-AI_MATH_HALF_PI_F, yup_to_zup);
		scene->mRootNode->mTransformation = yup_to_zup * scene->mRootNode->mTransformation;
		std::cout << "applied DAE Y_UP->Z_UP correction to " << intermediate_mesh_name << std::endl;
	}
#endif
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



