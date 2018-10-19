/*****************************************************************************

 Copyright (c) 2018 Institute of Robotics and Mechatronics, DLR


****************************************************************************//**

 \file Polyhedron.h
 
*****************************************************************************/
#ifndef POLYHEDRON_H_
#define POLYHEDRON_H_

#include "common.h"

/*
	 description format: a simplified, "Wavefront-like" file format

	nv 10 // number of vertices
	v 0.123 0.234 0.345 // List of Vertices, with (x,y,z) coordinates
	...
	nf 5 // number of faces
	f 3 1 2 3 // Polygon: number of vertices, and indices (in the vertices list, starting from "1")
	f 4 3 4 5 6
	f 4 6 3 7 8
	...
*/
namespace DLRtracker
{
class Polyhedron
{
		friend class GenericObjectTracker;
		friend class Multibody;

	public:
		
		Polyhedron(void);
		~Polyhedron(void);
		
		int init(bool isFlat, double angle_internal_edges);

		void changeLocalFrame(double * dT);

		bool isOpen()
		{
			return m_isOpen;
		};

		int getNVertices()
		{
			return m_nvertices;
		};

		double * getVertices()
		{
			return m_vertices;
		};

		double * getVerticesCamera()
		{
			return m_vertices_camera;
		};

		int getNEdges()
		{
			return m_nedges;
		};

		int * getEdges()
		{
			return m_unique_edges;
		};

		int getNFaces()
		{
			return m_nfaces;
		};

		int ** getFaces()
		{
			return m_faces;
		};

		int ** getFacesUniqueEdges()
		{
			return m_faces_unique_edges;
		};

		double * getFaceNormals()
		{
			return m_face_normals;
		};

		int * getFaceSizes()
		{
			return m_face_sizes;
		};

		unsigned char * getIsFeatureEdge()
		{
			return m_is_feature_edge;
		};

		double * getFaceCameraDistances()
		{
			return m_face_camera_distances;
		};

	protected:
		
		bool m_isOpen;
		double m_internal_edge_angle_threshold;
		int m_nvertices;
		double * m_vertices;
		int m_nedges;
		int * m_unique_edges;
		int * m_tmp_edges;
		int m_nfaces;
		int * m_face_sizes;
		int ** m_faces;
		int ** m_faces_unique_edges;
		double * m_face_normals;
		double * m_vertices_camera;
		double * m_normals_camera;
		unsigned char * m_is_feature_edge;
		double * m_face_camera_distances;
		
};

}

#endif /* POLYHEDRON_H_ */
