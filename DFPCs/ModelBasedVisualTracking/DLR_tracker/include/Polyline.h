/*****************************************************************************

 Copyright (c) 2018 Institute of Robotics and Mechatronics, DLR

****************************************************************************//**

 \file Polyline.h
 

*****************************************************************************/
#ifndef POLYLINE_H_
#define POLYLINE_H_

#include "common.h"

/*
	 Polyline description format:

	nv 10 // number of vertices
	v 0.123 0.234 0.345 // List of Vertices, with (x,y,z) coordinates
	...
	ne 5 // number of edges
	e 1 2 // Segment = unordered pair of vertices (indices, starting from "1")
	e 2 3
	e 4 5
	...
*/
namespace DLRtracker
{
class Polyline
{

		friend class GenericObjectTracker;
		friend class Multibody;

	public:

		Polyline(void);
		~Polyline(void);
		
		void changeLocalFrame(double * dT);

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
			return m_edges;
		};

	protected:
		
		double * m_vertices;

		int m_nvertices;

		int * m_edges;

		int m_nedges;

		double * m_vertices_camera;
};

}

#endif /* POLYLINE_H_ */
