/*****************************************************************************

 Copyright (c) 2018 Institute of Robotics and Mechatronics, DLR

 
****************************************************************************//**

 \file GenericModel.h
 
*****************************************************************************/
#ifndef GENERICMODEL_H_
#define GENERICMODEL_H_

#include "Cylinder.h"
#include "Polyhedron.h"
#include "Polyline.h"

//using namespace DLRtracker;
namespace DLRtracker
{
class Multibody
{
	public:

		friend class GenericObjectTracker;

		Multibody(void);
		~Multibody(void);

		int init(int max_views, int max_points_per_cam, int max_xres, int max_yres);

		int sampleContour_Zbuffer(const int cam,
					const double * T_co,
					const double * K,
					const int xres,
					const int yres,
					const double znear,
					const double zfar,
					const double front_polygon_thresh,
					const double min_segment_length,
					const int subsampling,
					unsigned char * rendered_image,
					unsigned char * rendered_edges,
					double * rendered_normals,
					int * sample_ids,
					bool do_sample_contour,
					bool timing,
					bool debug_show,
					bool diagnostics);

		
		int changeLocalFrame(int local_frame_index);

		void cross_check_raytracing(double * T_co, int nall_points, double * sample_points, int * sample_ids);

		double * getViewingFrustum(int camIdx)
		{
			if((camIdx<0)||(camIdx>=m_nviews))
			{
#ifdef _USE_STD
				std::cout << "ERROR: wrong camera ID" << std::endl;
#endif
				return 0;
			}
			else
				return m_viewing_frustum[camIdx];
		}

		int getNbodies(void) const
		{
			return m_nbodies;
		};

		double * getZbuffer()
		{
			return m_Zbuffer_depth_map;
		}

		double * getZbufferInit(int camIdx)
		{
			if((camIdx<0)||(camIdx>=m_nviews))
			{
#ifdef _USE_STD
				std::cout << "ERROR: wrong camera ID" << std::endl;
#endif
				return 0;
			}
			else
				return m_Zbuffer_init_depth_maps[camIdx];
		}

		double * getZbufferAllEdges()
		{
			return m_Zbuffer_all_edges;
		}

		unsigned char * getZbufferDebugShow()
		{
			return m_Zbuffer_debug_show;
		}

		int * getZbufferEdgeIds()
		{
			return m_Zbuffer_all_edge_ids;
		}

		int * getZbufferPolygonIds()
		{
			return m_Zbuffer_polygon_id_map;
		}

		double * getLocalFrame(int local_frame_index)
		{
			if((local_frame_index<0)||(local_frame_index>=m_nlocal_frames))
			{
#ifdef _USE_STD
				std::cout << "ERROR: wrong camera ID" << std::endl;
#endif
				return NULL;
			}
			else
				return m_local_frames[local_frame_index];
		}

		void resetNSamplePointsPerCam()
		{
			for(int c=0; c<m_nviews; c++)
				m_nsample_points_per_cam[c] = 0;
		};

		int getMaxSamplePointsPerCam() const
		{
			return m_max_points_per_cam;
		};

		int getNSamplePointsPerCam(int camIdx) const
		{
			if((camIdx<0)||(camIdx>=m_nviews))
			{
#ifdef _USE_STD
				std::cout << "ERROR: wrong camera ID" << std::endl;
#endif
				return 0;
			}
			else
				return m_nsample_points_per_cam[camIdx];
		};

		double * getSamplePointsPerCam(int camIdx) const
		{
			if((camIdx<0)||(camIdx>=m_nviews))
			{
#ifdef _USE_STD
				std::cout << "ERROR: wrong camera ID" << std::endl;
#endif
				return 0;
			}
			else
				return m_sample_points_per_cam[camIdx];
		};

		double * getSampleTangentDirPerCam(int camIdx) const
		{
			if((camIdx<0)||(camIdx>=m_nviews))
			{
#ifdef _USE_STD
				std::cout << "ERROR: wrong camera ID" << std::endl;
#endif
				return 0;
			}
			else
				return m_sample_tangent_directions_per_cam[camIdx];
		};

		classTags getBodyClass(int bodyIdx) const
		{
			if((bodyIdx<0)||(bodyIdx>=m_nbodies))
			{
#ifdef _USE_STD
				std::cout << "ERROR: wrong body ID" << std::endl;
#endif
				return INVALID_OBJECT_CLASS;
			}
			else
				return m_body_classes[bodyIdx];
		};

		Polyhedron * getPolyhedron(int bodyIdx)
		{
			if((bodyIdx<0)||(bodyIdx>=m_nbodies))
			{
#ifdef _USE_STD
				std::cout << "ERROR: wrong body ID" << std::endl;
#endif
				return NULL;
			}
			else if(m_body_classes[bodyIdx] != CLASS_POLYHEDRON)
			{
#ifdef _USE_STD
				std::cout << "ERROR: this part is not a polyhedron!" << std::endl;
#endif
				return NULL;
			}
			else
				return &(m_polyhedra[bodyIdx]);
		};

		Polyline * getPolyline(int bodyIdx)
		{
			if((bodyIdx<0)||(bodyIdx>=m_nbodies))
			{
#ifdef _USE_STD
				std::cout << "ERROR: wrong body ID" << std::endl;
#endif
				return NULL;
			}
			else if(m_body_classes[bodyIdx] != CLASS_POLYLINE)
			{
#ifdef _USE_STD
				std::cout << "ERROR: this part is not a polyline!" << std::endl;
#endif
				return NULL;
			}
			else
				return &(m_polylines[bodyIdx]);
		};

		int getActiveBody(int bodyIdx) const
		{
			if((bodyIdx<0)||(bodyIdx>=m_nbodies))
			{
#ifdef _USE_STD
				std::cout << "ERROR: wrong body ID" << std::endl;
#endif
				return _FAILURE_INTERNAL;
			}
			else
				return m_body_active[bodyIdx];
		};

		void setActiveBody(int bodyIdx, int val)
		{
			if((bodyIdx<0)||(bodyIdx>=m_nbodies))
			{
#ifdef _USE_STD
				std::cout << "ERROR: wrong body ID" << std::endl;
#endif
			}
			else
				m_body_active[bodyIdx] = val;
		};

		int getNOccludingROIs(int camIdx) const
		{
			if((camIdx<0)||(camIdx>=m_nviews))
			{
#ifdef _USE_STD
				std::cout << "ERROR: wrong camera ID" << std::endl;
#endif
				return 0;
			}
			else
				return m_nOcclusionROI[camIdx];
		};

		void setInitZbuffer_flag(bool initZbuffer)
		{
			m_reset_Zbuffer_externally = initZbuffer;
		};

		void setActiveOccludingROI(int camIdx, int ROI, int val)
		{
			if((camIdx<0)||(camIdx>=m_nviews))
			{
#ifdef _USE_STD
				std::cout << "ERROR: wrong camera ID" << std::endl;
#endif
				return;
			}

			if((ROI < 0)||(ROI >= m_nOcclusionROI[camIdx]))
			{
#ifdef _USE_STD
				std::cout << "ERROR: wrong ROI number" << std::endl;
#endif
				return;
			}

			m_isActive_occlusionROI[camIdx][ROI] = val;
		};

		void getPolylineAdjacentPolygon(int bIdx, int &adjacentBody, int &adjacentFace)
		{
			if((bIdx<0)||(bIdx>=m_nbodies))
			{
#ifdef _USE_STD
				std::cout << "ERROR: wrong body ID" << std::endl;
#endif
				adjacentBody = -1;
				adjacentFace = -1;
			}
			else if(m_body_classes[bIdx] != CLASS_POLYLINE)
			{
#ifdef _USE_STD
				std::cout << "ERROR: this part is not a polyline!" << std::endl;
#endif
				adjacentBody = -1;
				adjacentFace = -1;
			}
			else
			{
				adjacentBody = m_polyline_crossref_polyhedra[bIdx][0];
				adjacentFace = m_polyline_crossref_polyhedra[bIdx][1];
			}
		}

	private:
		
		int m_nviews;
		double m_viewing_frustum[_MAX_NCAMS][6*4];
		int m_nOcclusionROI[_MAX_NCAMS];
		int m_nvertices_occlusionROI[_MAX_NCAMS][_MAX_NROIS];
		int m_isActive_occlusionROI[_MAX_NCAMS][_MAX_NROIS];
		double m_occlusionROIs[_MAX_NCAMS][_MAX_NROIS][2*_MAX_NV_ROI];
		int m_max_points_per_cam;
		int m_nsample_points_per_cam[_MAX_NCAMS];
		double * m_sample_points_per_cam[_MAX_NCAMS];
		double * m_sample_tangent_directions_per_cam[_MAX_NCAMS];
		int m_nbodies;
		int m_polyline_crossref_polyhedra[_MAX_NBODIES][2];

		Polyhedron m_polyhedra[_MAX_NBODIES];
		Polyline m_polylines[_MAX_NBODIES];
		Cylinder m_cylinders[_MAX_NBODIES];

		classTags m_body_classes[_MAX_NBODIES];
		int m_body_active[_MAX_NBODIES];
		double m_current_local_frame[16];
		int m_nlocal_frames;
		double m_local_frames[_MAX_NLOCAL_FRAMES][16];
		unsigned char * m_Zbuffer_ROI_mask;
		unsigned char * m_Zbuffer_debug_show;
		double * m_Zbuffer_depth_map;
		int * m_Zbuffer_polygon_id_map;
		double * m_Zbuffer_all_edges;
		int * m_Zbuffer_all_edge_ids;
		bool m_reset_Zbuffer_externally;
		double * m_Zbuffer_init_depth_maps[_MAX_NCAMS];
};

}

#endif /* GENERICMODEL_H_ */
