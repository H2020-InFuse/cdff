/*****************************************************************************

 Copyright (c) 2018 Institute of Robotics and Mechatronics, DLR


****************************************************************************//**
/file GenerericObjectTracker.h
*****************************************************************************/
#ifndef GENERICOBJECTTRACKER_H_
#define GENERICOBJECTTRACKER_H_

#include "SimplePredictor.h"
#include "KalmanFilter.h"
#include "Multibody.h"
#include "ContourTracker.h"
namespace DLRtracker
{
class  GenericObjectTracker
{
	public:
		
		class TrackingParameters
		{
			public:

				TrackingParameters(void):
					npt_max_per_cam(100),
					front_polygon_thresh(1),
					pixel_subsampling(2),
                    min_segment_length(1),
					searchdist(20),
					canny_lowthresh(50),
					canny_hithresh(100),
					alpha_thresh(20.0),
					maxiter_GN(10),
					min_rot_upd(0.0),
					min_trasl_upd(0.0),
					max_rot_upd(50.0),
					max_trasl_upd(50.0),
					loss_inlier_lowthreshold(0.5),
					loss_inlier_highthreshold(0.5),
					max_final_reproj_error(1.0),
					min_rot_upd_resample(0),
					min_trasl_upd_resample(0),
					Kalman_process_std_rot(100),
					Kalman_process_std_trasl(100),
					Kalman_meas_std_rot(1),
					Kalman_meas_std_trasl(1),
					Kalman_init_cov(1000)
				{

				};

				
				int npt_max_per_cam;

				
				double front_polygon_thresh;

				
				int pixel_subsampling;

				
				double min_segment_length;

				
				int searchdist;

				
				int canny_lowthresh;
				int canny_hithresh;
				
				double alpha_thresh;

				int maxiter_GN;

				double min_rot_upd, min_trasl_upd;

				
				double max_rot_upd, max_trasl_upd;

				
				double loss_inlier_lowthreshold;
				double loss_inlier_highthreshold;
				
				double max_final_reproj_error;

				double min_rot_upd_resample;
				double min_trasl_upd_resample;
				
				double Kalman_process_std_rot;
				double Kalman_process_std_trasl;
				
				double Kalman_meas_std_rot;
				double Kalman_meas_std_trasl;
				double Kalman_init_cov;
		};

		
		GenericObjectTracker(void);
		~GenericObjectTracker(void);

		int setupFromGlobalArray(const int total_count, const double * big_parse_array);

		int getActiveCameraFlag(int camIdx)
		{
			if((camIdx>=0)&&(camIdx<m_ncams))
				return m_contourTracker.m_IsCameraActive[camIdx];
			else
			{
#ifdef _USE_STD
				std::cout << "ERROR: invalid camera index!" << std::endl;
#endif
				return _FAILURE_INTERNAL;
			}
		};

		int setActiveCameraFlag(int camIdx, int val)
		{
			if((camIdx>=0)&&(camIdx<m_ncams))
			{
				m_contourTracker.m_IsCameraActive[camIdx] = val;
				return 0;
			}
			else
			{
#ifdef _USE_STD
				std::cout << "ERROR: invalid camera index!" << std::endl;
#endif
				return _FAILURE_INTERNAL;
			}
		};

		
		int poseUpdateLoop(double * T_meas,
				   double * cov_mat,
				   const double * degreesOfFreedom,
				   bool timing,
				   bool debug_show,
				   bool diagnostics);
		int poseEstimation(unsigned char ** imgs,
				   double time_images,
				   double * T_egoMotion,
				   double * T_init0,
				   double * vel_init0,
				   double time0,
				   double * T_est,
				   double * vel_est,
				   double * ErrCov,
				   const double * degreesOfFreedom,
				   bool initialize,
				   bool timing,
				   bool debug_show,
				   bool diagnostics);
		
		void initAllUndistortMaps(void);

		void undistortCameraImage(unsigned char * img, unsigned char * und_img, int camIdx);

		void undistortAllImages(unsigned char ** imgs);

		int edgeDetectAllImages()
		{
			return m_contourTracker.edgeDetect(m_und_imgs, m_param.canny_lowthresh, m_param.canny_hithresh);
		};

		void setCannyThresholds(int lowt, int hight)
		{
			m_param.canny_lowthresh = lowt;
			m_param.canny_hithresh = hight;
		}
		
		void drawResult(double * T_wo, unsigned char * img_color, int camIdx, bool drawLocalFrame, double frame_axis_length, double * T_o_frame = NULL);

		double getReprojectionError(void) const
		{
			return m_contourTracker.m_GlobalError;
		}

		ContourTracker& getContourTracker(void)
		{
			return m_contourTracker;
		}

		TrackingParameters& getParam(void)
		{
			return m_param;
		}

		double * getKc(int camIdx)
		{
			if((camIdx>=0)&&(camIdx<m_ncams))
				return m_K_c[camIdx];
			else
			{
#ifdef _USE_STD
				std::cout << "getKc: INCORRECT CAMERA NUMBER!" << std::endl;
#endif
				return NULL;
			}
		}

		double * getDc(int camIdx)
		{
			if((camIdx>=0)&&(camIdx<m_ncams))
				return m_D_c[camIdx];
			else
			{
#ifdef _USE_STD
				std::cout << "getDc: INCORRECT CAMERA NUMBER!" << std::endl;
#endif
				return NULL;
			}
		}

		double * getTc(int camIdx)
		{
			if((camIdx>=0)&&(camIdx<m_ncams))
				return m_T_cw[camIdx];
			else
			{
#ifdef _USE_STD
				std::cout << "getTc: INCORRECT CAMERA NUMBER!" << std::endl;
#endif
				return NULL;
			}
		}

		Multibody& getObjectModel(void)
		{
			return m_objectModel;
		}

		int getNcams(void) const {return m_ncams;};

		int getXres(int camIdx) const
		{
			if((camIdx>=0)&&(camIdx<m_ncams))
				return m_xres[camIdx];
			else
			{
#ifdef _USE_STD
				std::cout << "getXres: INCORRECT CAMERA NUMBER!" << std::endl;
#endif
				return _FAILURE_INTERNAL;
			}
		}

		int getYres(int camIdx) const
		{
			if((camIdx>=0)&&(camIdx<m_ncams))
				return m_yres[camIdx];
			else
			{
#ifdef _USE_STD
				std::cout << "getYres: INCORRECT CAMERA NUMBER!" << std::endl;
#endif
				return _FAILURE_INTERNAL;
			}
		}

		double getZmin(int camIdx) const
		{
			if((camIdx>=0)&&(camIdx<m_ncams))
				return m_zmin[camIdx];
			else
			{
#ifdef _USE_STD
				std::cout << "getZmin: INCORRECT CAMERA NUMBER!" << std::endl;
#endif
				return _FAILURE_INTERNAL;
			}
		}

		double getZmax(int camIdx) const
		{
			if((camIdx>=0)&&(camIdx<m_ncams))
				return m_zmax[camIdx];
			else
			{
#ifdef _USE_STD
				std::cout << "getZmax: INCORRECT CAMERA NUMBER!" << std::endl;
#endif
				return _FAILURE_INTERNAL;
			}
		}

		unsigned char * getUndistortedImage(int camIdx)
		{
			if((camIdx>=0)&&(camIdx<m_ncams))
				return m_und_imgs[camIdx];
			else
			{
#ifdef _USE_STD
				std::cout << "getUndistortedImage: INCORRECT CAMERA NUMBER!" << std::endl;
#endif
				return NULL;
			}
		}

		void setPreProcess(bool doPreProcess)
		{
			m_doPreProcess = doPreProcess;
		};

	private:

		int setupCameraParam(int& counter, const double * big_parse_array);
		int setupTrackerParam(int& counter, const double * big_parse_array);
		int setupModelParam(int& counter, const double * big_parse_array);
		int setupPolyhedron(int& counter, const double * big_parse_array, Polyhedron& polyhedron, bool isFlat, double creaseAngle);
		int setupPolyline(int& counter, const double * big_parse_array, Polyline& polyline);

		int m_ncams;
		int m_xres[_MAX_NCAMS];
		int m_yres[_MAX_NCAMS];
		double m_zmin[_MAX_NCAMS];
		double m_zmax[_MAX_NCAMS];
		double * m_K_c[_MAX_NCAMS];
		double * m_D_c[_MAX_NCAMS];
		double * m_T_cw[_MAX_NCAMS];
		double * m_und_maps1[_MAX_NCAMS];
		double * m_und_maps2[_MAX_NCAMS];
		int * m_und_w1tables[_MAX_NCAMS];
		int * m_und_w2tables[_MAX_NCAMS];
		int * m_und_w3tables[_MAX_NCAMS];
		int * m_und_w4tables[_MAX_NCAMS];
		int * m_und_offtables[_MAX_NCAMS];
		int m_do_undistort[_MAX_NCAMS];
		unsigned char * m_und_imgs[_MAX_NCAMS];
		ContourTracker m_contourTracker;
		Multibody m_objectModel;
		TrackingParameters m_param;
		bool m_doPreProcess;
		KalmanFilter m_KalmanFilter;
		SimplePredictor m_SimplePredictor;
};

}
#endif /* GENERICOBJECTTRACKER_H_ */
