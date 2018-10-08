/*****************************************************************************

 Copyright (c) 2018 Institute of Robotics and Mechatronics, DLR

****************************************************************************//**

 \file trackContour.h
 
*****************************************************************************/
#ifndef TRACKCONTOUR_H_
#define TRACKCONTOUR_H_

#include "Multibody.h"

class ContourTracker
{
	public:
		ContourTracker(void);
		~ContourTracker(void);

		void init(int * xres, int * yres, double * zmin, double * zmax, int ncams, int npt_max);

		int edgeDetect(unsigned char ** imgs, int canny_lowthresh, int canny_hithresh);
		
		int poseUpdateLoop(Multibody& theModel,
					      unsigned char ** imgs,
				              double * T,
				              double * ErrCovariance,
				              double ** T_cw,
				              double ** K_cw,
				              int maxiter_GN,
				              double min_rot_upd,
				              double min_trasl_upd,
				              double max_rot_upd,
				              double max_trasl_upd,
					      double front_polygon_thresh,
					      int pixel_subsampling,
					      double min_segment_length,
				              double search_dist,
				              double alpha_thresh,
				              double loss_inlier_lowthresh,
				              double loss_inlier_highthresh,
				              double max_final_reproj_error,
				              double min_rot_upd_resample,
				              double min_trasl_upd_resample,
				              const double * filterSingVal,
							  bool timing,
				              bool debug_show,
				              bool diagnostics);

		
#ifdef DO_CONTOURTRACKER_LM
		int poseUpdateLoop_LM(Multibody& theModel,
					      unsigned char ** imgs,
				              double * T,
				              double * ErrCovariance,
				              double ** T_cw,
				              double ** K_cw,
				              int maxiter_GN,
				              double min_rot_upd,
				              double min_trasl_upd,
				              double max_rot_upd,
				              double max_trasl_upd,
					      double front_polygon_thresh,
					      int pixel_subsampling,
					      double min_segment_length,
				              double search_dist,
				              double alpha_thresh,
				              double loss_inlier_lowthresh,
				              double loss_inlier_highthresh,
				              double max_final_reproj_error,
				              double min_rot_upd_resample,
				              double min_trasl_upd_resample,
				              const double * filterSingVal,
					      bool timing,
				              bool debug_show,
					      bool diagnostics);
#endif
  		
	int computeOverallCostAndJacobian(Multibody& theModel,
					double& GlobalErr,
					const double * T,
					double ** T_cw,
					double ** K_cw,
					const double front_polygon_thresh,
					const double min_segment_length, 
					const int pixel_subsampling, 
					const double search_dist, 
					const double alpha_thresh, 
					const bool computeJacobian,
					const bool use_twist,
					const bool resample_contour,
					const bool doNormalSearch,
					const bool timing,
					const bool debug_show, 
					const bool diagnostics);

		
		int computeReprojectionAndJacobian(const Multibody& theModel, const int cam_idx, const double * T, const double * P_cw, const bool computeJacobian, const bool use_twist);

		int matchAlongNormals(const Multibody& theModel, int cam_idx, double search_dist, double alpha_thresh, bool debug_show, bool diagnostics);
   
		void correlateEdges(Multibody& theModel, unsigned char ** imgs_old, unsigned char ** imgs_new, double * T_wo, double ** K_cw, double ** T_cw);
		
		int computePoseUpdate_squareLSE(double * poseUpdate, const double * filterSingVal, bool computeErrCov, double * ErrCov);
		int computePoseUpdate_rectangularLSE(double * poseUpdate, const double * filterSingVal, bool computeErrCov, double * ErrCov);

		int computeGradientHessian(double * grad, double * Hess);
		
		bool measurement_single_line_nn_Canny(int cam_idx, int pt[2], double normal[2], double search_dist, double alpha_thresh, int bestmatch[2]);
		
		bool measurement_single_line_nn_dir_der(int cam_idx, int pt[2], double normal[2], double search_dist, double grad_thresh, double alpha_thresh, int bestmatch[2]);

		unsigned char * m_canny[_MAX_NCAMS];
		short * m_sobelx[_MAX_NCAMS];
		short * m_sobely[_MAX_NCAMS];

		double * m_reproj[_MAX_NCAMS];
		double * m_reproj_normals[_MAX_NCAMS];
		double * m_Jx[_MAX_NCAMS];
		double * m_Jy[_MAX_NCAMS];
		double * m_matched_points[_MAX_NCAMS];
		double * m_weights[_MAX_NCAMS];

		
		double * m_allResidual;
		
		double * m_allJacobian;
		
		double * m_allWeights;
		
		double m_GlobalError;

		double * m_tmp_SVD;
		
		double * m_workd_SVD;
		
		int m_npt_max_per_cam;

		int m_xres[_MAX_NCAMS];
		int m_yres[_MAX_NCAMS];

		double m_zmin[_MAX_NCAMS];
		double m_zmax[_MAX_NCAMS];
		
		int m_ncams;
		
		unsigned char * m_tmp_image;
		
		int * m_canny_buffer;
		
		unsigned char ** m_canny_stack;
		
		short * m_sobel_buffer;
		
		int m_npt_all;
		
		int m_IsCameraActive[_MAX_NCAMS];
};

#endif /* TRACKCONTOUR_H_ */
