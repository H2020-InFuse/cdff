/* --------------------------------------------------------------------------
*
* (C) Copyright …
*
* --------------------------------------------------------------------------
*/

/*!
 * @file HirschmullerDisparityMapping.hpp
 * @date 12/03/2018
 * @author Alessandro Bianco
 */

/*!
 * @addtogroup DFNs
 * 
 *  This DFN implementation uses the Hirschmuller disparity mapping algorithm for the reconstruction of 3D point clouds from images taken by a stereo camera.
 *  
 *  This DFN implementation executes the following tasks: conversion of the input images to grey-scale image, application of the Hirschmuller disparity algorithm for the computation of a disparity map,
 *  application of reprojection algorithm for the reconstruction of a point cloud from the disparity Map, and reduction of the density of the output point cloud by position sampling.
 *
 *  This implementation requires the following parameters:
 *  @param prefilter.maximum
 *  @param disparities.minimum, this is the value of the minimum disparity.
 *  @param disparities.numberOfIntervals, this is the number of disparity intervals that will be detected.
 *  @param disparities.useMaximumDifference
 *  @param disparities.maximumDifference
 *  @param disparities.speckleRange
 *  @param disparities.speckleWindow
 *  @param disparities.smoothnessParameter1
 *  @param disparities.smoothnessParameter2
 *  @param blocksMatching.blockSize, this is the dimension of the blocks that need to be matched in order to compute the disparity, it needs to be an odd number greater or equal than 5.
 *  @param blocksMatching.uniquenessRatio
 *  @param useFullScaleTwoPassAlgorithm
 *  @param pointCloudSamplingDensity, this defines the ratio between sampled point cloud and full point cloud, it has to be a number between 0 and 1. The sampled point cloud is constructed by taking the 
 *					points at positions multiple of n where n is 1/pointCloudSamplingDensity.
 *  @param useDisparityToDepthMap, this determines whether the camera parameters are provided in the form of a DisparityToDepthMatrix or in form of focal length, principle point and baseline.
 *  @param disparityToDepthMap, the camera parameter in form of a 4x4 disparity to depth matrix, each element is defined as Element_X_Y where X and Y belong to {0, 1, 2, 3}.
 *  @param stereoCameraParameters, the camera parameter represented as left camera focal length (LeftFocalLength), Left Camera principle point coordinates (LeftPrinciplePointX and LeftPrinciplePointY),
 *					and distance between the two camera (Baseline).
 *  @param reconstructionSpace, the limits on the reconstructed 3d points coordinates as LimitX, LimitY and LimitZ. A point (x,y,z) is accepted in the output cloud if -LimitX<=x<=LimitX, -LimitY<=y<=LimitY
 *					and 0<z<=LimitZ.
 *
 *  @reference, the algorithm is inspired by the paper: Hirschmuller, H. "Stereo Processing by Semiglobal Matching and Mutual Information", PAMI(30), No. 2, February 2008, pp. 328-341.
 *
 * @{
 */

#ifndef HIRSCHMULLER_DISPARITY_MAPPING_HPP
#define HIRSCHMULLER_DISPARITY_MAPPING_HPP

/* --------------------------------------------------------------------------
 *
 * Includes
 *
 * --------------------------------------------------------------------------
 */
#include <StereoReconstruction/StereoReconstructionInterface.hpp>
#include <Pose.hpp>
#include <PointCloud.hpp>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <stdlib.h>
#include <string>
#include <pcl/keypoints/harris_3d.h>
#include <yaml-cpp/yaml.h>
#include <SupportTypes.hpp>
#include "opencv2/calib3d.hpp"
#include <Helpers/ParametersListHelper.hpp>

namespace dfn_ci {

/* --------------------------------------------------------------------------
 *
 * Class definition
 *
 * --------------------------------------------------------------------------
 */
    class HirschmullerDisparityMapping : public StereoReconstructionInterface
    {
	/* --------------------------------------------------------------------
	 * Public
	 * --------------------------------------------------------------------
	 */
        public:
        	HirschmullerDisparityMapping();
        	~HirschmullerDisparityMapping();
        	void process();
        	void configure();


	/* --------------------------------------------------------------------
	 * Protected
	 * --------------------------------------------------------------------
	 */
        protected:

	/* --------------------------------------------------------------------
	 * Private
	 * --------------------------------------------------------------------
	 */	
	private:
		static const float EPSILON;

		struct ReconstructionSpace
			{
			float limitX;
			float limitY;
			float limitZ;
			};

		struct PrefilterOptionsSet
			{
			int maximum;
			};

		struct DisparitiesOptionsSet
			{
			int minimum;
			int numberOfIntervals;
			bool useMaximumDifference;
			int maximumDifference;
			int speckleRange;
			int speckleWindow;
			int smoothnessParameter1;
			int smoothnessParameter2;
			};

		struct BlocksMatchingOptionsSet
			{
			int blockSize;
			int uniquenessRatio;
			};
		
		struct StereoCameraParameters
			{
			float leftFocalLength;
			float leftPrinciplePointX;
			float leftPrinciplePointY;
			float baseline;
			};

		typedef double DisparityToDepthMap[16];
		struct HirschmullerDisparityMappingOptionsSet
			{
			ReconstructionSpace reconstructionSpace;
			PrefilterOptionsSet prefilter;
			DisparitiesOptionsSet disparities;
			BlocksMatchingOptionsSet blocksMatching;
			DisparityToDepthMap disparityToDepthMap;
			float pointCloudSamplingDensity;
			float voxelGridLeafSize;
			bool useFullScaleTwoPassAlgorithm;
			bool useDisparityToDepthMap;
			StereoCameraParameters stereoCameraParameters;
			};

		cv::Mat disparityToDepthMap;

		Helpers::ParametersListHelper parametersHelper;
		HirschmullerDisparityMappingOptionsSet parameters;
		static const HirschmullerDisparityMappingOptionsSet DEFAULT_PARAMETERS;

		cv::Mat ComputePointCloud(cv::Mat leftImage, cv::Mat rightImage);
		PointCloudWrapper::PointCloudConstPtr Convert(cv::Mat cvPointCloud);
		PointCloudWrapper::PointCloudConstPtr ConvertWithPeriodicSampling(cv::Mat cvPointCloud);
		PointCloudWrapper::PointCloudConstPtr ConvertWithVoxelFilter(cv::Mat cvPointCloud);
		cv::Mat Convert(DisparityToDepthMap disparityToDepthMap);
		cv::Mat ComputePointCloudFromDisparity(cv::Mat disparity);

		void ValidateParameters();


	/* --------------------------------------------------------------------
	 * Private Testing 
	 * --------------------------------------------------------------------
	 */
	#ifdef TESTING
		#define SAVE_DISPARITY_MATRIX(disparity) disparityMatrix = disparity
	#else
		#define SAVE_DISPARITY_MATRIX(disparity)
	#endif
    };
}
#endif
/* HirschmullerDisparityMapping.hpp */
/** @} */
