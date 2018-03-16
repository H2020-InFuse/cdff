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
		cv::Mat Convert(DisparityToDepthMap disparityToDepthMap);
		cv::Mat ComputePointCloudFromDisparity(cv::Mat disparity);

		void ValidateParameters();
    };
}
#endif
/* HirschmullerDisparityMapping.hpp */
/** @} */
