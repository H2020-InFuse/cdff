/* --------------------------------------------------------------------------
*
* (C) Copyright …
*
* --------------------------------------------------------------------------
*/

/*!
 * @file DisparityMapping.hpp
 * @date 08/03/2018
 * @author Alessandro Bianco
 */

/*!
 * @addtogroup DFNs
 * 
 *  This DFN implementation uses the Konolige disparity mapping algorithm for the reconstruction of 3D point clouds from images taken by a stereo camera.
 *  
 *  This DFN implementation executes the following tasks: conversion of the input images to grey-scale image, application of the Konolige disparity algorithm for the computation of a disparity map,
 *  application of reprojection algorithm for the reconstruction of a point cloud from the disparity Map, and reduction of the density of the output point cloud by position sampling.
 *
 *  This implementation requires the following parameters:
 *  @param prefilter.maximum
 *  @param prefilter.size
 *  @param prefilter.type
 *  @param disparities.minimum, this is the value of the minimum disparity.
 *  @param disparities.numberOfIntervals, this is the number of disparity intervals that will be detected.
 *  @param disparities.useMaximumDifference
 *  @param disparities.maximumDifference
 *  @param disparities.speckleRange
 *  @param disparities.speckleWindow
 *  @param blocksMatching.blockSize, this is the dimension of the blocks that need to be matched in order to compute the disparity, it needs to be an odd number greater or equal than 5.
 *  @param blocksMatching.uniquenessRatio
 *  @param blocksMatching.textureThreshold
 *  @param blocksMatching.smallerBlockSize
 *  @param firstRegionOfInterest
 *  @param secondRegionOfInterest
 *  @param pointCloudSamplingDensity, this defines the ratio between sampled point cloud and full point cloud, it has to be a number between 0 and 1. The sampled point cloud is constructed by taking the 
 *					points at positions multiple of n where n is 1/pointCloudSamplingDensity.
 *  @param useDisparityToDepthMap, this determines whether the camera parameters are provided in the form of a DisparityToDepthMatrix or in form of focal length, principle point and baseline.
 *  @param disparityToDepthMap, the camera parameter in form of a 4x4 disparity to depth matrix, each element is defined as Element_X_Y where X and Y belong to {0, 1, 2, 3}.
 *  @param stereoCameraParameters, the camera parameter represented as left camera focal length (LeftFocalLength), Left Camera principle point coordinates (LeftPrinciplePointX and LeftPrinciplePointY),
 *					and distance between the two camera (Baseline).
 *  @param reconstructionSpace, the limits on the reconstructed 3d points coordinates as LimitX, LimitY and LimitZ. A point (x,y,z) is accepted in the output cloud if -LimitX<=x<=LimitX, -LimitY<=y<=LimitY
 *					and 0<z<=LimitZ.
 *
 * @{
 */

#ifndef DISPARITY_MAPPING_HPP
#define DISPARITY_MAPPING_HPP

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
    class DisparityMapping : public StereoReconstructionInterface
    {
	/* --------------------------------------------------------------------
	 * Public
	 * --------------------------------------------------------------------
	 */
        public:
        	DisparityMapping();
        	~DisparityMapping();
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

		enum PrefilterType
			{
			NORMALIZED_RESPONSE,
			XSOBEL
			};
		class PrefilterTypeHelper : public Helpers::ParameterHelper<PrefilterType, std::string>
			{
			public:
				PrefilterTypeHelper(const std::string& parameterName, PrefilterType& boundVariable, const PrefilterType& defaultValue);
			private:
				PrefilterType Convert(const std::string& value);
			};

		struct ReconstructionSpace
			{
			float limitX;
			float limitY;
			float limitZ;
			};

		struct Rectangle
			{
			int topLeftColumn;
			int topLeftRow;
			int numberOfColumns;
			int numberOfRows;
			};

		struct PrefilterOptionsSet
			{
			int size;
			PrefilterType type;
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
			};

		struct BlocksMatchingOptionsSet
			{
			int blockSize;
			int smallerBlockSize;
			int textureThreshold;
			int uniquenessRatio;
			};

		struct StereoCameraParameters
			{
			float leftFocalLength;
			float leftPrinciplePointX;
			float leftPrinciplePointY;
			float baseline;
			};

		typedef float DisparityToDepthMap[16];
		struct DisparityMappingOptionsSet
			{
			ReconstructionSpace reconstructionSpace;
			PrefilterOptionsSet prefilter;
			DisparitiesOptionsSet disparities;
			Rectangle firstRegionOfInterest;
			Rectangle secondRegionOfInterest;
			BlocksMatchingOptionsSet blocksMatching;
			DisparityToDepthMap disparityToDepthMap;
			float pointCloudSamplingDensity;
			float voxelGridLeafSize;
			bool useDisparityToDepthMap;
			StereoCameraParameters stereoCameraParameters;
			};

		cv::Mat disparityToDepthMap;

		Helpers::ParametersListHelper parametersHelper;
		DisparityMappingOptionsSet parameters;
		static const DisparityMappingOptionsSet DEFAULT_PARAMETERS;

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
/* DisparityMapping.hpp */
/** @} */
