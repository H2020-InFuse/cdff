/* --------------------------------------------------------------------------
*
* (C) Copyright …
*
* --------------------------------------------------------------------------
*/

/*!
 * @file ScanlineOptimization.hpp
 * @date 15/03/2018
 * @author Alessandro Bianco
 */

/*!
 * @addtogroup DFNs
 * 
 *  This DFN implementation uses the scanline optimization algorithm for the reconstruction of 3D point clouds from images taken by a stereo camera.
 *  
 *
 * @{
 */

/*!
 * @addtogroup DFNs
 * 
 *  This DFN implementation uses the Adaptive Cost 2-pass Scanline Optimization disparity mapping algorithm (by Tombari) for the reconstruction of 3D point clouds from images taken by a stereo camera.
 *  
 *  This DFN implementation executes the following tasks: conversion of the input images to pcl representation, application of the Tombari disparity algorithm for the computation of a disparity map,
 *  application of reprojection algorithm for the reconstruction of a point cloud from the disparity Map, and reduction of the density of the output point cloud by position sampling.
 *
 *  This implementation requires the following parameters:
 *  @param costAggregationRadius
 *  @param spatialBandwidth
 *  @param colorBandwidth
 *  @param weakSmoothnessPenalty
 *  @param strongSmoothnessPenalty
 *  @param matchingOptionsSet.numberOfDisparities, this is the number of disparity intervals that will be detected.
 *  @param matchingOptionsSet.horizontalOffset
 *  @param matchingOptionsSet.ratioFilter
 *  @param matchingOptionsSet.peakFilter
 *  @param matchingOptionsSet.usePreprocessing
 *  @param matchingOptionsSet.useLeftRightConsistencyCheck
 *  @param matchingOptionsSet.leftRightConsistencyThreshold
 *  @param pointCloudSamplingDensity, this defines the ratio between sampled point cloud and full point cloud, it has to be a number between 0 and 1. The sampled point cloud is constructed by taking the 
 *					points at positions multiple of n where n is 1/pointCloudSamplingDensity.
 *  @param stereoCameraParameters, the camera parameter represented as left camera focal length (LeftFocalLength), Left Camera principle point coordinates (LeftPrinciplePointX and LeftPrinciplePointY),
 *					and distance between the two camera (Baseline).
 *  @param reconstructionSpace, the limits on the reconstructed 3d points coordinates as LimitX, LimitY and LimitZ. A point (x,y,z) is accepted in the output cloud if -LimitX<=x<=LimitX, -LimitY<=y<=LimitY
 *					and 0<z<=LimitZ.
 *
 *  @references, the algorithm is inspired by the paper: [1] L. Wang et al., "High Quality Real-time Stereo using Adaptive Cost Aggregation and Dynamic Programming". 
 *
 * @{
 */

#ifndef SCANLINE_OPTIMIZATION_HPP
#define SCANLINE_OPTIMIZATION_HPP

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
#include <Helpers/ParametersListHelper.hpp>

namespace dfn_ci {

/* --------------------------------------------------------------------------
 *
 * Class definition
 *
 * --------------------------------------------------------------------------
 */
    class ScanlineOptimization : public StereoReconstructionInterface
    {
	/* --------------------------------------------------------------------
	 * Public
	 * --------------------------------------------------------------------
	 */
        public:
        	ScanlineOptimization();
        	~ScanlineOptimization();
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
		typedef pcl::PointCloud<pcl::RGB> PclImage;
		typedef pcl::PointCloud<pcl::RGB>::Ptr PclImagePtr;
		typedef pcl::PointCloud<pcl::RGB>::ConstPtr PclImageConstPtr;

		typedef pcl::PointCloud<pcl::PointXYZ> PclPointCloud;
		typedef pcl::PointCloud<pcl::PointXYZ>::Ptr PclPointCloudPtr;
		typedef pcl::PointCloud<pcl::PointXYZ>::ConstPtr PclPointCloudConstPtr;

		struct ReconstructionSpace
			{
			float limitX;
			float limitY;
			float limitZ;
			};

		struct CameraParameters
			{
			float principlePointX;
			float principlePointY;
			float focalLength;
			float baseline;
			};

		struct MatchingOptionsSet
			{
			int numberOfDisparities;
			int horizontalOffset;
			int ratioFilter;
			int peakFilter;
			bool usePreprocessing;
			bool useLeftRightConsistencyCheck;
			int leftRightConsistencyThreshold;
			};

		struct ScanlineOptimizationOptionsSet
			{
			int costAggregationRadius;
			int spatialBandwidth;
			int colorBandwidth;
			int strongSmoothnessPenalty;
			int weakSmoothnessPenalty;
			float pointCloudSamplingDensity;
			MatchingOptionsSet matchingOptionsSet;
			CameraParameters cameraParameters;
			ReconstructionSpace reconstructionSpace;
			};

		Helpers::ParametersListHelper parametersHelper;
		ScanlineOptimizationOptionsSet parameters;
		static const ScanlineOptimizationOptionsSet DEFAULT_PARAMETERS;

		PclPointCloudPtr ComputePointCloud(PclImagePtr leftImage, PclImagePtr rightImage);
		PclImagePtr Convert(FrameWrapper::FrameConstPtr frame);
		PointCloudWrapper::PointCloudConstPtr SampleCloud(PclPointCloudConstPtr pointCloud);

		void ValidateParameters();
    };
}
#endif
/* ScanlineOptimization.hpp */
/** @} */
