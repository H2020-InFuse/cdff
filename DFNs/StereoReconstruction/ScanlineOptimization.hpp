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
