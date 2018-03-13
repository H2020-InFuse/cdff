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
 *  This DFN implements the Hirschmuller disparity mapping algorithm for the reconstruction of 3D point clouds from images taken by a stereo camera.
 *  
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

		typedef float DisparityToDepthMap[16];
		struct HirschmullerDisparityMappingOptionsSet
			{
			PrefilterOptionsSet prefilter;
			DisparitiesOptionsSet disparities;
			BlocksMatchingOptionsSet blocksMatching;
			DisparityToDepthMap disparityToDepthMap;
			float pointCloudSamplingDensity;
			bool useFullScaleTwoPassAlgorithm;
			};

		cv::Mat disparityToDepthMap;

		Helpers::ParametersListHelper parametersHelper;
		HirschmullerDisparityMappingOptionsSet parameters;
		static const HirschmullerDisparityMappingOptionsSet DEFAULT_PARAMETERS;

		cv::Mat ComputePointCloud(cv::Mat leftImage, cv::Mat rightImage);
		PointCloudWrapper::PointCloudConstPtr Convert(cv::Mat cvPointCloud);
		cv::Mat Convert(DisparityToDepthMap disparityToDepthMap);

		void ValidateParameters();
    };
}
#endif
/* HirschmullerDisparityMapping.hpp */
/** @} */
