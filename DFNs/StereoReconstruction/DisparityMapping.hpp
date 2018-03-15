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
 *  This DFN implements the disparity mapping algorithm for the reconstruction of 3D point clouds from images taken by a stereo camera.
 *  
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
			};

		cv::Mat disparityToDepthMap;

		Helpers::ParametersListHelper parametersHelper;
		DisparityMappingOptionsSet parameters;
		static const DisparityMappingOptionsSet DEFAULT_PARAMETERS;

		cv::Mat ComputePointCloud(cv::Mat leftImage, cv::Mat rightImage);
		PointCloudWrapper::PointCloudConstPtr Convert(cv::Mat cvPointCloud);
		cv::Mat Convert(DisparityToDepthMap disparityToDepthMap);

		void ValidateParameters();
    };
}
#endif
/* DisparityMapping.hpp */
/** @} */
