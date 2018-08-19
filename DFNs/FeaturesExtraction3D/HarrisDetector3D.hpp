/**
 * @author Alessandro Bianco
 */

/**
 * @addtogroup DFNs
 * @{
 */

#ifndef FEATURESEXTRACTION3D_HARRISDETECTOR3D_HPP
#define FEATURESEXTRACTION3D_HARRISDETECTOR3D_HPP

#include "FeaturesExtraction3DInterface.hpp"

#include <PointCloud.hpp>
#include <VisualPointFeatureVector3D.hpp>
#include <PointCloudToPclPointCloudConverter.hpp>
#include <Helpers/ParametersListHelper.hpp>

#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl/keypoints/harris_3d.h>
#include <yaml-cpp/yaml.h>

#include <stdlib.h>
#include <string>

namespace CDFF
{
namespace DFN
{
namespace FeaturesExtraction3D
{
	/**
	 * Extraction of keypoints in a 3D pointcloud using the Harris Detector
	 * algorithm.
	 *
	 * @param nonMaxSuppression
	 * @param radius
	 * @param searchRadius
	 * @param detectionThreshold
	 * @param enableRefinement
	 * @param numberOfThreads
	 * @param method
	 * @param outputFormat
	 *        Format of the returned pointcloud of keypoints:
	 *        * Positions:
	 *          The pointcloud contains the 3D coordinates of the keypoints
	 *        * References:
	 *          The pointcloud contains the indices of the 3D points,
	 *          as they appear in the input pointcloud
	 */
	class HarrisDetector3D : public FeaturesExtraction3DInterface
	{
		public:

			HarrisDetector3D();
			virtual ~HarrisDetector3D();

			virtual void configure();
			virtual void process();

		private:

			enum OutputFormat
			{
				POSITIONS_OUTPUT,
				REFERENCES_OUTPUT
			};
			class OutputFormatHelper : public Helpers::ParameterHelper<OutputFormat, std::string>
			{
				public:
					OutputFormatHelper(const std::string& parameterName, OutputFormat& boundVariable, const OutputFormat& defaultValue);
				private:
					OutputFormat Convert(const std::string& value);
			};

			typedef pcl::HarrisKeypoint3D<pcl::PointXYZ, pcl::PointXYZI>::ResponseMethod HarrisMethod;
			class HarrisMethodHelper : public Helpers::ParameterHelper<HarrisMethod, std::string>
			{
				public:
					HarrisMethodHelper(const std::string& parameterName, HarrisMethod& boundVariable, const HarrisMethod& defaultValue);
				private:
					HarrisMethod Convert(const std::string& value);
			};

			struct HarryOptionsSet
			{
				bool nonMaxSuppression;
				float radius;
				float searchRadius;
				float detectionThreshold;
				bool enableRefinement;
				int numberOfThreads;
				HarrisMethod method;
				OutputFormat outputFormat;
			};

			Helpers::ParametersListHelper parametersHelper;
			HarryOptionsSet parameters;
			static const HarryOptionsSet DEFAULT_PARAMETERS;

			Converters::PointCloudToPclPointCloudConverter pointCloudToPclPointCloud;

			pcl::PointIndicesConstPtr ComputeHarrisPoints(pcl::PointCloud<pcl::PointXYZ>::ConstPtr pointCloud);
			VisualPointFeatureVector3DWrapper::VisualPointFeatureVector3DConstPtr Convert(const pcl::PointCloud<pcl::PointXYZ>::ConstPtr inputCloud, const pcl::PointIndicesConstPtr indicesList);

			void ValidateParameters();
			void ValidateInputs(pcl::PointCloud<pcl::PointXYZ>::ConstPtr pointCloud);
	};
}
}
}

#endif // FEATURESEXTRACTION3D_HARRISDETECTOR3D_HPP

/** @} */
