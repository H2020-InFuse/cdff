/**
 * @author Alessandro Bianco
 */

/**
 * @addtogroup DFNs
 * @{
 */

#ifndef FEATURESEXTRACTION3D_ISSDETECTOR3D_HPP
#define FEATURESEXTRACTION3D_ISSDETECTOR3D_HPP

#include "FeaturesExtraction3DInterface.hpp"

#include <Types/CPP/PointCloud.hpp>
#include <Types/CPP/VisualPointFeatureVector3D.hpp>
#include <Converters/PointCloudToPclPointCloudConverter.hpp>
#include <Helpers/ParametersListHelper.hpp>

#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl/keypoints/iss_3d.h>
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
	 * @param salientRadius
	 * @param nonMaximaSupressionRadius
	 * @param normalRadius
	 * @param firstThreshold
	 * @param secondThreshold
	 * @param minNumberOfNeighbours
	 * @param angleThreshold
	 * @param numberOfThreads
	 * @param outputFormat
	 *        Format of the returned pointcloud of keypoints:
	 *        * Positions:
	 *          The pointcloud contains the 3D coordinates of the keypoints
	 *        * References:
	 *          The pointcloud contains the indices of the 3D points,
	 *          as they appear in the input pointcloud
	 */
	class IssDetector3D : public FeaturesExtraction3DInterface
	{
		public:

			IssDetector3D();
			virtual ~IssDetector3D();

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
					OutputFormat Convert(const std::string& value) override;
			};

			//There is one more parameters in PCL documentation, borderRadius, but that is for organized clouds.
			struct IssOptionsSet
			{
				double salientRadius;
				double nonMaximaSupressionRadius;
				double normalRadius;
				double firstThreshold;
				double secondThreshold;
				int minNumberOfNeighbours;
				float angleThreshold;
				int numberOfThreads;
				OutputFormat outputFormat;
			};

			Helpers::ParametersListHelper parametersHelper;
			IssOptionsSet parameters;
			static const IssOptionsSet DEFAULT_PARAMETERS;

			Converters::PointCloudToPclPointCloudConverter pointCloudToPclPointCloud;

			pcl::PointIndicesConstPtr ComputeIssPoints(pcl::PointCloud<pcl::PointXYZ>::ConstPtr pointCloud);
			VisualPointFeatureVector3DWrapper::VisualPointFeatureVector3DConstPtr Convert(const pcl::PointCloud<pcl::PointXYZ>::ConstPtr inputCloud, 
				const pcl::PointIndicesConstPtr indicesList);

			void ValidateParameters();
			void ValidateInputs(pcl::PointCloud<pcl::PointXYZ>::ConstPtr pointCloud);
	};
}
}
}

#endif // FEATURESEXTRACTION3D_ISSDETECTOR3D_HPP

/** @} */
