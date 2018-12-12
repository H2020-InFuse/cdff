/**
 * @author Alessandro Bianco
 */

/**
 * @addtogroup DFNs
 * @{
 */

#ifndef FEATURESEXTRACTION3D_CORNERDETECTOR3D_HPP
#define FEATURESEXTRACTION3D_CORNERDETECTOR3D_HPP

#include "FeaturesExtraction3DInterface.hpp"

#include <Types/CPP/PointCloud.hpp>
#include <Types/CPP/VisualPointFeatureVector3D.hpp>
#include <Converters/PointCloudToPclPointCloudConverter.hpp>
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
	 * Extraction of keypoints at the borders between smoothness and curvature clusters
	 *
	 * @param numberOfNeighboursNormalEstimation, the number of neighbours used for normal estimation;
	 * @param numberOfNeighboursRegionGrowing, the number of neighbours visited during the region growing algorithm used for clustering;
	 * @param numberOfNeightbourBorderSelection, the number of neighbours used to evaluate whether the point is at the border between two clusters;
	 * @param smoothnessThreshold, smoothnessThreshold for the region growing, points are in the same cluster if the smoothness difference or the curvature difference are below threshold;
	 * @param curvatureThreshold, curvatureThreshold for regiong growing, points are in the same cluster if the smoothness difference or the curvature difference are below threshold;
	 * @param minimumClusterSize, minimum size of a cluster;
	 * @param maximumClusterSize, maximum size of a cluster;
	 * @param searchRadiusBorderSelection, this is the radius that determines whether the point is internal to the point cloud or at the edge. Edge points are discarded; 
	 * @param numberOfHistogramSlots, each slot is an angle range, each point will have an histogram, for each angle and point the slot will tell how many neighbours fall in that angle direction;
	 * @param maximumNumberOfEmptySlots, if a point histogram has a larger number of empty slots, then it will be considered an edge point (too many sides of the point are empty);
	 * @param outputFormat
	 *        Format of the returned pointcloud of keypoints:
	 *        * Positions:
	 *          The pointcloud contains the 3D coordinates of the keypoints
	 *        * References:
	 *          The pointcloud contains the indices of the 3D points,
	 *          as they appear in the input pointcloud
	 */
	class CornerDetector3D : public FeaturesExtraction3DInterface
	{
		public:

			CornerDetector3D();
			virtual ~CornerDetector3D();

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

			struct CornerOptionsSet
			{
				int numberOfNeighboursNormalEstimation;
				int numberOfNeighboursRegionGrowing;
				int numberOfNeightbourBorderSelection;
				float smoothnessThreshold;
				float curvatureThreshold;
				int minimumClusterSize;
				int maximumClusterSize;
				float searchRadiusBorderSelection;
				int numberOfHistogramSlots;
				int maximumNumberOfEmptySlots;
				OutputFormat outputFormat;
			};

			Helpers::ParametersListHelper parametersHelper;
			CornerOptionsSet parameters;
			static const CornerOptionsSet DEFAULT_PARAMETERS;

			Converters::PointCloudToPclPointCloudConverter pointCloudToPclPointCloud;

			pcl::PointIndicesConstPtr DetectCorners(pcl::PointCloud<pcl::PointXYZ>::ConstPtr pointCloud);
			bool PointIsInner(pcl::PointCloud<pcl::PointXYZ>::ConstPtr pointCloud, const pcl::PointXYZ& originPoint, const pcl::Normal& normal, const std::vector<int>& neighbours);
			VisualPointFeatureVector3DWrapper::VisualPointFeatureVector3DConstPtr Convert(const pcl::PointCloud<pcl::PointXYZ>::ConstPtr inputCloud, const pcl::PointIndicesConstPtr indicesList);

			void ValidateParameters();
			void ValidateInputs(pcl::PointCloud<pcl::PointXYZ>::ConstPtr pointCloud);
	};
}
}
}

#endif // FEATURESEXTRACTION3D_CORNERDETECTOR3D_HPP

/** @} */
