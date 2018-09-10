/**
 * @author Alessandro Bianco
 */

/**
 * @addtogroup DFNs
 * @{
 */

#ifndef POINTCLOUDASSEMBLY_NEIGHBOURPOINTAVERAGE_HPP
#define POINTCLOUDASSEMBLY_NEIGHBOURPOINTAVERAGE_HPP

#include "PointCloudAssemblyInterface.hpp"

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
namespace PointCloudAssembly
{
	/**
	 * Fusion of two point clouds using the neighbour point average algorithm.
	 *
	 * @param maxNeighbourDistance, maximum distance for a point to be considered neighbour of another point
	 */
	class NeighbourPointAverage : public PointCloudAssemblyInterface
	{
		public:

			NeighbourPointAverage();
			virtual ~NeighbourPointAverage();

			virtual void configure();
			virtual void process();

		private:
			struct NeighbourPointAverageOptionsSet
			{
				float maxNeighbourDistance;
			};

			Helpers::ParametersListHelper parametersHelper;
			NeighbourPointAverageOptionsSet parameters;
			static const NeighbourPointAverageOptionsSet DEFAULT_PARAMETERS;

			Converters::PointCloudToPclPointCloudConverter pointCloudToPclPointCloud;
			pcl::PointCloud<pcl::PointXYZ>::ConstPtr firstCloud;
			pcl::PointCloud<pcl::PointXYZ>::ConstPtr secondCloud;

			//correspondenceMap: For each point of index x in firstCloud, if correspondenceMap.at(x) is the list of points y in secondCloud, such that x is the 
			//closest neighbour of y in firstCloud.
			std::vector< std::vector<int> > correspondenceMap;

			// partitions of input clouds in two partial clouds, fusedcloud contains the points that appear in correspondenceMap, leftover cloud contains the 
			// point that do not appear in correspondenceMap
			pcl::PointCloud<pcl::PointXYZ>::Ptr firstFusedCloud, firstLeftoverCloud;
			pcl::PointCloud<pcl::PointXYZ>::Ptr secondFusedCloud, secondLeftoverCloud;

			// For each point P of index x of a point cloud, if x appears in correspondenceMap, then replacementMap[x] is the point that should replace P in the fused cloud.
			std::map<int, pcl::PointXYZ > firstReplacementMap, secondReplacementMap;

			void ComputeCorrespondenceMap();
			void ComputeReplacementPoints();
			void AssemblePointCloud();
			void AssembleLeftoverPoints(pcl::PointCloud<pcl::PointXYZ>::ConstPtr cloud, const std::map<int, pcl::PointXYZ >& replacementMap);

			pcl::PointXYZ ComputeAveragePoint(pcl::PointCloud<pcl::PointXYZ>::ConstPtr cloud, const std::vector<int>& indexList);
			pcl::PointXYZ ComputeAveragePoint(const pcl::PointXYZ& firstPoint, const pcl::PointXYZ& secondPoint);
			pcl::PointXYZ DisplacePointBySameDistance(const pcl::PointXYZ& pointToDisplace, const pcl::PointXYZ& startDistanceReference, const pcl::PointXYZ& endDistanceReference);

			void ValidateParameters();
	};
}
}
}

#endif // POINTCLOUDASSEMBLY_NEIGHBOURPOINTAVERAGE_HPP

/** @} */
