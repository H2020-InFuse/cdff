/**
 * @author Alessandro Bianco
 */

/**
 * @addtogroup DFNs
 * @{
 */

#ifndef POINTCLOUDASSEMBLY_NEIGHBOURSINGLEPOINTAVERAGE_HPP
#define POINTCLOUDASSEMBLY_NEIGHBOURSINGLEPOINTAVERAGE_HPP

#include "PointCloudAssemblyInterface.hpp"

#include <Types/CPP/PointCloud.hpp>
#include <Types/CPP/Pose.hpp>
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
namespace PointCloudAssembly
{
	/**
	 * Fusion of two point clouds using the neighbour point average algorithm. This algorithm uses one-one matches, instead of one-to-many matches used by the NeighbourPointAverage variant.
	 *
	 * @param maxNeighbourDistance, maximum distance for a point to be considered neighbour of another point
	 * @param incrementalMode, when this is false the output is given by the fusion of the two input clouds, when this is true only the first cloud will be used and will be fused with
	 * the an incrementally constructed stored cloud.
	 * @param incrementalMode, whether to use the distance filter or not for the point cloud output.
	 */
	class NeighbourSinglePointAverage : public PointCloudAssemblyInterface
	{
		public:

			NeighbourSinglePointAverage();
			virtual ~NeighbourSinglePointAverage();

			virtual void configure() override;
			virtual void process() override;

		private:
			struct Correspondence
				{
				int firstIndex;
				int secondIndex;
				float squaredDistance;
				};

			struct NeighbourSinglePointAverageOptionsSet
			{
				float maxNeighbourDistance;
				bool ignoreUnmatchedPointsOnFirstCloud;
				bool ignoreUnmatchedPointsOnSecondCloud;
				bool useIncrementalMode;
				bool useDistanceFilter;
			};

			Helpers::ParametersListHelper parametersHelper;
			NeighbourSinglePointAverageOptionsSet parameters;
			static const NeighbourSinglePointAverageOptionsSet DEFAULT_PARAMETERS;

			//Variable for input-output handling
			pcl::PointCloud<pcl::PointXYZ>::ConstPtr firstCloud; //first operand of the merge
			pcl::PointCloud<pcl::PointXYZ>::ConstPtr secondCloud; //second operand of the merge
			pcl::PointCloud<pcl::PointXYZ>::Ptr assembledCloud; //result of the merge

			//Variable for storage
			pcl::PointCloud<pcl::PointXYZ>::Ptr storedCloud; //result of previous operations

			//External conversion helpers
			Converters::PointCloudToPclPointCloudConverter pointCloudToPclPointCloud;

			//For some index x in first cloud, bijectiveCorrespondenceMap[x] is the index y in second cloud such that (i) the distance between x and y is smaller than maxNeighbourDistance,
			// and (ii) there is no other index y' in second cloud such that the distance between x and y' is smaller than the distance between x and y,
			// and (iii) there is nother index x' in first cloud such that the distance between x' and y is smaller than the distance between x and y.
			std::vector< Correspondence > correspondenceMap;

			// For each point P of index x of a point cloud, if x appears in correspondenceMap, then replacementMap[x] is the point that should replace P in the fused cloud.
			std::map<int, pcl::PointXYZ > firstReplacementMap, secondReplacementMap;

			//This method computes the correspondenceMap, only after the call to this method correspondenceMap will be valid.
			void ComputeCorrespondenceMap();

			//This method computes the replacement points, only after this method firstReplacementMap and secondReplacementMap are valid.
			void ComputeReplacementPoints();

			//This method assemble the final point cloud.
			void AssemblePointCloud();
			void AssembleLeftoverPoints(pcl::PointCloud<pcl::PointXYZ>::ConstPtr cloud, const std::map<int, pcl::PointXYZ >& replacementMap, bool ignoreCloseUmantchedPoints = false);

			//Other Core computation methods
			pcl::PointXYZ ComputeAveragePoint(pcl::PointCloud<pcl::PointXYZ>::ConstPtr cloud, const std::vector<int>& indexList);
			pcl::PointXYZ ComputeAveragePoint(const pcl::PointXYZ& firstPoint, const pcl::PointXYZ& secondPoint);
			pcl::PointXYZ DisplacePointBySameDistance(const pcl::PointXYZ& pointToDisplace, const pcl::PointXYZ& startDistanceReference, const pcl::PointXYZ& endDistanceReference);
			void AddNewMatchToCorrespondenceMap(Correspondence newCorrespondence);

			//This method converts the output to the expected output type.
			void PrepareOutAssembledPointCloud();

			//Input Validation methods
			void ValidateParameters();
	};
}
}
}

#endif // POINTCLOUDASSEMBLY_NEIGHBOURSINGLEPOINTAVERAGE_HPP

/** @} */
