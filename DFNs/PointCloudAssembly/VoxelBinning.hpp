/**
 * @author Alessandro Bianco
 */

/**
 * @addtogroup DFNs
 * @{
 */

#ifndef POINTCLOUDASSEMBLY_VOXELBINNING_HPP
#define POINTCLOUDASSEMBLY_VOXELBINNING_HPP

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
	 * Fusion of two point clouds using voxel binnig.
	 *
	 * @param voxelResolution, all points are replaced by their voxel center, each voxel is a cube of dimension given by voxelResolution
	 * @param incrementalMode, when this is false the output is given by the fusion of the two input clouds, when this is true only the first cloud will be used and will be fused with
	 * the an incrementally constructed stored cloud;
	 * @param useDistanceFilter, whether the output point cloud has to be filtered by distance from a give center.
	 */
	class VoxelBinning : public PointCloudAssemblyInterface
	{
		public:

			VoxelBinning();
			virtual ~VoxelBinning();

			virtual void configure() override;
			virtual void process() override;

		private:
			//DFN Parameters
			struct VoxelBinningOptionsSet
			{
				float voxelResolution;
				bool useIncrementalMode;
				bool useDistanceFilter;
			};

			Helpers::ParametersListHelper parametersHelper;
			VoxelBinningOptionsSet parameters;
			static const VoxelBinningOptionsSet DEFAULT_PARAMETERS;

			//Variable for input-output handling
			pcl::PointCloud<pcl::PointXYZ>::ConstPtr firstCloud; //first operand of the merge
			pcl::PointCloud<pcl::PointXYZ>::ConstPtr secondCloud; //second operand of the merge
			pcl::PointCloud<pcl::PointXYZ>::Ptr assembledCloud; //result of the merge

			//Variable for storage
			pcl::PointCloud<pcl::PointXYZ>::Ptr storedCloud; //result of previous operations

			//External conversion helpers
			Converters::PointCloudToPclPointCloudConverter pointCloudToPclPointCloud;

			// For each point P of index x of a point cloud, if x appears in correspondenceMap, then replacementMap[x] is the point that should replace P in the fused cloud.
			std::map<int, pcl::PointXYZ > firstReplacementMap, secondReplacementMap;

			//This method assemble the final point cloud.
			void AssemblePointCloud();

			//This method converts the output to the expected output type.
			void PrepareOutAssembledPointCloud();

			//Input Validation methods
			void ValidateParameters();
	};
}
}
}

#endif // POINTCLOUDASSEMBLY_VOXELBINNING_HPP

/** @} */
