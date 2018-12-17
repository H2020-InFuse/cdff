/**
 * @author Irene Sanz
 */

/**
 * @addtogroup DFNs
 * @{
 */

#ifndef VOXELIZATION_OCTREE_HPP
#define VOXELIZATION_OCTREE_HPP

#include "VoxelizationInterface.hpp"
#include <Helpers/ParametersListHelper.hpp>

#include <pcl/point_cloud.h>
#include <pcl/octree/octree_search.h>

#include <opencv2/core/core.hpp>
#include <opencv2/imgproc/imgproc.hpp>

namespace CDFF
{
namespace DFN
{
namespace Voxelization
{
	/**
	 * Voxelization of a 2D Depth image
	 *
	 *
	 */

	class Octree : public VoxelizationInterface
	{
		public:

			Octree();
            virtual ~Octree() = default;

			virtual void configure() override;
			virtual void process() override;

		private:
			struct OctreeOptionsSet
			{
				double minDistance;
				double maxDistance;
				double resolution;
			};

			Helpers::ParametersListHelper parametersHelper;
            OctreeOptionsSet parameters;
            static const OctreeOptionsSet DEFAULT_PARAMETERS;

			pcl::PointCloud<pcl::PointXYZ>::Ptr convertToPointCloud(const cv::Mat& img);
            void ValidateParameters();
            void ValidateInputs(const cv::Mat& inputImage);

    };
}
}
}

#endif // VOXELIZATION_OCTREE_HPP

/** @} */
