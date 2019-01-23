/**
 * @author Alessandro Bianco
 */

/**
 * @addtogroup DFNs
 * @{
 */

#ifndef POINTCLOUDASSEMBLY_MATCHERASSEMBLY_HPP
#define POINTCLOUDASSEMBLY_MATCHERASSEMBLY_HPP

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
#include <pointmatcher/PointMatcher.h>

namespace CDFF
{
namespace DFN
{
namespace PointCloudAssembly
{
	/**
	 * Fusion of two point clouds using the libpointmatcher fusion algorithm.
	 *
	 * @param samplingSurfaceNormalPrefilter.numberOfNeighbours, number of neighbours for estimation of normals; 
	 * @param samplingSurfaceNormalPrefilter.approximationEpsilon, approximation used in nearest neighbor search; 
	 * @param samplingSurfaceNormalPrefilter.keepNormals, whether the normals should remain as descriptors in the output cloud; 
	 * @param samplingSurfaceNormalPrefilter.keepDensities, whether the densities should remain as descriptors in the output cloud;
	 * @param samplingSurfaceNormalPrefilter.keepEigenValues, whether the eigenvalues should remain as descriptors in the output cloud;
	 * @param samplingSurfaceNormalPrefilter.keepEigenVectors, whether the eigenvectors should remain as descriptors in the output cloud;
         *
	 * @param maximumSamplingDensity, maximum number of points per cubic meter;
	 * @param incrementalMode, when this is false the output is given by the fusion of the two input clouds, when this is true only the first cloud will be used and will be fused with
	 * the an incrementally constructed stored cloud;
	 * @param incrementalMode, whether to use the distance filter or not for the point cloud output.
	 */
	class MatcherAssembly : public PointCloudAssemblyInterface
	{
		public:

			MatcherAssembly();
			virtual ~MatcherAssembly();

			virtual void configure() override;
			virtual void process() override;

		private:

			typedef std::shared_ptr<PointMatcher<float>::DataPointsFilter> SharedDataPointsFilter;
	
			//DFN Parameters
			struct SurfaceNormalPrefilterOptionsSet
				{
				int numberOfNeighbours;
				float approximationEpsilon;
				bool keepNormals;
				bool keepDensities;
				bool keepEigenValues;
				bool keepEigenVectors;
				};

			struct MatcherAssemblyOptionsSet
				{
				SurfaceNormalPrefilterOptionsSet samplingSurfaceNormalPrefilter;
				float maximumSamplingDensity;
				bool useIncrementalMode;
				bool useDistanceFilter;
				};

			Helpers::ParametersListHelper parametersHelper;
			MatcherAssemblyOptionsSet parameters;
			static const MatcherAssemblyOptionsSet DEFAULT_PARAMETERS;

			//PointMatcher library required variables
			SharedDataPointsFilter samplingSurfaceNormalPrefilter;
			SharedDataPointsFilter maxDensityFilter;
			PointMatcher<float>::DataPoints assembledCloud;

			//Core computation Methods.
			void AssemblePointCloud(const PointMatcher<float>::DataPoints& cloud);
			void AssemblePointCloud(const PointMatcher<float>::DataPoints& cloud1, const PointMatcher<float>::DataPoints& cloud2);

			//Output conversion methods
			void PrepareOutAssembledPointCloud();

			//Input conversion methods
			PointMatcher<float>::DataPoints ConvertToDataPoints(const PointCloudWrapper::PointCloud& cloud);

			//Input validation methods
			void ValidateParameters();
	};
}
}
}

#endif // POINTCLOUDASSEMBLY_MATCHERASSEMBLY_HPP

/** @} */
