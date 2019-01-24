/**
 * @author Alessandro Bianco
 */

/**
 * @addtogroup DFNs
 * @{
 */

#ifndef FEATURESMATCHING3D_BESTDESCRIPTORMATCH_HPP
#define FEATURESMATCHING3D_BESTDESCRIPTORMATCH_HPP

#include "FeaturesMatching3DInterface.hpp"

#include <Types/CPP/VisualPointFeatureVector3D.hpp>
#include <Converters/VisualPointFeatureVector3DToPclPointCloudConverter.hpp>
#include <Types/CPP/Pose.hpp>
#include <Converters/SupportTypes.hpp>
#include <Helpers/ParametersListHelper.hpp>

namespace CDFF
{
namespace DFN
{
namespace FeaturesMatching3D
{
	/**
	 * Brute force algorithm for matching points. The best matches are returned as long
	 * as their distance is below a define threshold. A point appears at most once in one match.
	 *
	 * @param maxCorrespondenceDistance
	 *        largest distance allowed between the transformed point from the
	 *        model pointcloud and a point from the scene pointcloud before
	 *        they are no longer considered an acceptable match
	 */
	class BestDescriptorMatch : public FeaturesMatching3DInterface
	{
		public:

			BestDescriptorMatch();
			virtual ~BestDescriptorMatch();

			virtual void configure() override;
			virtual void process() override;

		private:

			//DFN Parameters
			struct BestDescriptorMatchOptionsSet
			{
				double maxCorrespondenceDistance;
			};

			Helpers::ParametersListHelper parametersHelper;
			BestDescriptorMatchOptionsSet parameters;
			static const BestDescriptorMatchOptionsSet DEFAULT_PARAMETERS;

			//External Converters
			Converters::VisualPointFeatureVector3DToPclPointCloudConverter
				visualPointFeatureVector3DToPclPointCloud;

			//Type Conversion Methods
			PoseWrapper::Pose3DConstPtr Convert(Eigen::Matrix4f eigenTransform);

			//Core Computation Methods
			void ComputeBestMatches(pcl::PointCloud<pcl::PointXYZ>::Ptr bestMatchSourceCloud, pcl::PointCloud<pcl::PointXYZ>::Ptr bestMatchSinkCloud);
				
			PoseWrapper::Pose3DConstPtr ComputeTransform(pcl::PointCloud<pcl::PointXYZ>::Ptr sourceCloud, pcl::PointCloud<pcl::PointXYZ>::Ptr sinkCloud);

			float ComputeDistance(int sourceIndex, int sinkIndex);

			//Input Validation Methods
			void ValidateParameters();
			void ValidateInputs();
			void ValidateCloud(const VisualPointFeatureVector3DWrapper::VisualPointFeatureVector3D& features);
	};
}
}
}

#endif // FEATURESMATCHING3D_BESTDESCRIPTORMATCH_HPP

/** @} */
