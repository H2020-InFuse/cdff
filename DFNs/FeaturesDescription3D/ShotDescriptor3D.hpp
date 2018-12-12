/**
 * @author Alessandro Bianco
 */

/**
 * @addtogroup DFNs
 * @{
 */

#ifndef FEATURESDESCRIPTION3D_SHOTDESCRIPTOR3D_HPP
#define FEATURESDESCRIPTION3D_SHOTDESCRIPTOR3D_HPP

#include "FeaturesDescription3DInterface.hpp"

#include <Types/CPP/PointCloud.hpp>
#include <Types/CPP/VisualPointFeatureVector3D.hpp>
#include <Converters/PointCloudToPclPointCloudConverter.hpp>
#include <Converters/PointCloudToPclNormalsCloudConverter.hpp>
#include <Helpers/ParametersListHelper.hpp>

#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl/features/shot.h>
#include <boost/make_shared.hpp>
#include <yaml-cpp/yaml.h>

#include <stdlib.h>
#include <string>

namespace CDFF
{
namespace DFN
{
namespace FeaturesDescription3D
{
	/**
	 * Computation of descriptors for input 3D keypoints using SHOT (provided
	 * by PCL).
	 *
	 * @param localReferenceFrameEstimationRadius
	 * @param searchRadius
	 *        neighbor points within this distance of the keypoint are used
	 *        in the computation of its descriptor
	 *
	 * @param forceNormalsEstimation
	 *        compute surface normals regardless of whether estimates are
	 *        provided as input
	 * @param enableNormalsEstimation, this
	 *        determines whether the absence of input estimates of the surface
	 *        normals causes the class to throw an error or compute estimates
	 *
	 * @param outputFormat
	 *        Format of the returned pointcloud of keypoints:
	 *        * Positions:
	 *          The pointcloud contains the 3D coordinates of the keypoints
	 *        * References:
	 *          The pointcloud contains the indices of the 3D points,
	 *          as they appear in the input pointcloud
	 *
	 * @param normalEstimationOptions.searchRadius
	 *        neighbor points within this distance of a point are used
	 *        in the computation of the surface normal at the point
	 * @param normalEstimationOptions.neighboursSetSize
	 *        how many closest neighbors to a point are used
	 *        in the computation of the surface normal at the point
	 */
	class ShotDescriptor3D : public FeaturesDescription3DInterface
	{
		public:

			ShotDescriptor3D();
			virtual ~ShotDescriptor3D();

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

			struct BaseOptionsSet
			{
				float localReferenceFrameEstimationRadius;
				double searchRadius;
				OutputFormat outputFormat;
				bool enableNormalsEstimation;
				bool forceNormalsEstimation;
			};

			struct NormalEstimationOptionsSet
			{
				double searchRadius;
				int neighboursSetSize;
			};

			struct ShotOptionsSet
			{
				BaseOptionsSet baseOptions;
				NormalEstimationOptionsSet normalEstimationOptions;
			};

			Helpers::ParametersListHelper parametersHelper;
			ShotOptionsSet parameters;
			static const ShotOptionsSet DEFAULT_PARAMETERS;

			pcl::PointCloud<pcl::SHOT352>::ConstPtr ComputeShotDescriptors(
				pcl::PointCloud<pcl::PointXYZ>::ConstPtr pointCloud,
				pcl::IndicesConstPtr indicesList,
				pcl::PointCloud<pcl::Normal>::ConstPtr optionalNormalsCloud
			);
			pcl::PointCloud<pcl::Normal>::ConstPtr EstimateNormals(pcl::PointCloud<pcl::PointXYZ>::ConstPtr pointCloud, pcl::search::KdTree<pcl::PointXYZ>::Ptr kdTree);
			pcl::IndicesConstPtr Convert(const VisualPointFeatureVector3DWrapper::VisualPointFeatureVector3DConstPtr featuresVector);
			VisualPointFeatureVector3DWrapper::VisualPointFeatureVector3DConstPtr Convert(
				const pcl::PointCloud<pcl::PointXYZ>::ConstPtr inputCloud,
				const pcl::IndicesConstPtr indicesList,
				const pcl::PointCloud<pcl::SHOT352>::ConstPtr shotPointCloud
			);
			Converters::PointCloudToPclPointCloudConverter pointCloudToPclPointCloud;
			Converters::PointCloudToPclNormalsCloudConverter pointCloudToPclNormalsCloud;

			void ValidateParameters();
			void ValidateMandatoryInputs(pcl::PointCloud<pcl::PointXYZ>::ConstPtr pointCloud, pcl::IndicesConstPtr indicesList);
			bool IsNormalsCloudValid(pcl::PointCloud<pcl::PointXYZ>::ConstPtr pointCloud, pcl::PointCloud<pcl::Normal>::ConstPtr normalsCloud);
	};
}
}
}

#endif // FEATURESDESCRIPTION3D_SHOTDESCRIPTOR3D_HPP

/** @} */
