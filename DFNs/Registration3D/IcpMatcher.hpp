/**
 * @author Alessandro Bianco
 */

/**
 * @addtogroup DFNs
 * @{
 */

#ifndef REGISTRATION3D_ICPMATCHER_HPP
#define REGISTRATION3D_ICPMATCHER_HPP

#include "Registration3DInterface.hpp"

#include <Pose.hpp>
#include <PointCloudToPclPointCloudConverter.hpp>
#include <EigenTransformToTransform3DConverter.hpp>
#include <Transform3DToEigenTransformConverter.hpp>
#include <Helpers/ParametersListHelper.hpp>

#include <pcl/point_cloud.h>
#include <pcl/point_types.h>

#include <pointmatcher/PointMatcher.h>

namespace CDFF
{
namespace DFN
{
namespace Registration3D
{
	/**
	 * Registration of pointclouds using the ICP algorithm as implemented by ETHZ available at https://github.com/ethz-asl/libpointmatcher/blob/master/doc/Datafilters.md#surfacenormalhead
	 *
	 * @param useDefault, whether to use the defaut configuration of the library. If this is set to true, other parameters will not be considered;
	 * @param fixRotationByNormalization, whether the rotation matrix is fixed by normalization (on true) or by changing just one element by the smallest amount possible (on false);
	 *
	 * @param distanceLimitPrefilter.use.useOnSource, whether to use the distance limit prefilter on the source cloud;
	 * @param distanceLimitPrefilter.use.stepOnSource, the step in the source prefilter pipeline the distance limit prefilter should be used at; 
	 * @param distanceLimitPrefilter.use.useOnSink, whether to use the distance limit prefilter on the sink cloud;
	 * @param distanceLimitPrefilter.use.stepOnSink, the step in the sink prefilter pipeline the distance limit prefilter should be used at;
	 * @param distanceLimitPrefilter.thresholdDimension, dimension along which we apply the filter: {Radial, AxisX, AxisY, AxisZ}; 
	 * @param distanceLimitPrefilter.distanceThreshold, threshold of the filter; 
	 * @param distanceLimitPrefilter.removePointsWithinThreshold, whether points within the treshold are removed (on true) or the points outside the treshold are removed (on false);
	 *
	 * @param randomSamplingPrefilter.use.useOnSource, whether to use the random sampling prefilter on the source cloud;
	 * @param randomSamplingPrefilter.use.stepOnSource, the step in the source prefilter pipeline the random sampling prefilter should be used at; 
	 * @param randomSamplingPrefilter.use.useOnSink, whether to use the random sampling prefilter on the sink cloud;
	 * @param randomSamplingPrefilter.use.stepOnSink, the step in the sink prefilter pipeline the random sampling prefilter should be used at;
	 * @param randomSamplingPrefilter.probability, probability of keeping a point;
	 *
	 * @param samplingSurfaceNormalPrefilter.use.useOnSource, whether to use the sampling surface normal prefilter on the source cloud;
	 * @param samplingSurfaceNormalPrefilter.use.stepOnSource, the step in the source prefilter pipeline the sampling surface normal prefilter should be used at; 
	 * @param samplingSurfaceNormalPrefilter.use.useOnSink, whether to use the sampling surface normal prefilter on the sink cloud;
	 * @param samplingSurfaceNormalPrefilter.use.stepOnSink, the step in the sink prefilter pipeline the sampling surface normal prefilter should be used at;
	 * @param samplingSurfaceNormalPrefilter.numberOfNeighbours, number of neighbours for estimation of normals; 
	 * @param samplingSurfaceNormalPrefilter.keepNormals, whether the normals should remain as descriptors in the output cloud; 
	 * @param samplingSurfaceNormalPrefilter.keepDensities, whether the densities should remain as descriptors in the output cloud;
	 * @param samplingSurfaceNormalPrefilter.keepEigenValues, whether the eigenvalues should remain as descriptors in the output cloud;
	 * @param samplingSurfaceNormalPrefilter.keepEigenVectors, whether the eigenvectors should remain as descriptors in the output cloud;
	 *
	 * @param trimmedDistancePostfilter.use,  whether to use the trimmed distance postfilter on the output cloud;
	 * @param trimmedDistancePostfilter.step, the step in the postfilter pipeline the  trimmed distance postfilter should be used at;
	 * @param trimmedDistancePostfilter.ratio, the percentage of matches to keep expressed as a number in [0, 1];
	 *
	 * @param kdTreeNumberOfNearestNeighbours, number of nearest neighbours in the kd tree search used for matching;
	 * @param kdTreeMatchingEpsilon, the matching distance used in the kd tree search;
	 * @param minimizerType, the type of error minimized during ICP, one of {Identity, PointToPoint, PointToPlane};
	 * 
	 * @param maximumIterations, number of iteration after which ICP refinement stops;
	 * @param maxTranslationDistance, distance threshold, if all matches are within this distance and within the following rotation distance, ICP stops;
	 * @param maxRotationDistance, orientation threshold, if all matches are within this distance and within the previous translation distance, ICP stops;
	 * @param smoothnessLength
	 */
	class IcpMatcher : public Registration3DInterface
	{
		public:

			IcpMatcher();
			virtual ~IcpMatcher();

			virtual void configure();
			virtual void process();

		private:

			enum MinimizerType
				{
				Identity,
				PointToPoint,
				PointToPlane
				};
			class MinimizerTypeHelper : public Helpers::ParameterHelper<MinimizerType, std::string>
				{
				public:
					MinimizerTypeHelper(const std::string& parameterName, MinimizerType& boundVariable, const MinimizerType& defaultValue);
				private:
					MinimizerType Convert(const std::string& value);
				};

			enum ThresholdDimension
				{
				Radial,
				AxisX,
				AxisY,
				AxisZ
				};

			class ThresholdDimensionHelper : public Helpers::ParameterHelper<ThresholdDimension, std::string>
				{
				public:
					ThresholdDimensionHelper(const std::string& parameterName, ThresholdDimension& boundVariable, const ThresholdDimension& defaultValue);
				private:
					ThresholdDimension Convert(const std::string& value);
				};

			struct PrefilterUse
				{
				bool useOnSource;
				int stepOnSource;
				bool useOnSink;
				int stepOnSink;
				};

			struct DistanceLimitPrefilterOptionsSet
				{
				PrefilterUse use;
				ThresholdDimension thresholdDimension;
				float distanceThreshold;
				bool removePointsWithinThreshold;
				};
			struct RandomSamplingPrefilterOptionsSet
				{
				PrefilterUse use;
				float probability;
				};
			struct SamplingSurfaceNormalOptionsSet //Two more parameters are available in libpointmatcher documentation: but there is a run-time error upon seeting, documentation not updated?
				{
				PrefilterUse use;
				int numberOfNeighbours;
				bool keepNormals;
				bool keepDensities;
				bool keepEigenValues;
				bool keepEigenVectors;
				};
			struct TrimmedDistancePostfilterOptionsSet
				{
				bool use;
				int step;
				float ratio;
				};

			struct IcpOptionsSet
			{
				bool useDefault;
				bool fixRotationByNormalization;

				DistanceLimitPrefilterOptionsSet distanceLimitPrefilter;
				RandomSamplingPrefilterOptionsSet randomSamplingPrefilter;
				SamplingSurfaceNormalOptionsSet samplingSurfaceNormalPrefilter;

				TrimmedDistancePostfilterOptionsSet trimmedDistancePostfilter;

				int kdTreeNumberOfNearestNeighbours;
				float kdTreeMatchingEpsilon;

				MinimizerType minimizerType;

				int maximumIterations;
				float maxTranslationDistance;
				float maxRotationDistance;
				int smoothnessLength;
			};

			Helpers::ParametersListHelper parametersHelper;
			IcpOptionsSet parameters;
			static const IcpOptionsSet DEFAULT_PARAMETERS;

			PointMatcher<float>::TransformationParameters lastTransformGuess;
			PointMatcher<float>::ICP icp;

			Converters::PointCloudToPclPointCloudConverter pointCloudToPclPointCloud;
			Converters::EigenTransformToTransform3DConverter eigenTransformToTransform3D;
			Converters::Transform3DToEigenTransformConverter transform3DToEigenTransform;

			PointMatcher<float>::DataPoints ConvertToDataPoints(const PointCloudWrapper::PointCloud& cloud);
			PoseWrapper::Pose3D ConvertToPose3D(PointMatcher<float>::TransformationParameters transform);

			PoseWrapper::Pose3DConstPtr ComputeTransform(PointMatcher<float>::DataPoints sourceCloud, PointMatcher<float>::DataPoints sinkCloud);
			PoseWrapper::Pose3DConstPtr ComputeTransform(PointMatcher<float>::DataPoints sourceCloud, PointMatcher<float>::DataPoints sinkCloud, 
				PointMatcher<float>::TransformationParameters transformGuess);

			void SetupIcpMatcher();
			bool FixTransformationMatrix(PointMatcher<float>::TransformationParameters& transform);
			bool FixSingleRotationElement(PointMatcher<float>::TransformationParameters& transform, float determinant);
			bool NormalizeRotationMatrix(PointMatcher<float>::TransformationParameters& transform, float determinant);

			void ValidateParameters();
			void ValidateInputs(pcl::PointCloud<pcl::PointXYZ>::ConstPtr sourceCloud, pcl::PointCloud<pcl::PointXYZ>::ConstPtr sinkCloud);
			void ValidateCloud(pcl::PointCloud<pcl::PointXYZ>::ConstPtr cloud);
	};
}
}
}

#endif // REGISTRATION3D_ICPMATCHER_HPP

/** @} */

