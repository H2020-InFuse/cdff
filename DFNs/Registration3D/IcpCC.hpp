/**
 * @author Alessandro Bianco
 */

/**
 * @addtogroup DFNs
 * @{
 */

#ifndef ICPCC_HPP
#define ICPCC_HPP

#include "Registration3DInterface.hpp"

#include <Pose.hpp>
#include <PointCloudToPclPointCloudConverter.hpp>
#include <EigenTransformToTransform3DConverter.hpp>
#include <Helpers/ParametersListHelper.hpp>
#include <PointCloud.hpp>

#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <RegistrationTools.h>
#include <ChunkedPointCloud.h>

#include <string>

namespace CDFF
{
namespace DFN
{
namespace Registration3D
{
	/**
	 * Registration of Point clouds using the ICP algorithm and the CC library
	 *
	 * @param convergenceType
	 *        whether convergence is asserted on error reduction or on the
	 *        number of iterations
	 * @param minimumErrorReduction
	 *        when convergence is asserted on error reduction, error threshold
	 *        above which the algorithm keeps running
	 * @param maximumNumberOfIterations
	 *        when convergence is asserted on the number of iterations, number
	 *        of iterations that the algorithm will run
	 *
	 * @param scaleIsAdjustable
	 *        allows or disallows adjustment of the scale parameter during
	 *        registration
	 * @param farthestPointsAreFilteredOut
	 *        ignore points farthest away from the reference, for better
	 *        convergence
	 * @param samplingLimit
	 *        maximum number of points in a cloud: if a cloud has more points,
	 *        it is randomly resampled to be below this limit
	 * @param finalOverlapRatio
	 *        theoretical overlap ratio: at each iteration, only this percentage
	 *        of points (between 0 and 1) is used for registration
	 * @param maximumNumberOfThreads
	 *        maximum number of threads to use (0 = max)
	 */
	class IcpCC : public Registration3DInterface
	{
		public:

			IcpCC();
			virtual ~IcpCC();

			virtual void configure();
			virtual void process();

		private:

			enum ConvergenceType
			{
				MINIMUM_ERROR_REDUCTION,
				NUMBER_OF_ITERATIONS
			};
			class ConvergenceTypeHelper : public Helpers::ParameterHelper<ConvergenceType, std::string>
			{
				public:
					ConvergenceTypeHelper(const std::string& parameterName, ConvergenceType& boundVariable, const ConvergenceType& defaultValue);
				private:
					ConvergenceType Convert(const std::string& value);
			};

			struct IcpOptionsSet
			{
				ConvergenceType convergenceType;
				double minimumErrorReduction;
				int maximumNumberOfIterations;
				bool scaleIsAdjustable;
				bool farthestPointsAreFilteredOut;
				int samplingLimit;
				double finalOverlapRatio;
				int maximumNumberOfThreads;
			};

			Helpers::ParametersListHelper parametersHelper;
			IcpOptionsSet parameters;
			static const IcpOptionsSet DEFAULT_PARAMETERS;
			CCLib::ICPRegistrationTools::Parameters ccParametersList;

			Converters::PointCloudToPclPointCloudConverter pointCloudToPclPointCloud;
			Converters::EigenTransformToTransform3DConverter eigenTransformToTransform3D;

			void ComputeTransform(CCLib::ChunkedPointCloud* sourceCloud, CCLib::ChunkedPointCloud* sinkCloud);
			CCLib::ChunkedPointCloud* Convert(PointCloudWrapper::PointCloudConstPtr cloud);
			void ConvertParametersToCCParametersList();
			CCLib::RegistrationTools::ScaledTransformation ConvertTrasformToCCTransform(const PoseWrapper::Pose3D& transform);
			PoseWrapper::Pose3D ConvertCCTransformToTranform(const CCLib::RegistrationTools::ScaledTransformation& ccTransform);

			void ValidateParameters();
			void ValidateInputs(CCLib::ChunkedPointCloud* sourceCloud, CCLib::ChunkedPointCloud* sinkCloud);
			void ValidateCloud(CCLib::ChunkedPointCloud* cloud);
	};
}
}
}

#endif // ICPCC_HPP

/** @} */
