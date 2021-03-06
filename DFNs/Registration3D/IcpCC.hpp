/**
 * @author Alessandro Bianco
 */

/**
 * @addtogroup DFNs
 * @{
 */

#ifndef REGISTRATION3D_ICPCC_HPP
#define REGISTRATION3D_ICPCC_HPP

#include "Registration3DInterface.hpp"

#include <Types/CPP/Pose.hpp>
#include <Converters/PointCloudToPclPointCloudConverter.hpp>
#include <Converters/EigenTransformToTransform3DConverter.hpp>
#include <Helpers/ParametersListHelper.hpp>
#include <Types/CPP/PointCloud.hpp>

#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <cloudcompare-core/RegistrationTools.h>
#include <cloudcompare-core/PointCloud.h>

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

			virtual void configure() override;
			virtual void process() override;

		private:

			//DFN Parameters
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
					ConvergenceType Convert(const std::string& value) override;
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

			//Parameters Conversion
			CCLib::ICPRegistrationTools::Parameters ccParametersList;
			void ConvertParametersToCCParametersList();

			//External conversion helpers
			Converters::PointCloudToPclPointCloudConverter pointCloudToPclPointCloud;
			Converters::EigenTransformToTransform3DConverter eigenTransformToTransform3D;

			//Type conversion methods
			CCLib::PointCloud* Convert(PointCloudWrapper::PointCloudConstPtr cloud);
			CCLib::RegistrationTools::ScaledTransformation ConvertTrasformToCCTransform(const PoseWrapper::Pose3D& transform);
			PoseWrapper::Pose3D ConvertCCTransformToTranform(const CCLib::RegistrationTools::ScaledTransformation& ccTransform);

			//Core computation methods
			void ComputeTransform(CCLib::PointCloud* sourceCloud, CCLib::PointCloud* sinkCloud);

			//Input Validation methods
			void ValidateParameters();
			void ValidateInputs(CCLib::PointCloud* sourceCloud, CCLib::PointCloud* sinkCloud);
			void ValidateCloud(CCLib::PointCloud* cloud);
	};
}
}
}

#endif // REGISTRATION3D_ICPCC_HPP

/** @} */
