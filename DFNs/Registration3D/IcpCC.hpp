/**
 * @author Alessandro Bianco
 */

/**
 * @addtogroup DFNs
 * @{
 */

#ifndef ICP3D_HPP
#define ICP3D_HPP

#include "Registration3DInterface.hpp"

#include <PointCloud.hpp>
#include <Pose.hpp>
#include <PointCloudToPclPointCloudConverter.hpp>
#include <EigenTransformToTransform3DConverter.hpp>
#include <Helpers/ParametersListHelper.hpp>

#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl/keypoints/harris_3d.h>
#include <yaml-cpp/yaml.h>
#include "RegistrationTools.h"
#include "ChunkedPointCloud.h"

#include <stdlib.h>
#include <string>

namespace dfn_ci
{
	/**
	 * Registration of Point clouds using the ICP algorithm
	 * 
	 * @param convergenceType, whether the converge is based on error reduction or on number of iterations
	 * @param minimumErrorReduction, the upper error threshold above which the algorithm continues if converge type is based on error reduction; 
	 * @param maximumNumberOfIterations, the number of iteratio the algorithm will run if the converge type is based on the number of iterations;
	 * @param scaleIsAdjustable, whether to release the scale parameter during the registration procedure or not;
	 * @param farthestPointsAreFilteredOut, If true, the algorithm will automatically ignore farthest points from the reference, for better convergence;
	 * @param samplingLimit, Maximum number of points per cloud (they are randomly resampled below this limit otherwise);
	 * @param finalOverlapRatio, Theoretical overlap ratio (at each iteration, only this percentage (between 0 and 1) will be used for registration;
	 * @param maximumNumberOfThreads, Maximum number of threads to use (0 = max);
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

		Converters::PointCloudToPclPointCloudConverter pointCloudToPclPointCloudConverter;
		Converters::EigenTransformToTransform3DConverter eigenTransformToTransform3dConverter;

		void ComputeTransform(CCLib::ChunkedPointCloud* sourceCloud, CCLib::ChunkedPointCloud* sinkCloud);
		CCLib::ChunkedPointCloud* Convert(PointCloudWrapper::PointCloudConstPtr cloud);
		void ConvertParametersToCCParametersList();
		CCLib::RegistrationTools::ScaledTransformation ConvertTrasformToCCTransform(const PoseWrapper::Transform3D& transform);
		PoseWrapper::Transform3D ConvertCCTransformToTranform(const CCLib::RegistrationTools::ScaledTransformation& ccTransform);

		void ValidateParameters();
		void ValidateInputs(CCLib::ChunkedPointCloud* sourceCloud, CCLib::ChunkedPointCloud* sinkCloud);
		void ValidateCloud(CCLib::ChunkedPointCloud* cloud);
	};
}

#endif // ICP3D_HPP

/** @} */
