/**
 * @author Alessandro Bianco
 */

/**
 * @addtogroup DFNs
 * @{
 */

#ifndef TRANSFORM3DESTIMATION_CERESESTIMATION_HPP
#define TRANSFORM3DESTIMATION_CERESESTIMATION_HPP

#include "Transform3DEstimationInterface.hpp"

#include <Types/CPP/CorrespondenceMap3D.hpp>
#include <Types/CPP/CorrespondenceMaps3DSequence.hpp>
#include <Types/CPP/PosesSequence.hpp>
#include <Types/CPP/Pose.hpp>
#include <Types/CPP/Matrix.hpp>
#include <Helpers/ParametersListHelper.hpp>

#include <opencv2/core/core.hpp>
#include <opencv2/imgproc/imgproc.hpp>
#include <yaml-cpp/yaml.h>
#include <ceres/ceres.h>

namespace CDFF
{
namespace DFN
{
namespace Transform3DEstimation
{
	/**
	 * Estimation of the geometric transformation between matches of 3d points,
	 * with Ceres Library.
	 *
	 * Processing steps, repeated for each correspondence map :
	 *
	 * (i)   Definition of the linear system for computation of the 3d transform;
	 * (ii)  Resolution of the linear system in the least squares sense
	 * (iii) Computation of the pose from the output of the linear system.
	 *
	 * @param maximumAllowedError
	 *        this is the maximum error allowed (root of the squared error), if a
	 *	  a transform estimation exceeds this error, no transform output is provided.
	 */
	class CeresEstimation : public Transform3DEstimationInterface
	{
		public:

			CeresEstimation();
			virtual ~CeresEstimation();

			virtual void configure() override;
			virtual void process() override;

		private:

			Helpers::ParametersListHelper parametersHelper;

			struct CeresEstimationOptionsSet
			{
				float maximumAllowedError;
				float maximumAllowedDeterminantError;
			};

			struct Transform3DCostFunctor
			{
				Transform3DCostFunctor(BaseTypesWrapper::Point3D source, BaseTypesWrapper::Point3D sink);
				template <typename T>
				bool operator()(const T* const cameraTransform, T* residual) const;
				template <typename T>
				bool operator()(const T* const firstCameraTransform, const T* const secondCameraTransform, T* residual) const;

				template <typename T>
				void TransformPoint(const T* const cameraTransform, const T* const originalPoint, T* transformedPoint) const;

				static ceres::CostFunction* Create(BaseTypesWrapper::Point3D source, BaseTypesWrapper::Point3D sink, int transformChainLength);
				BaseTypesWrapper::Point3D source;
				BaseTypesWrapper::Point3D sink;
			};
			typedef double Transform3d[12];

			CeresEstimationOptionsSet parameters;
			static const CeresEstimationOptionsSet DEFAULT_PARAMETERS;

			int ComputeNumberOfCameras(int numberOfCorrespondenceMaps);
			void InitializeTransforms(std::vector<Transform3d>& transformList);
			float SolveEstimation(const CorrespondenceMap3DWrapper::CorrespondenceMaps3DSequence& sequence, int numberOfCameras, std::vector<Transform3d>& transformList);

			bool SetOutputPoses(const std::vector<Transform3d>& transformList);
			bool CreateLinearSystem(const CorrespondenceMap3DWrapper::CorrespondenceMap3D& map, cv::Mat& coefficientMatrix, cv::Mat& valueMatrix);
			cv::Mat SolveLinearSystem(cv::Mat coefficientMatrix, cv::Mat valueMatrix, float& error);

			void ValidateParameters();
			void ValidateInputs(const CorrespondenceMap3DWrapper::CorrespondenceMap3D& map);
	};
}
}
}

#endif // TRANSFORM3DESTIMATION_CERESESTIMATION_HPP

/** @} */
