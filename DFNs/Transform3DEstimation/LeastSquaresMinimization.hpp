/**
 * @author Alessandro Bianco
 */

/**
 * @addtogroup DFNs
 * @{
 */

#ifndef TRANSFORM3DESTIMATION_LEASTSQUARESMINIMIZATION_HPP
#define TRANSFORM3DESTIMATION_LEASTSQUARESMINIMIZATION_HPP

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

namespace CDFF
{
namespace DFN
{
namespace Transform3DEstimation
{
	/**
	 * Estimation of the geometric transformation between matches of 3d points,
	 * by least squares minimization.
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
	class LeastSquaresMinimization : public Transform3DEstimationInterface
	{
		public:

			LeastSquaresMinimization();
			virtual ~LeastSquaresMinimization();

			virtual void configure();
			virtual void process();

		private:

			Helpers::ParametersListHelper parametersHelper;

			struct LeastSquaresMinimizationOptionsSet
			{
				float maximumAllowedError;
			};

			PoseWrapper::Pose3D emptyPose;
			LeastSquaresMinimizationOptionsSet parameters;
			static const LeastSquaresMinimizationOptionsSet DEFAULT_PARAMETERS;

			bool CreateLinearSystem(const CorrespondenceMap3DWrapper::CorrespondenceMap3D& map, cv::Mat& coefficientMatrix, cv::Mat& valueMatrix);
			cv::Mat SolveLinearSystem(cv::Mat coefficientMatrix, cv::Mat valueMatrix, float& error);
			void AddTransformOutput(cv::Mat transformMatrix);

			void ValidateParameters();
			void ValidateInputs(const CorrespondenceMap3DWrapper::CorrespondenceMap3D& map);
	};
}
}
}

#endif // TRANSFORM3DESTIMATION_LEASTSQUARESMINIMIZATION_HPP

/** @} */
