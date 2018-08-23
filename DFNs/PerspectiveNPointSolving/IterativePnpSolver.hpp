/**
 * @author Alessandro Bianco
 */

/**
 * @addtogroup DFNs
 * @{
 */

#ifndef PERSPECTIVENPOINTSOLVING_ITERATIVEPNPSOLVER_HPP
#define PERSPECTIVENPOINTSOLVING_ITERATIVEPNPSOLVER_HPP

#include "PerspectiveNPointSolvingInterface.hpp"

#include <PointCloud.hpp>
#include <VisualPointFeatureVector2DToMatConverter.hpp>
#include <MatToTransform3DConverter.hpp>
#include <Helpers/ParametersListHelper.hpp>

#include <opencv2/core/core.hpp>

namespace CDFF
{
namespace DFN
{
namespace PerspectiveNPointSolving
{
	/**
	 * Iterative Perspective-n-Point algorithm (provided by OpenCV).
	 *
	 * @param cameraMatrix
	 *        intrinsic camera parameters (focal length and principal points)
	 */
	class IterativePnpSolver : public PerspectiveNPointSolvingInterface
	{
		public:

			IterativePnpSolver();
			virtual ~IterativePnpSolver();

			virtual void configure();
			virtual void process();

		private:

			struct CameraMatrix
			{
				float focalLengthX;
				float focalLengthY;
				float principalPointX;
				float principalPointY;
			};

			struct IterativePnpOptionsSet
			{
				CameraMatrix cameraMatrix;
			};

			Helpers::ParametersListHelper parametersHelper;
			IterativePnpOptionsSet parameters;
			static const IterativePnpOptionsSet DEFAULT_PARAMETERS;

			cv::Mat Convert(PointCloudWrapper::PointCloudConstPtr points);
			Converters::VisualPointFeatureVector2DToMatConverter visualPointFeatureVector2DToMat;
			Converters::MatToPose3DConverter matToPose3D;

			cv::Mat ComputePose(cv::Mat points, cv::Mat projections, bool& success);

			void ValidateParameters();
			void ValidateInputs(cv::Mat points, cv::Mat projections);
	};
}
}
}

#endif // PERSPECTIVENPOINTSOLVING_ITERATIVEPNPSOLVER_HPP

/** @} */
