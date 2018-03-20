/* --------------------------------------------------------------------------
*
* (C) Copyright …
*
* --------------------------------------------------------------------------
*/

/*!
 * @file IterativePnpSolver.hpp
 * @date 20/02/2018
 * @author Alessandro Bianco
 */

/*!
 * @addtogroup DFNs
 * 
 * @brief This DFN executes the iterative Perspective N Point algorithm implemented in openCV.
 *  
 *
 * This DFN implementation uses the following parameters:
 * @param cameraMatrix, the intrinsic parameter matrix of the camera that took the image, in terms of focal length and principle point coordinates.
 *
 * @{
 */

#ifndef ITERATIVE_PNP_SOLVER_HPP
#define ITERATIVE_PNP_SOLVER_HPP

/* --------------------------------------------------------------------------
 *
 * Includes
 *
 * --------------------------------------------------------------------------
 */
#include <PerspectiveNPointSolving/PerspectiveNPointSolvingInterface.hpp>
#include <PointCloud.hpp>
#include <Pose.hpp>
#include <VisualPointFeatureVector2D.hpp>
#include <opencv2/core/core.hpp>
#include <opencv2/imgproc/imgproc.hpp>
#include <yaml-cpp/yaml.h>
#include "opencv2/features2d/features2d.hpp"
#include <Helpers/ParametersListHelper.hpp>


namespace dfn_ci {

/* --------------------------------------------------------------------------
 *
 * Class definition
 *
 * --------------------------------------------------------------------------
 */
    class IterativePnpSolver : public PerspectiveNPointSolvingInterface
    {
	/* --------------------------------------------------------------------
	 * Public
	 * --------------------------------------------------------------------
	 */
        public:
            IterativePnpSolver();
            ~IterativePnpSolver();
            void process();
            void configure();

	/* --------------------------------------------------------------------
	 * Protected
	 * --------------------------------------------------------------------
	 */
        protected:

	/* --------------------------------------------------------------------
	 * Private
	 * --------------------------------------------------------------------
	 */	
	private:

		struct CameraMatrix
			{
			float focalLengthX;
			float focalLengthY;
			float principlePointX;
			float principlePointY;
			};

		struct IterativePnpOptionsSet
			{
			CameraMatrix cameraMatrix;
			};

		Helpers::ParametersListHelper parametersHelper;
		IterativePnpOptionsSet parameters;
		static const IterativePnpOptionsSet DEFAULT_PARAMETERS;

		cv::Mat ComputePose(cv::Mat pointCloud, cv::Mat featuresMatrix, bool& success);
		cv::Mat Convert(PointCloudWrapper::PointCloudConstPtr pointCloud);

		void ValidateParameters();
		void ValidateInputs(cv::Mat sourceFeaturesMatrix, cv::Mat sinkFeaturesMatrix);
    };
}
#endif
/* IterativePnpSolver.hpp */
/** @} */
