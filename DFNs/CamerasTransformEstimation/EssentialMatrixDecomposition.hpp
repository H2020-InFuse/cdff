/* --------------------------------------------------------------------------
*
* (C) Copyright …
*
* --------------------------------------------------------------------------
*/

/*!
 * @file EssentialMatrixDecomposition.hpp
 * @date 31/01/2018
 * @author Alessandro Bianco
 */

/*!
 * @addtogroup DFNs
 * 
 *  @brief This DFN computes the camera transform by decomposition of the essential matrix.
 *  
 * The DFN takes a fundamental matrix as input and a set of 2d features matches from one image to another. It operates according to the following steps:
 * (i) computation of the essential matrix from the fundamental matrix and the camera parameters; (ii) decomposition of the fundamental matrix according to OpenCV algorithm described in 
 * "Multiple view geometry in computer vision" by Richard Hartley, and Andrew Zisserman; (iii) as the second step yelds 4 candidates transforms, the 2d features matches are used to rule out three
 * of them and discover the valid one. If more tha one transform is considered valid, the decomposition fails and no transform is given as a result.
 * 
 * This DFN implementation requires the following parameters:
 * @param firstCameraMatrix, this represents the internal parameters of the first (or left) camera: focal lengths and principle points.
 * @param secondCameraMatrix, this represents the internal parameters of the second (or right) camera: focal lengths and principle points.
 * @param numberOfTestPoints, this represents the number of points that need to be tested from the 2d features matches, the half of this number represents the minimum number of points whose 2d matching
 *				 agrees with a transform in order to consider that transform as valid. 
 *
 * @{
 */

#ifndef ESSENTIAL_MATRIX_DECOMPOSITION_HPP
#define ESSENTIAL_MATRIX_DECOMPOSITION_HPP

/* --------------------------------------------------------------------------
 *
 * Includes
 *
 * --------------------------------------------------------------------------
 */
#include <CamerasTransformEstimation/CamerasTransformEstimationInterface.hpp>
#include <CorrespondenceMap2D.hpp>
#include <Pose.hpp>
#include <opencv2/core/core.hpp>
#include <opencv2/imgproc/imgproc.hpp>
#include <yaml-cpp/yaml.h>
#include <Helpers/ParametersListHelper.hpp>


namespace dfn_ci {

/* --------------------------------------------------------------------------
 *
 * Class definition
 *
 * --------------------------------------------------------------------------
 */
    class EssentialMatrixDecomposition : public CamerasTransformEstimationInterface
    {
	/* --------------------------------------------------------------------
	 * Public
	 * --------------------------------------------------------------------
	 */
        public:
            EssentialMatrixDecomposition();
            ~EssentialMatrixDecomposition();
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
		Helpers::ParametersListHelper parametersHelper;

		struct CameraMatrix
			{
			double focalLengthX;
			double focalLengthY;
			cv::Point2d principlePoint;
			};

		struct EssentialMatrixDecompositionOptionsSet
			{
			int numberOfTestPoints;
			CameraMatrix firstCameraMatrix;
			CameraMatrix secondCameraMatrix;
			};

		cv::Mat firstCameraMatrix;
		cv::Mat secondCameraMatrix;

		EssentialMatrixDecompositionOptionsSet parameters;
		static const EssentialMatrixDecompositionOptionsSet DEFAULT_PARAMETERS;
		cv::Mat ConvertToMat(CameraMatrix cameraMatrix);
		cv::Mat ConvertToMat(MatrixWrapper::Matrix3dConstPtr matrix);
		cv::Mat Convert(CorrespondenceMap2DWrapper::CorrespondenceMap2DConstPtr correspondenceMap);

		std::vector<cv::Mat> ComputeTransformMatrix(cv::Mat fundamentalMatrix);
		int FindValidTransform(std::vector<cv::Mat> transformsList, cv::Mat correspondenceMap);
		bool ProjectionMatrixIsValidForTestPoints(cv::Mat projectionMatrix, cv::Mat correspondenceMap);

		void ValidateParameters();
		void ValidateInputs(cv::Mat fundamentalMatrix, cv::Mat CorrespondenceMap);
    };
}
#endif
/* EssentialMatrixDecomposition.hpp */
/** @} */
