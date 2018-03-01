/* --------------------------------------------------------------------------
*
* (C) Copyright …
*
* --------------------------------------------------------------------------
*/

/*!
 * @file OpencvVisualizer.hpp
 * @date 28/02/2018
 * @author Alessandro Bianco
 */

/*!
 * @addtogroup Visualizers
 * 
 *  The OpencvVisualizer contains method for the visualization of images and other opencv data structures
 * 
 * @{
 */

#ifndef OPENCV_VISUALIZER_HPP
#define OPENCV_VISUALIZER_HPP

/* --------------------------------------------------------------------------
 *
 * Includes
 *
 * --------------------------------------------------------------------------
 */
#include "OpencvVisualizer.hpp"
#include <opencv2/highgui/highgui.hpp>
#include <opencv2/imgproc/imgproc.hpp>
#include <opencv2/core/core.hpp>
#include "opencv2/features2d/features2d.hpp"
#include <stdlib.h>
#include <string>

#include <Frame.hpp>
#include <VisualPointFeatureVector2D.hpp>
#include <CorrespondenceMap2D.hpp>
#include <Pose.hpp>
#include <Matrix.hpp>

namespace Visualizers
{
/* --------------------------------------------------------------------------
 *
 * Class definition
 *
 * --------------------------------------------------------------------------
 */
class OpencvVisualizer
	{
	/* --------------------------------------------------------------------
	 * Public
	 * --------------------------------------------------------------------
	 */
	public:
		static void ShowImage(cv::Mat image);
		static void ShowImage(FrameWrapper::FrameConstPtr frame);
		static void ShowVisualFeatures(FrameWrapper::FrameConstPtr frame, VisualPointFeatureVector2DWrapper::VisualPointFeatureVector2DConstPtr featuresVector);
		static void ShowCorrespondences(FrameWrapper::FrameConstPtr frame1, FrameWrapper::FrameConstPtr frame2, CorrespondenceMap2DWrapper::CorrespondenceMap2DConstPtr correspondenceMap);

		static void ShowMatrix(MatrixWrapper::Matrix3dConstPtr matrix);
		static void ShowPose(PoseWrapper::Pose3DConstPtr pose);

		static void Enable();
		static void Disable();
	/* --------------------------------------------------------------------
	 * Protected
	 * --------------------------------------------------------------------
	 */
	protected:
		OpencvVisualizer();

	/* --------------------------------------------------------------------
	 * Private
	 * --------------------------------------------------------------------
	 */
	private:
		static const std::string WINDOW_NAME;
		static bool enabled;
	};
}

/* --------------------------------------------------------------------------
 *
 * Macros definition
 *
 * --------------------------------------------------------------------------
 */
	#ifndef TESTING
		#define DEBUG_SHOW_IMAGE(image)
		#define DEBUG_SHOW_2D_VISUAL_FEATURES(frame, featuresVector)
		#define DEBUG_SHOW_2D_CORRESPONDENCES(frame1, frame2, correspondenceMap)
		#define DEBUG_SHOW_POSE(pose)
		#define DEBUG_SHOW_MATRIX(matrix)
	#else
		#define DEBUG_SHOW_IMAGE(image) Visualizers::OpencvVisualizer::ShowImage(image)
		#define DEBUG_SHOW_2D_VISUAL_FEATURES(frame, featuresVector) Visualizers::OpencvVisualizer::ShowVisualFeatures(frame, featuresVector)
		#define DEBUG_SHOW_2D_CORRESPONDENCES(frame1, frame2, correspondenceMap) Visualizers::OpencvVisualizer::ShowCorrespondences(frame1, frame2, correspondenceMap)
		#define DEBUG_SHOW_POSE(pose) Visualizers::OpencvVisualizer::ShowPose(pose)
		#define DEBUG_SHOW_MATRIX(matrix) Visualizers::OpencvVisualizer::ShowMatrix(matrix)
	#endif

#endif
/* OpencvVisualizer.hpp */
/** @} */

