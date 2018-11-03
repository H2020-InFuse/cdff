/* --------------------------------------------------------------------------
*
* (C) Copyright …
*
* --------------------------------------------------------------------------
*/

/*!
 * @file CameraPair.hpp
 * @date 06/03/2018
 * @author Alessandro Bianco
 */

/*!
 * @addtogroup DataGenerators
 * 
 *  This class represents a camera pair.
 *  
 *
 * @{
 */

#ifndef CAMERA_PAIR_HPP
#define CAMERA_PAIR_HPP

/* --------------------------------------------------------------------------
 *
 * Includes
 *
 * --------------------------------------------------------------------------
 */
#include <opencv2/core/core.hpp>
#include <opencv2/imgproc/imgproc.hpp>
#include <Types/CPP/CorrespondenceMap2D.hpp>
#include <Types/CPP/Pose.hpp>


namespace DataGenerators {

/* --------------------------------------------------------------------------
 *
 * Class definition
 *
 * --------------------------------------------------------------------------
 */
    class CameraPair
    {
	/* --------------------------------------------------------------------
	 * Public
	 * --------------------------------------------------------------------
	 */
        public:
        	CameraPair();
        	~CameraPair();

		void SetFirstCameraMatrix( cv::Point2d focalLength, cv::Point2d principlePoint);
		void SetSecondCameraMatrix( cv::Point2d focalLength, cv::Point2d principlePoint);
		void SetSecondCameraRotationAroundX( float angle);
		void SetSecondCameraRotationAroundY( float angle);
		void SetSecondCameraRotationAroundZ( float angle);
		void SetSecondCameraTranslation( float x, float y, float z);
		void SetSecondCameraPose(PoseWrapper::Pose3DConstPtr secondCameraPose);
		cv::Mat GetFundamentalMatrix();
		CorrespondenceMap2DWrapper::CorrespondenceMap2DConstPtr GetSomeRandomCorrespondences(unsigned correspondencesNumber, float noiseStandardDeviation = 0);
		CorrespondenceMap2DWrapper::CorrespondenceMap2DConstPtr GetSomeRandomCorrespondences(unsigned correspondencesNumber, cv::Mat& pointCloud, float noiseStandardDeviation = 0);

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
		cv::Mat firstCameraMatrix;
		cv::Mat secondCameraMatrix;
		cv::Mat secondCameraRotation;
		cv::Mat secondCameraTranslation;
		cv::Mat fundamentalMatrix;
		cv::Mat firstProjectionMatrix;
		cv::Mat secondProjectionMatrix;
		bool changed;

		void ComputeProjectionMatrices();
		void ComputeFundamentalMatrix();
		void PrepareCameraPair();
    };

}
#endif
/* CameraPair.hpp */
/** @} */
