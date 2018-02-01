/* --------------------------------------------------------------------------
*
* (C) Copyright …
*
* --------------------------------------------------------------------------
*/

/*!
 * @file Triangulation.hpp
 * @date 29/01/2018
 * @author Alessandro Bianco
 */

/*!
 * @addtogroup DFNs
 * 
 *  This DFN implements the triangulation algorithm for the reconstruction of 3D point clouds from matched 2d images.
 *  
 *
 * @{
 */

#ifndef TRIANGULATION_HPP
#define TRIANGULATION_HPP

/* --------------------------------------------------------------------------
 *
 * Includes
 *
 * --------------------------------------------------------------------------
 */
#include <PointCloudReconstruction2DTo3D/PointCloudReconstruction2DTo3DInterface.hpp>
#include <CorrespondenceMap2D.hpp>
#include <Pose.hpp>
#include <PointCloud.hpp>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <stdlib.h>
#include <string>
#include <pcl/keypoints/harris_3d.h>
#include <yaml-cpp/yaml.h>
#include <SupportTypes.hpp>
#include "opencv2/calib3d.hpp"
#include <Helpers/ParametersListHelper.hpp>

namespace dfn_ci {

/* --------------------------------------------------------------------------
 *
 * Class definition
 *
 * --------------------------------------------------------------------------
 */
    class Triangulation : public PointCloudReconstruction2DTo3DInterface
    {
	/* --------------------------------------------------------------------
	 * Public
	 * --------------------------------------------------------------------
	 */
        public:
        	Triangulation();
        	~Triangulation();
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
		enum CAMERA_TYPE
			{
			SOURCE_CAMERA,
			SINK_CAMERA
			};

		cv::Mat Triangulate(cv::Mat projectionMatrix, cv::Mat pointsVectorAtPose1, cv::Mat pointsVectorAtPose2);
		cv::Mat Convert(PoseWrapper::Transform3DConstPtr transform);
		cv::Mat ConvertAtPose(CorrespondenceMap2DWrapper::CorrespondenceMap2DConstPtr correspondenceMap, CAMERA_TYPE cameraPoseIdentifier);
		PointCloudWrapper::PointCloudConstPtr Convert(cv::Mat pointCloudMatrix);

		void ValidateInputs(CorrespondenceMap2DWrapper::CorrespondenceMap2DConstPtr correspondenceMap, PoseWrapper::Transform3DConstPtr transform);
    };
}
#endif
/* Triangulation.hpp */
/** @} */
