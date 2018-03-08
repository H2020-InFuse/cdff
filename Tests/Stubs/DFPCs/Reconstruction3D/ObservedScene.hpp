/* --------------------------------------------------------------------------
*
* (C) Copyright …
*
* --------------------------------------------------------------------------
*/

/*!
 * @file ObservedScene.hpp
 * @date 08/03/2018
 * @author Alessandro Bianco
 */

/*!
 * @addtogroup DFNs
 * 
 *  This is an example implementation of the Map interface for the Reconstruction3D DFPC.
 *  
 *
 * @{
 */

#ifndef OBSERVED_SCENE_HPP
#define OBSERVED_SCENE_HPP

/* --------------------------------------------------------------------------
 *
 * Includes
 *
 * --------------------------------------------------------------------------
 */
#include <Reconstruction3D/Map.hpp>
#include <stdlib.h>
#include <vector>
#include <opencv2/core/core.hpp>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>

namespace dfpc_ci {

/* --------------------------------------------------------------------------
 *
 * Class definition
 *
 * --------------------------------------------------------------------------
 */
    class ObservedScene : public Map
    {
	/* --------------------------------------------------------------------
	 * Public
	 * --------------------------------------------------------------------
	 */
        public:
		ObservedScene();
		~ObservedScene();
		void AddFrames(FrameWrapper::FrameConstPtr leftFrame, FrameWrapper::FrameConstPtr rightFrame);
		FrameWrapper::FrameConstPtr GetNextReferenceLeftFrame();
		FrameWrapper::FrameConstPtr GetNextReferenceRightFrame();

		void AddFramePoseInReference(PoseWrapper::Pose3DConstPtr poseInReference);
		PoseWrapper::Pose3DConstPtr GetCurrentFramePoseInOrigin();
		void AddPointCloudInLastReference(PointCloudWrapper::PointCloudConstPtr pointCloudInReference);
		PointCloudWrapper::PointCloudConstPtr GetPartialScene(BaseTypesWrapper::Point3D origin, float radius);
		PointCloudWrapper::PointCloudConstPtr GetPartialScene(float radius);

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
		typedef Eigen::Transform<float, 3, Eigen::Affine, Eigen::DontAlign> AffineTransform;

		struct FrameNode
			{
			FrameWrapper::FrameConstPtr leftFrame;
			FrameWrapper::FrameConstPtr rightFrame;
			AffineTransform transformInOrigin;
			bool validTransform;
			};
		
		std::vector<FrameNode> framesMap;
		unsigned referenceFrameId;
		pcl::PointCloud<pcl::PointXYZ>::Ptr scene;	

		AffineTransform IdentityTransform();
		AffineTransform Convert(PoseWrapper::Transform3DConstPtr transform);
		PoseWrapper::Pose3DConstPtr Convert(AffineTransform affineTransform);
		pcl::PointXYZ TransformPoint(pcl::PointXYZ point, AffineTransform transform);	


    };
}
#endif
/* Map.hpp */
/** @} */
