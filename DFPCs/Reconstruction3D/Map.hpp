/* --------------------------------------------------------------------------
*
* (C) Copyright …
*
* --------------------------------------------------------------------------
*/

/*!
 * @file Map.hpp
 * @date 26/02/2018
 * @author Alessandro Bianco
 */

/*!
 * @addtogroup DFNs
 * 
 * @brief This a general interface for accessing a map database, a future implementation should link to the DPM.
 *  
 * The map offers method for storing the history of camera images at differen poses and for assembling point clouds constructed at those poses.
 * It also provides a method for iterating through the history by proceeding backward in time.
 *
 * @{
 */

#ifndef MAP_HPP
#define MAP_HPP

/* --------------------------------------------------------------------------
 *
 * Includes
 *
 * --------------------------------------------------------------------------
 */
#include <PointCloud.hpp>
#include <Pose.hpp>
#include <BaseTypes.hpp>
#include <Frame.hpp>

namespace dfpc_ci {

/* --------------------------------------------------------------------------
 *
 * Class definition
 *
 * --------------------------------------------------------------------------
 */
    class Map
    {
	/* --------------------------------------------------------------------
	 * Public
	 * --------------------------------------------------------------------
	 */
        public:
		Map() { };
		~Map() { };

		/*
		* @brief Adds a left and a right frame to the current map, the frames carry time information, but no pose information.
		*/
		virtual void AddFrames(FrameWrapper::FrameConstPtr leftFrame, FrameWrapper::FrameConstPtr rightFrame) = 0;

		/*
		* @brief Iterates through left past frames by proceeding backward in time.
		*
		* After a call to AddFrame, GetNextReferenceLeftFrame() returns the most recent left frame before the frame that was just added;
		* After a call to GetNextReferenceLeftFrame(), GetNextReferenceLeftFrame() returns the most recent frame before the output frame of the previous call;
		* It outputs NULL if AddFrame was never called or if no past frame can be found.
		*/
		virtual FrameWrapper::FrameConstPtr GetNextReferenceLeftFrame() = 0;

		/*
		* @brief Iterates through past right frames by proceeding backward in time.
		*
		* After a call to AddFrame, GetNextReferenceRightFrame() returns the most recent right frame before the frame that was just added;
		* After a call to GetNextReferenceRightFrame(), GetNextReferenceRightFrame() returns the most recent frame before the output frame of the previous call;
		* It outputs NULL if AddFrame was never called or if no past frame can be found.
		*/
		virtual FrameWrapper::FrameConstPtr GetNextReferenceRightFrame() = 0;


		/*
		* @brief Assigns a pose to the last added frame.
		*
		* @param poseInReference, this is the transform between the pose of the last added frames and the last reference frame obtained from the last call to GetNextReferenceFrame().
		*/
		virtual void AddFramePoseInReference(PoseWrapper::Pose3DConstPtr poseInReference) = 0;

		/*
		* @brief Retrieves the pose of the current frame in the coordinate system aligned to the first origin frame.
		*/
		virtual PoseWrapper::Pose3DConstPtr GetCurrentFramePoseInOrigin() = 0;

		/*
		* @brief Adds a point cloud to the total point cloud map.
		*
		* @param pointCloudInReference, this is the point cloud in the coordinate system aligned with the last reference frame obtained from the last call to GetNextReferenceFrame().
		*/
		virtual void AddPointCloudInLastReference(PointCloudWrapper::PointCloudConstPtr pointCloudInReference) = 0;

		/*
		* @brief Retrieves a point cloud given by all the mapped points which are within a give radius from a center
		*
		* @param origin, the reference center for the retrivial
		* @param radius, the reference distance from the center, if radius is negative all points are selected.
		*/
		virtual PointCloudWrapper::PointCloudConstPtr GetPartialScene(BaseTypesWrapper::Point3D origin, float radius) = 0;

		/*
		* @brief Retrieves a point cloud given by all the mapped points which are within a give radius from the current camera pose
		*
		* @param radius, the reference distance from the current camera pose, if radius is negative all points are selected.
		*/
		virtual PointCloudWrapper::PointCloudConstPtr GetPartialScene(float radius) = 0;
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

    };
}
#endif
/* Map.hpp */
/** @} */
