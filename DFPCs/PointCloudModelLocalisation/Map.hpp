/**
 * @author Alessandro Bianco
 */

/**
 * @addtogroup DFPCs
 * @{
 */

#ifndef POINTCLOUDMODELLOCALISATION_MAP_HPP
#define POINTCLOUDMODELLOCALISATION_MAP_HPP

#include <PointCloud.hpp>
#include <Pose.hpp>
#include <BaseTypes.hpp>
#include <Frame.hpp>

namespace CDFF
{
namespace DFPC
{
namespace PointCloudModelLocalisation
{
	/**
	 * This a general interface for accessing a map database, a future implementation should link to the DPM.
	 *
	 * The map offers method for storing the history of camera images at differen poses and for assembling point clouds constructed at those poses.
	 * It also provides a method for iterating through the history by proceeding backward in time.
	 */
	class Map
	{
	public:

		Map() { };
		~Map() { };

		/*
		* @brief Adds a frame to the current map, the frame carries time information, but no pose information. The frame can be either the left or right image of a stereo camera.
		*/
		virtual void AddFrame(FrameWrapper::FrameConstPtr frame) = 0;

		/*
		* @brief Iterates through past frames by proceeding backward in time.
		*
		* After a call to AddFrame, GetNextReferenceFrame() returns the most recent frame before the frame that was just added;
		* After a call to GetNextReferenceFrame(), GetNextReferenceFrame() returns the most recent frame before the output frame of the previous call;
		* It outputs NULL if AddFrame was never called or if no past frame can be found.
		*/
		virtual FrameWrapper::FrameConstPtr GetNextReferenceFrame() = 0;

		/*
		* @brief Assigns a pose to the last added frame.
		*
		* @param poseInReference, this is the transform between the pose of the last added frame and the last reference frame obtained from the last call to GetNextReferenceFrame().
		*/
		virtual void AddFramePose(PoseWrapper::Pose3DConstPtr poseInReference) = 0;

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
	};
}
}
}

#endif // POINTCLOUDMODELLOCALISATION_MAP_HPP

/** @} */
