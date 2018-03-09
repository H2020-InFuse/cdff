/* --------------------------------------------------------------------------
*
* (C) Copyright …
*
* --------------------------------------------------------------------------
*/

/*!
 * @file StereoReconstructionInterface.hpp
 * @date 08/03/2018
 * @author Alessandro Bianco (with code generation support)
 */

/*!
 * @addtogroup DFNs
 * 
 *  This is the common interface of all DFNs that reconstruct a 3d scene from a pair of stereo images.    
 *
 * @{
 */
#ifndef STEREO_RECONSTRUCTION_INTERFACE_HPP
#define STEREO_RECONSTRUCTION_INTERFACE_HPP

/* --------------------------------------------------------------------------
 *
 * Includes
 *
 * --------------------------------------------------------------------------
 */
#include <DFNCommonInterface.hpp>
#include <PointCloud.hpp>
#include <Frame.hpp>


namespace dfn_ci {


/* --------------------------------------------------------------------------
 *
 * Class definition
 *
 * --------------------------------------------------------------------------
 */
    class StereoReconstructionInterface : public DFNCommonInterface
    {
	/* --------------------------------------------------------------------
	 * Public
	 * --------------------------------------------------------------------
	 */
        public:
            StereoReconstructionInterface();
            virtual ~StereoReconstructionInterface();
            /**
            * Send value to input port leftImage
            * @param leftImage, The left image taken by a stereo camera.
            */
            virtual void leftImageInput(FrameWrapper::FrameConstPtr data);

            /**
            * Send value to input port rightImage
            * @param rightImage, The right image taken by a stereo camera.
            */
            virtual void rightImageInput(FrameWrapper::FrameConstPtr data);

            /**
            * Receive value from output port pointCloud
            * @param pointCloud, This is the 3d point cloud representing the scene in front of the stereo camera. The coordinate system is aligned with the position of the left camera.
            */
            virtual PointCloudWrapper::PointCloudConstPtr pointCloudOutput();

	/* --------------------------------------------------------------------
	 * Protected
	 * --------------------------------------------------------------------
	 */
        protected:
            FrameWrapper::FrameConstPtr inLeftImage;
	    FrameWrapper::FrameConstPtr inRightImage;
            PointCloudWrapper::PointCloudConstPtr outPointCloud;

	/* --------------------------------------------------------------------
	 * Private
	 * --------------------------------------------------------------------
	 */
	private:
    };
}
#endif
/* StereoReconstructionInterface.hpp */
/** @} */
