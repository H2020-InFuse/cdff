/* --------------------------------------------------------------------------
*
* (C) Copyright …
*
* --------------------------------------------------------------------------
*/

/*!
 * @file CamerasTransformEstimationInterface.hpp
 * @date 31/01/2018
 * @author Alessandro Bianco (with code generation support)
 */

/*!
 * @addtogroup DFNs
 * 
 *  This is the common interface of all DFNs that estimate a camera transform from a set of 2D correspondences.    
 *
 * @{
 */
#ifndef CAMERAS_TRANSFORM_ESTIMATION_INTERFACE_HPP
#define CAMERAS_TRANSFORM_ESTIMATION_INTERFACE_HPP

/* --------------------------------------------------------------------------
 *
 * Includes
 *
 * --------------------------------------------------------------------------
 */
#include <DFNCommonInterface.hpp>
#include <Matrix.hpp>
#include <CorrespondenceMap2D.hpp>
#include <Pose.hpp>


namespace dfn_ci {


/* --------------------------------------------------------------------------
 *
 * Class definition
 *
 * --------------------------------------------------------------------------
 */
    class CamerasTransformEstimationInterface : public DFNCommonInterface
    {
	/* --------------------------------------------------------------------
	 * Public
	 * --------------------------------------------------------------------
	 */
        public:
            CamerasTransformEstimationInterface();
            virtual ~CamerasTransformEstimationInterface();
            /**
            * Send value to input port fundamentalMatrix
            * @param fundamentalMatrix, This is the fundamental matrix of the camera pair that took the two images.
            */
            virtual void fundamentalMatrixInput(MatrixWrapper::Matrix3dConstPtr data);

            /**
            * Send value to input port correspondenceMap
            * @param correspondenceMap, This is a set of correspondence between points (x, y) in one image to point (x', y') in the other image. 
	    * These are test points for the final transform, they should be as reliable as possible.
            */
            virtual void correspondenceMapInput(CorrespondenceMap2DWrapper::CorrespondenceMap2DConstPtr data);

            /**
            * Receive value from output port transform
            * @param transform, This is the relative pose of the camera that took the second image into the system of the camera that took the first image.
            */
            virtual PoseWrapper::Transform3DConstPtr transformOutput();

            /**
            * Receive value from output port success
            * @param success, This outputs tells whether the computation was succesfull. The computation may fail if the inputs are not good. 
	    * If the computation fails the first output is meaningless.
            */
            virtual bool successOutput();

	/* --------------------------------------------------------------------
	 * Protected
	 * --------------------------------------------------------------------
	 */
        protected:
            MatrixWrapper::Matrix3dConstPtr inFundamentalMatrix;
	    CorrespondenceMap2DWrapper::CorrespondenceMap2DConstPtr inCorrespondenceMap;
            PoseWrapper::Transform3DConstPtr outTransform;
	    bool outSuccess;

	/* --------------------------------------------------------------------
	 * Private
	 * --------------------------------------------------------------------
	 */
	private:
    };
}
#endif
/* CamerasTransformEstimationInterface.hpp */
/** @} */
