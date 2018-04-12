/* --------------------------------------------------------------------------
*
* (C) Copyright …
*
* --------------------------------------------------------------------------
*/

/*!
 * @file EdgeDetectionInterface.hpp
 * @date 11/04/2018
 * @author Nassir W. Oumer 
 */

/*!
 * @addtogroup DFNs
 * 
 *  This is the common interface of all DFNs that apply edge detector to an image.    
 *
 * @{
 */
#ifndef EDGE_DETECTION_INTERFACE_HPP
#define EDGE_DETECTION_INTERFACE_HPP

/* --------------------------------------------------------------------------
 *
 * Includes
 *
 * --------------------------------------------------------------------------
 */
#include <DFNCommonInterface.hpp>
#include <Frame.hpp>
#include <FrameToMatConverter.hpp>
#include <MatToFrameConverter.hpp>


namespace dfn_ci {


/* --------------------------------------------------------------------------
 *
 * Class definition
 *
 * --------------------------------------------------------------------------
 */
    class EdgeDetectionInterface : public DFNCommonInterface
    {
	/* --------------------------------------------------------------------
	 * Public
	 * --------------------------------------------------------------------
	 */
        public:
            EdgeDetectionInterface();
            virtual ~EdgeDetectionInterface();
            /**
            * Send value to input port image
            * @param image, a 2D camera image
            */
            virtual void imageInput(FrameWrapper::FrameConstPtr data);

            /**
            * Receive value from output port edgeMap
            * @param image, the edge Map output.
            */
            virtual FrameWrapper::FrameConstPtr edgeMapOutput();

	/* --------------------------------------------------------------------
	 * Protected
	 * --------------------------------------------------------------------
	 */
        protected:
            FrameWrapper::FrameConstPtr inImage;
            FrameWrapper::FrameConstPtr outEdgeMap;

	/* --------------------------------------------------------------------
	 * Private
	 * --------------------------------------------------------------------
	 */
	private:
    };
}
#endif
/* EdgeDetectionInterface.hpp */
/** @} */
