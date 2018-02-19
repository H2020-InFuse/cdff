/* --------------------------------------------------------------------------
*
* (C) Copyright …
*
* --------------------------------------------------------------------------
*/

/*!
 * @file ImageFilteringInterface.hpp
 * @date 19/02/2018
 * @author Alessandro Bianco (with code generation support)
 */

/*!
 * @addtogroup DFNs
 * 
 *  This is the common interface of all DFNs that apply some filter to the image.    
 *
 * @{
 */
#ifndef IMAGE_FILTERING_INTERFACE_HPP
#define IMAGE_FILTERING_INTERFACE_HPP

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
    class ImageFilteringInterface : public DFNCommonInterface
    {
	/* --------------------------------------------------------------------
	 * Public
	 * --------------------------------------------------------------------
	 */
        public:
            ImageFilteringInterface();
            virtual ~ImageFilteringInterface();
            /**
            * Send value to input port image
            * @param image, a 2D image taken from a camera
            */
            virtual void imageInput(FrameWrapper::FrameConstPtr data);

            /**
            * Receive value from output port undistortedImage
            * @param undistortedImage, the undistorted image output.
            */
            virtual FrameWrapper::FrameConstPtr filteredImageOutput();

	/* --------------------------------------------------------------------
	 * Protected
	 * --------------------------------------------------------------------
	 */
        protected:
            FrameWrapper::FrameConstPtr inImage;
            FrameWrapper::FrameConstPtr outFilteredImage;

	/* --------------------------------------------------------------------
	 * Private
	 * --------------------------------------------------------------------
	 */
	private:
    };
}
#endif
/* ImageFilteringInterface.hpp */
/** @} */
