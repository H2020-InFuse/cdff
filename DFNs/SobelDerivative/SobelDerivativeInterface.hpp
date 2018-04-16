/* --------------------------------------------------------------------------
*
* (C) Copyright …
*
* --------------------------------------------------------------------------
*/

/*!
 * @file SobelDerivativeInterface.hpp
 * @date 12/04/2018
 * @author Nassir W. Oumer 
 */

/*!
 * @addtogroup DFNs
 * 
 *  This is the common interface of DFNs that apply Sobel derivative to an image.    
 *
 * @{
 */
/* --------------------------------------------------------------------------
 *
 * Includes
 *
 * --------------------------------------------------------------------------
 */
#ifndef SOBELDERIVATIVE_INTERFACE_HPP
#define SOBELDERIVATIVE_INTERFACE_HPP

#include <DFNCommonInterface.hpp>
#include <Frame.hpp>
#include <FrameToMatConverter.hpp>
#include <MatToFrameConverter.hpp>

namespace dfn_ci {
    class SobelDerivativeInterface : public DFNCommonInterface
    {
        public:
            SobelDerivativeInterface();
            virtual ~SobelDerivativeInterface();
            /**
            * Send value to input port image
            * @param image, a 2D  camera image
            */
            virtual void imageInput(FrameWrapper::FrameConstPtr data);

            /**
            * Receive value from output port sobelGradx
            * @return sobelGradx, the image gradient in x-direction output
            */
            virtual FrameWrapper::FrameConstPtr sobelGradxOutput();
            /**
            * Receive value from output port sobelGrady
            * @return sobelGrady, the image gradient in y-direction output
            */
            virtual FrameWrapper::FrameConstPtr sobelGradyOutput();
  
        protected:
              FrameWrapper::FrameConstPtr inImage;
              FrameWrapper::FrameConstPtr outSobelGradx;
              FrameWrapper::FrameConstPtr outSobelGrady;

	/* --------------------------------------------------------------------
	 * Private
	 * --------------------------------------------------------------------
	 */
	private:
    };
}
#endif
/* SobelDerivativeInterface.hpp */
/** @} */
