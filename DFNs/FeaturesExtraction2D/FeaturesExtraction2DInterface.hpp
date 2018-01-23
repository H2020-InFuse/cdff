/* --------------------------------------------------------------------------
*
* (C) Copyright …
*
* --------------------------------------------------------------------------
*/

/*!
 * @file FeaturesExtraction2DInterface.hpp
 * @date 15/11/2017
 * @author Alessandro Bianco (with code generation support)
 */

/*!
 * @addtogroup DFNs
 * 
 *  This is the common interface of all DFNs that extract 2D keypoints from a 2D image.    
 *
 * @{
 */
#ifndef FEATURES_EXTRACTION_2D_INTERFACE_HPP
#define FEATURES_EXTRACTION_2D_INTERFACE_HPP

/* --------------------------------------------------------------------------
 *
 * Includes
 *
 * --------------------------------------------------------------------------
 */
#include <DFNCommonInterface.hpp>
#include <Frame.hpp>
#include <VisualPointFeatureVector2D.hpp>
#include <FrameToMatConverter.hpp>


namespace dfn_ci {


/* --------------------------------------------------------------------------
 *
 * Class definition
 *
 * --------------------------------------------------------------------------
 */
    class FeaturesExtraction2DInterface : public DFNCommonInterface
    {
	/* --------------------------------------------------------------------
	 * Public
	 * --------------------------------------------------------------------
	 */
        public:
            FeaturesExtraction2DInterface();
            virtual ~FeaturesExtraction2DInterface();
            /**
            * Send value to input port image
            * @param image, a 2D image taken from a camera
            */
            virtual void imageInput(FrameWrapper::FrameConstPtr data);

            /**
            * Receive value from output port featuresSet
            * @param featuresSet, This is the set of the points extracted from the image, a descriptor may or may not be provided.
            */
            virtual VisualPointFeatureVector2DWrapper::VisualPointFeatureVector2DConstPtr featuresSetOutput();

	/* --------------------------------------------------------------------
	 * Protected
	 * --------------------------------------------------------------------
	 */
        protected:
            FrameWrapper::FrameConstPtr inImage;
            VisualPointFeatureVector2DWrapper::VisualPointFeatureVector2DConstPtr outFeaturesSet;

	/* --------------------------------------------------------------------
	 * Private
	 * --------------------------------------------------------------------
	 */
	private:
    };
}
#endif
/* FeaturesExtraction2DInterface.hpp */
/** @} */
