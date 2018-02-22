/* --------------------------------------------------------------------------
*
* (C) Copyright …
*
* --------------------------------------------------------------------------
*/

/*!
 * @file FeaturesDescription2DInterface.hpp
 * @date 21/02/2018
 * @author Alessandro Bianco (with code generation support)
 */

/*!
 * @addtogroup DFNs
 * 
 *  This is the common interface of all DFNs that compute a descriptor for a set of 2D points.    
 *
 * @{
 */
#ifndef FEATURES_DESCRIPTION_2D_INTERFACE_HPP
#define FEATURES_DESCRIPTION_2D_INTERFACE_HPP

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
    class FeaturesDescription2DInterface : public DFNCommonInterface
    {
	/* --------------------------------------------------------------------
	 * Public
	 * --------------------------------------------------------------------
	 */
        public:
            FeaturesDescription2DInterface();
            virtual ~FeaturesDescription2DInterface();
            /**
            * Send value to input port image
            * @param image, a 2D image taken from a camera
            */
            virtual void imageInput(FrameWrapper::FrameConstPtr data);

            /**
            * Receive value from input port featuresSet
            * @param featuresSet, This is the set of the points extracted from the image with no associated descriptor.
            */
            virtual void featuresSetInput(VisualPointFeatureVector2DWrapper::VisualPointFeatureVector2DConstPtr data);

            /**
            * Receive value from output port featuresSetWithDescriptors
            * @param featuresSetWithDescriptors, This is the set of the points extracted from the image. A descriptor is associated to each point.
            */
            virtual VisualPointFeatureVector2DWrapper::VisualPointFeatureVector2DConstPtr featuresSetWithDescriptorsOutput();

	/* --------------------------------------------------------------------
	 * Protected
	 * --------------------------------------------------------------------
	 */
        protected:
            FrameWrapper::FrameConstPtr inImage;
            VisualPointFeatureVector2DWrapper::VisualPointFeatureVector2DConstPtr inFeaturesSet;
            VisualPointFeatureVector2DWrapper::VisualPointFeatureVector2DConstPtr outFeaturesSetWithDescriptors;

	/* --------------------------------------------------------------------
	 * Private
	 * --------------------------------------------------------------------
	 */
	private:
    };
}
#endif
/* FeaturesDescription2DInterface.hpp */
/** @} */
