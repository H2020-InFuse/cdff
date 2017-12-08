/* --------------------------------------------------------------------------
*
* (C) Copyright …
*
* --------------------------------------------------------------------------
*/

/*!
 * @file StubFeaturesExtraction2D.hpp
 * @date 15/11/2017
 * @author Alessandro Bianco
 */

/*!
 * @addtogroup DFNs
 * 
 *  This is a stub DFN for the extraction of 2D keypoint from a 2D image. 
 *  At the moment it does nothing, but we plan to add a fake behaviour.  
 *
 * @{
 */
#ifndef STUB_FEATURE_EXTRACTION_2D_HPP
#define STUB_FEATURES_EXTRACTION_2D_HPP

/* --------------------------------------------------------------------------
 *
 * Includes
 *
 * --------------------------------------------------------------------------
 */
#include <FeaturesExtraction2D/FeaturesExtraction2DInterface.hpp>
#include <Frame.hpp>
#include <VisualPointFeatureVector2D.hpp>


namespace dfn_ci {

/* --------------------------------------------------------------------------
 *
 * Class definition
 *
 * --------------------------------------------------------------------------
 */
    class StubFeaturesExtraction2D : public FeaturesExtraction2DInterface
    {
	/* --------------------------------------------------------------------
	 * Public
	 * --------------------------------------------------------------------
	 */
        public:
            StubFeaturesExtraction2D();
            ~StubFeaturesExtraction2D();
            void process();
            void configure();

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
/* StubFeaturesExtraction2D.hpp */
/** @} */
