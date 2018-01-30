/* --------------------------------------------------------------------------
*
* (C) Copyright …
*
* --------------------------------------------------------------------------
*/

/*!
 * @file FeaturesMatching2DInterface.hpp
 * @date 29/01/2018
 * @author Alessandro Bianco (with code generation support)
 */

/*!
 * @addtogroup DFNs
 * 
 *  This is the common interface of all DFNs that matches 2d features between two images.    
 *
 * @{
 */
#ifndef FEATURES_MATCHING_2D_INTERFACE_HPP
#define FEATURES_MATCHING_2D_INTERFACE_HPP

/* --------------------------------------------------------------------------
 *
 * Includes
 *
 * --------------------------------------------------------------------------
 */
#include <DFNCommonInterface.hpp>
#include <CorrespondenceMap2D.hpp>
#include <VisualPointFeatureVector2D.hpp>


namespace dfn_ci {


/* --------------------------------------------------------------------------
 *
 * Class definition
 *
 * --------------------------------------------------------------------------
 */
    class FeaturesMatching2DInterface : public DFNCommonInterface
    {
	/* --------------------------------------------------------------------
	 * Public
	 * --------------------------------------------------------------------
	 */
        public:
            FeaturesMatching2DInterface();
            virtual ~FeaturesMatching2DInterface();
            /**
            * Send value to input port sourceFeaturesVector
            * @param sourceFeaturesVector, these are the extracted features of the 2D source image we would like to match with the sink.
            */
            virtual void sourceFeaturesVectorInput(VisualPointFeatureVector2DWrapper::VisualPointFeatureVector2DConstPtr data);

            /**
            * Send value to input port sinkFeaturesVector
            * @param sinkFeaturesVector, these are the extracted features of the 2D sink image we would like to match with the source.
            */
            virtual void sinkFeaturesVectorInput(VisualPointFeatureVector2DWrapper::VisualPointFeatureVector2DConstPtr data);

            /**
            * Receive value from output port correspondenceMap
            * @param correspondenceMap, This is the best set of correspondences between the source features set and the sink features set.
            */
            virtual CorrespondenceMap2DWrapper::CorrespondenceMap2DConstPtr correspondenceMapOutput();

	/* --------------------------------------------------------------------
	 * Protected
	 * --------------------------------------------------------------------
	 */
        protected:
            VisualPointFeatureVector2DWrapper::VisualPointFeatureVector2DConstPtr inSourceFeaturesVector;
            VisualPointFeatureVector2DWrapper::VisualPointFeatureVector2DConstPtr inSinkFeaturesVector;
            CorrespondenceMap2DWrapper::CorrespondenceMap2DConstPtr outCorrespondenceMap;

	/* --------------------------------------------------------------------
	 * Private
	 * --------------------------------------------------------------------
	 */
	private:
    };
}
#endif
/* FeaturesMatching2DInterface.hpp */
/** @} */
