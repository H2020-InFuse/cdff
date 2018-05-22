/* --------------------------------------------------------------------------
*
* (C) Copyright …
*
* --------------------------------------------------------------------------
*/

/*!
 * @file DFNsBuilder.hpp
 * @date 23/02/2018
 * @author Alessandro Bianco 
 */

/*!
 * @addtogroup DFNs
 * 
 *  @brief This is a DFN builder, it offers static method for the instantiation of a DFN implementation from the name of the DFN and implementation name of the DFN.
 *
 *  It has only one public method, which takes as input two strings (name of DFN and implementation name of the DFN) and gives a pointer to the instantiated implementation object.
 * 
 * @{
 */
#ifndef DFNS_BUILDER_HPP
#define DFNS_BUILDER_HPP


/* --------------------------------------------------------------------------
 *
 * Includes
 *
 * --------------------------------------------------------------------------
 */
#include <DFNCommonInterface.hpp>
#include <stdlib.h>
#include <string>

#include <ImageFiltering/ImageFilteringInterface.hpp>
#include <PointCloudReconstruction2DTo3D/PointCloudReconstruction2DTo3DInterface.hpp>
#include <FeaturesMatching2D/FeaturesMatching2DInterface.hpp>
#include <FeaturesExtraction2D/FeaturesExtraction2DInterface.hpp>
#include <FeaturesDescription2D/FeaturesDescription2DInterface.hpp>
#include <FundamentalMatrixComputation/FundamentalMatrixComputationInterface.hpp>
#include <CamerasTransformEstimation/CamerasTransformEstimationInterface.hpp>
#include <PerspectiveNPointSolving/PerspectiveNPointSolvingInterface.hpp>
#include <FeaturesExtraction3D/FeaturesExtraction3DInterface.hpp>
#include <FeaturesDescription3D/FeaturesDescription3DInterface.hpp>
#include <FeaturesMatching3D/FeaturesMatching3DInterface.hpp>
#include <StereoReconstruction/StereoReconstructionInterface.hpp>


namespace dfn_ci {


/* --------------------------------------------------------------------------
 *
 * Class definition
 *
 * --------------------------------------------------------------------------
 */
    class DFNsBuilder
    {
	/* --------------------------------------------------------------------
	 * Public
	 * --------------------------------------------------------------------
	 */
        public:
		static DFNCommonInterface* CreateDFN(std::string dfnType, std::string dfnImplementation);		

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
		static ImageFilteringInterface* CreateImageFiltering(std::string dfnImplementation);
		static PointCloudReconstruction2DTo3DInterface* CreatePointCloudReconstruction2DTo3D(std::string dfnImplementation);
		static FeaturesMatching2DInterface* CreateFeaturesMatching2D(std::string dfnImplementation);
		static FeaturesExtraction2DInterface* CreateFeaturesExtraction2D(std::string dfnImplementation);
		static FeaturesDescription2DInterface* CreateFeaturesDescription2D(std::string dfnImplementation);
		static FundamentalMatrixComputationInterface* CreateFundamentalMatrixComputation(std::string dfnImplementation);
		static CamerasTransformEstimationInterface* CreateCamerasTransformEstimation(std::string dfnImplementation);
		static PerspectiveNPointSolvingInterface* CreatePerspectiveNPointSolving(std::string dfnImplementation);
		static FeaturesExtraction3DInterface* CreateFeaturesExtraction3D(std::string dfnImplementation);
		static FeaturesDescription3DInterface* CreateFeaturesDescription3D(std::string dfnImplementation);
		static FeaturesMatching3DInterface* CreateFeaturesMatching3D(std::string dfnImplementation);
		static StereoReconstructionInterface* CreateStereoReconstruction(std::string dfnImplementation);
    };
}
#endif
/* DFNsBuilder.h */
/** @} */
