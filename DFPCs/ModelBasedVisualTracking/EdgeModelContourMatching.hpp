/* --------------------------------------------------------------------------
*
* (C) Copyright …
*
* --------------------------------------------------------------------------
*/

/*!
 * @file EdgeModelContourMatching.hpp
 * @date 30/05/2018
 * @author Nassir W. Oumer
 */

/*!
 * @addtogroup DFNs
 * 
 *  This is the implementation of visual tracker, for matching model and image edges.    
 *
 * @{
 */

#ifndef EDGE_MODEL_CONTOUR_MATCHING_HPP
#define EDGE_MODEL_CONTOUR_MATCHING_HPP

/* --------------------------------------------------------------------------
 *
 * Includes
 *
 * --------------------------------------------------------------------------
 */
#include <ModelBasedVisualTracking/ModelBasedVisualTrackingInterface.hpp>
#include <DfpcConfigurator.hpp>
#include <DLRtracker_core/FileParser.h>
#include <DLRtracker_core/GenericObjectTracker.h>

#include <opencv2/core/core.hpp>
#include <opencv2/imgproc/imgproc.hpp>
#include <opencv2/imgcodecs/imgcodecs.hpp>
#include <opencv2/highgui/highgui.hpp>

#include <stdlib.h>

namespace CDFF
{
namespace DFPC
{

/* --------------------------------------------------------------------------
 *
 * Class definition
 *
 * --------------------------------------------------------------------------
 */
    class  EdgeModelContourMatching : public ModelBasedVisualTrackingInterface
    {
	/* --------------------------------------------------------------------
	 * Public
	 * --------------------------------------------------------------------
	 */
        public:
		EdgeModelContourMatching();
		~EdgeModelContourMatching();
		void run();
		void setup();
				

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
		DLRtracker::FileParser parser;
		DLRtracker::GenericObjectTracker DLRTracker;
		int status;
		int numberOfCameras;
		unsigned char* images[];
		unsigned char* imageOutputColor;
	        int xResolutionMax;
		int yResolutionMax;

		DfpcConfigurator configurator;
		Converters::FrameToMatConverter frameToMat;
		void ConvertAsnStateToState(asn1SccRigidBodyState& poseState, double* pose, double* velocity = NULL);
		asn1SccRigidBodyState ConvertStateToAsnState(double* pose, double* velocity);
		void allocateImageMemory();
		bool edgeMatching(unsigned char** images, double timeImages, double* egomotion, double* guessT0, double* velocity0, double time0,bool useInitialGuess, double* estimatedT, double* estimatedVelocity, double* ErrorCovariance);
			

    };
}
}

#endif
/* EdgeModelContourMatching.hpp */
/** @} */
