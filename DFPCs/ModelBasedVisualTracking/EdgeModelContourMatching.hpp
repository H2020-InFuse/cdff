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
#include <FileParser.h>
#include <GenericObjectTracker.h>

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
		FileParser parser;
		GenericObjectTracker DLRTracker;
		int status;
		bool diagnostics;
		double T_egomotion[16];
		int numberOfCameras;
		unsigned char* imgs[];
		unsigned char* img_color;
		int xrmax;
		int yrmax;

		DfpcConfigurator configurator;
		Converters::FrameToMatConverter frameToMat;
		void ConvertAsnStateToState(asn1SccRigidBodyState& poseState, double* pose, double* velocity);
		asn1SccRigidBodyState ConvertStateToAsnState(double* pose, double* velocity);
		void allocateImageMemory();
		bool edgeMatching(unsigned char** imgs, double time_images, double* T_egomotion, double* T_guess0, double* vel0, double time0,bool useInitialGuess, double* T_est, double* vel_est, double* ErrCov);
			

    };
}
}

#endif
/* EdgeModelContourMatching.hpp */
/** @} */
