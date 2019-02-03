/**
 * @author Nassir W. Oumer
 */

/**
 * @addtogroup DFPCs
 * @{
 */

#ifndef MODELBASEDVISUALTRACKING_EDGEMODELCONTOURMATCHING_HPP
#define MODELBASEDVISUALTRACKING_EDGEMODELCONTOURMATCHING_HPP

#include <ModelBasedVisualTracking/ModelBasedVisualTrackingInterface.hpp>
#include <Types/C/RigidBodyState.h>
#include <Converters/FrameToMatConverter.hpp>

#include <DLRtracker_core/FileParser.h>
#include <DLRtracker_core/GenericObjectTracker.h>

namespace CDFF
{
namespace DFPC
{
namespace ModelBasedVisualTracking
{
    /**
     * The EdgeModelContourMatching class implements a ModelBasedVisualTracking
     * DFPC by wrapping up functions of the DLRTracker-core proprietary library,
     * instead of being composed of DFNs. For that reason, it is an exception to
     * DFPCs being made up of DFNs, and therefore not an actual DFPC
     * implementation.
     */
    class EdgeModelContourMatching : public ModelBasedVisualTrackingInterface
    {
        public:

            EdgeModelContourMatching();
            ~EdgeModelContourMatching();
            void run();
            void setup();

        private:

            DLRtracker::FileParser parser;
            DLRtracker::GenericObjectTracker DLRTracker;
            int status;
            int numberOfCameras;
            unsigned char* images[];
            unsigned char* imageOutputColor;
            int xResolutionMax;
            int yResolutionMax;

            Converters::FrameToMatConverter frameToMat;

            void ConvertASN1StateToState(
                asn1SccRigidBodyState& poseState,
                double* pose, double* velocity = NULL);
            asn1SccRigidBodyState ConvertStateToASN1State(
                double* pose, double* velocity);

            void allocateImageMemory();
            bool edgeMatching(
                unsigned char** images, double timeImages, double* egomotion,
                double* guessT0, double* velocity0, double time0, bool useInitialGuess,
                double* estimatedT, double* estimatedVelocity, double* ErrorCovariance);
    };
}
}
}

#endif // MODELBASEDVISUALTRACKING_EDGEMODELCONTOURMATCHING_HPP

/** @} */
