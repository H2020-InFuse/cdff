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

#include <Converters/FrameToMatConverter.hpp>

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
namespace ModelBasedVisualTracking
{
    /**
     * TODO: doc
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
            asn1SccRigidBodyState& poseState, double* pose, double* velocity = NULL);
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
