/**
 * @addtogroup DFNs
 * @{
 */

#ifndef IMAGEFILTERING_BACKGROUNDSUBTRACTORMOG2_HPP
#define IMAGEFILTERING_BACKGROUNDSUBTRACTORMOG2_HPP

#include "ImageFilteringInterface.hpp"
#include <opencv2/video/background_segm.hpp>
#include <Helpers/ParametersListHelper.hpp>

namespace CDFF
{
namespace DFN
{
namespace ImageFiltering
{
    /**
     * Background subtractor that uses the MOG2 implementation by OpenCV
     */
    class BackgroundSubtractorMOG2 : public ImageFilteringInterface
    {
        public:

            BackgroundSubtractorMOG2();
            virtual ~BackgroundSubtractorMOG2();

            virtual void configure();
            virtual void process();

        private:
            struct BackgroundSubtractorMOG2OptionsSet
            {
                int erosionSize;        //  Size of the kernel used in the blur and the dilate applied to the input image and the foreground mask respectively
                int history;            //  Number of last frames that affect the background model
                int nMixtures;          //  Number of gaussian components in the background model
                double backgroundRatio; //  If a foreground pixel keeps semi-constant value for about backgroundRatio*history frames, it's considered background and added to the model as a center of a new component.
                bool detectShadows;     //  If true, the algorithm detects shadows and marks them.
            };

            Helpers::ParametersListHelper parametersHelper;
            BackgroundSubtractorMOG2OptionsSet parameters;
            static const BackgroundSubtractorMOG2OptionsSet DEFAULT_PARAMETERS;

            cv::Ptr<cv::BackgroundSubtractorMOG2> pMOG2;

    };
}
}
}

#endif // IMAGEFILTERING_BACKGROUNDSUBTRACTORMOG2_HPP

/** @} */
