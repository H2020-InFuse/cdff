/**
 * @addtogroup DFNs
 * @{
 */

#include "BackgroundSubtractorMOG2.hpp"
#include <Converters/FrameToMatConverter.hpp>
#include <Converters/MatToFrameConverter.hpp>

#include <opencv2/imgproc/imgproc.hpp>

namespace CDFF
{
namespace DFN
{
namespace ImageFiltering
{

BackgroundSubtractorMOG2::BackgroundSubtractorMOG2()
:pMOG2()
{
    configurationFilePath = "";
    parametersHelper.AddParameter<int>("GeneralParameters", "ErosionSize", parameters.erosionSize, DEFAULT_PARAMETERS.erosionSize);
    parametersHelper.AddParameter<int>("GeneralParameters", "History", parameters.history, DEFAULT_PARAMETERS.history);
    parametersHelper.AddParameter<int>("GeneralParameters", "NMixtures", parameters.nMixtures, DEFAULT_PARAMETERS.nMixtures);
    parametersHelper.AddParameter<double>("GeneralParameters", "BackgroundRatio", parameters.backgroundRatio, DEFAULT_PARAMETERS.backgroundRatio);
    parametersHelper.AddParameter<bool>("GeneralParameters", "DetectShadows", parameters.detectShadows, DEFAULT_PARAMETERS.detectShadows);

    pMOG2->setHistory(parameters.history);
    pMOG2->setNMixtures(parameters.nMixtures);
    pMOG2->setBackgroundRatio(parameters.backgroundRatio);
    pMOG2->setDetectShadows(parameters.detectShadows);
}

const BackgroundSubtractorMOG2::BackgroundSubtractorMOG2OptionsSet BackgroundSubtractorMOG2::DEFAULT_PARAMETERS =
{
    /* erosionSize= */  10,
    /* history= */ 100,
    /* nMixtures= */ 3,
    /* backgroundRatio= */ 0.7,
    /* detectShadows= */ false
};

BackgroundSubtractorMOG2::~BackgroundSubtractorMOG2()
{
}

void BackgroundSubtractorMOG2::configure()
{
    if( configurationFilePath.empty() == false )
    {
        parametersHelper.ReadFile(configurationFilePath);

        pMOG2->setHistory(parameters.history);
        pMOG2->setNMixtures(parameters.nMixtures);
        pMOG2->setBackgroundRatio(parameters.backgroundRatio);
        pMOG2->setDetectShadows(parameters.detectShadows);
    }
}

void BackgroundSubtractorMOG2::process()
{
    cv::Mat left_image = Converters::FrameToMatConverter().Convert(&inImage);
    cv::Mat mask, image_without_background;
    cv::Mat blurred_image_for_background_substractor;
    cv::blur(left_image, blurred_image_for_background_substractor, cv::Size(parameters.erosionSize, parameters.erosionSize));

    pMOG2->apply(blurred_image_for_background_substractor, mask);

    cv::Mat element = cv::getStructuringElement(cv::MORPH_RECT, cv::Size(2 * parameters.erosionSize + 1, 2 * parameters.erosionSize + 1), cv::Point(parameters.erosionSize, parameters.erosionSize) );
    cv::dilate(mask, mask, element);

    std::vector<std::vector<cv::Point> > contours;
    std::vector<cv::Vec4i> hierarchy;
    cv::findContours(mask, contours, hierarchy, CV_RETR_EXTERNAL, CV_CHAIN_APPROX_NONE);
    std::sort(contours.begin(), contours.end(), [](std::vector<cv::Point> a, std::vector<cv::Point> b) {
        return cv::contourArea(a) > cv::contourArea(b);
    });

    for( int index = 1; index < contours.size(); index ++ )
    {
        cv::fillPoly(mask, std::vector<std::vector<cv::Point> >(1,contours[index]), (0,0,0));
    }

    cv::bitwise_and(left_image, mask, image_without_background);
    FrameWrapper::FrameConstPtr frame_without_background = Converters::MatToFrameConverter().Convert(image_without_background);
    FrameWrapper::Copy(*frame_without_background, outImage);
    delete frame_without_background;

}

}
}
}

/** @} */
