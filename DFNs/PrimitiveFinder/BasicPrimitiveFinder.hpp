/**
 * @addtogroup DFNs
 * @{
 */

#ifndef PRIMITIVEFINDER_BASICPRIMITIVEFINDER_HPP
#define PRIMITIVEFINDER_BASICPRIMITIVEFINDER_HPP

#include "PrimitiveFinderInterface.hpp"
#include <Helpers/ParametersListHelper.hpp>
#include <opencv2/core/core.hpp>

namespace CDFF
{
namespace DFN
{
namespace PrimitiveFinder
{
    /**
	 * Find primitives ( lines, circles and ellipses ). Lines and circles are currently detected using Hough transform and ellipses are detected using HuInvariants.
	 *
	 */
    class BasicPrimitiveFinder : public PrimitiveFinderInterface
    {
        public:

            BasicPrimitiveFinder();
            virtual ~BasicPrimitiveFinder();

            virtual void configure();
            virtual void process();

        private:
            struct BasicPrimitiveFinderOptionsSet
            {
                struct HoughCirclesParameters
                {
                    double dp;             //Inverse ratio of the accumulator resolution to the image resolution
                    double minDist;        //Minimum distance between the centers of the detected circles
                    double param1;         //First method-specific parameter. Higher threshold of the Canny edge detector
                    double param2;         //Second method-specific parameter. Accumulator threshold for the circle centers at detection stage.
                    double minRadius;      //Minimum circle radius
                    double maxRadius;      //Maximum circle radius
                } houghCirclesParameters;

                struct EllipsesParameters
                {
                    double minArea;         //Minimum area of the ellipse
                    double similarityRatio; //Similarity threshold to consider a contour an ellipse
                } ellipsesParameters;

                struct HoughLinesParameters
                {
                    double minLineLenght;   //Minimum line length. Line segments shorter than that are rejected.
                    double maxLineGap;      //Maximum allowed gap between points on the same line to link them.

                } houghLinesParameters;
            };

            Helpers::ParametersListHelper parametersHelper;
            BasicPrimitiveFinderOptionsSet parameters;
            static const BasicPrimitiveFinderOptionsSet DEFAULT_PARAMETERS;


            void FindPrimitive(const cv::Mat& inputImage, const std::string & primitiveName);
            void FindCircles(const cv::Mat& inputImage);
            void FindEllipses(const cv::Mat& inputImage);
            void FindLines(const cv::Mat& inputImage);

            std::vector<std::vector<double>> m_primitives;
            std::vector<cv::Point> m_ellipse_contour;
            std::string m_ellipse_path;
    };
}
}
}

#endif // PRIMITIVEFINDER_BASICPRIMITIVEFINDER_HPP

/** @} */
