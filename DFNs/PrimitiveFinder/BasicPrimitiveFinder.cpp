/**
 * @addtogroup DFNs
 * @{
 */

#include "BasicPrimitiveFinder.hpp"

#include <Converters/FrameToMatConverter.hpp>
#include <Converters/MatToFrameConverter.hpp>

#include <opencv2/imgproc/imgproc.hpp>

namespace
{
    const std::string CIRCLE = "circle";
    const std::string LINE = "line";
    const std::string ELLIPSE = "ellipse";
}

namespace CDFF
{
namespace DFN
{
namespace PrimitiveFinder
{

BasicPrimitiveFinder::BasicPrimitiveFinder()
{
    if( std::getenv("CDFFPATH") )
    {
        m_ellipse_path = std::string(std::getenv("CDFFPATH"))+"/Tests/Data/Images/primitive_matching/templates/ellipse.yml";
    }
    else
    {
        m_ellipse_path = "../tests/Data/Images/primitive_matching/templates/ellipse.yml";
    }

    configurationFilePath = "";
    parametersHelper.AddParameter<double>("HoughCirclesParameters", "DP", parameters.houghCirclesParameters.dp, DEFAULT_PARAMETERS.houghCirclesParameters.dp);
    parametersHelper.AddParameter<double>("HoughCirclesParameters", "MinDist", parameters.houghCirclesParameters.minDist, DEFAULT_PARAMETERS.houghCirclesParameters.minDist);
    parametersHelper.AddParameter<double>("HoughCirclesParameters", "Param1", parameters.houghCirclesParameters.param1, DEFAULT_PARAMETERS.houghCirclesParameters.param1);
    parametersHelper.AddParameter<double>("HoughCirclesParameters", "Param2", parameters.houghCirclesParameters.param2, DEFAULT_PARAMETERS.houghCirclesParameters.param2);
    parametersHelper.AddParameter<double>("HoughCirclesParameters", "MinRadius", parameters.houghCirclesParameters.minRadius, DEFAULT_PARAMETERS.houghCirclesParameters.minRadius);
    parametersHelper.AddParameter<double>("HoughCirclesParameters", "MaxRadius", parameters.houghCirclesParameters.maxRadius, DEFAULT_PARAMETERS.houghCirclesParameters.maxRadius);
    parametersHelper.AddParameter<double>("EllipsesParameters", "MinArea", parameters.ellipsesParameters.minArea, DEFAULT_PARAMETERS.ellipsesParameters.minArea);
    parametersHelper.AddParameter<double>("EllipsesParameters", "SimilarityRatio", parameters.ellipsesParameters.similarityRatio, DEFAULT_PARAMETERS.ellipsesParameters.similarityRatio);
    parametersHelper.AddParameter<double>("HoughLinesParameters", "MinLineLenght", parameters.houghLinesParameters.minLineLenght, DEFAULT_PARAMETERS.houghLinesParameters.minLineLenght);
    parametersHelper.AddParameter<double>("HoughLinesParameters", "MaxLineGap", parameters.houghLinesParameters.maxLineGap, DEFAULT_PARAMETERS.houghLinesParameters.maxLineGap);

    cv::FileStorage file(m_ellipse_path, cv::FileStorage::READ);
    file["ellipse"] >> m_ellipse_contour;
    file.release();
}

BasicPrimitiveFinder::~BasicPrimitiveFinder()
{
}

const BasicPrimitiveFinder::BasicPrimitiveFinderOptionsSet BasicPrimitiveFinder::DEFAULT_PARAMETERS =
{
    //.houghCirclesParameters =
    {
        /*dp =*/ 1.2,
        /*minDist =*/ 300,
        /*param1 =*/ 100,
        /*param2 =*/ 1,
        /*minRadius =*/ 10,
        /*maxRadius =*/ 0
    },
    //.ellipsesParameters =
    {
        /*minArea =*/ 500,
        /*similarityRatio =*/ 0.15
    },
    //.houghLinesParameters =
    {
        /*minLineLenght =*/ 100,
        /*maxLineGap =*/ 10
    }
};

void BasicPrimitiveFinder::configure()
{
    if( configurationFilePath.empty() == false )
    {
        parametersHelper.ReadFile(configurationFilePath);
    }
}

void BasicPrimitiveFinder::process()
{
    asn1SccVectorXdSequence_Initialize(&outPrimitives);
    cv::Mat inputImage = Converters::FrameToMatConverter().Convert(&inImage);
    std::string primitive (reinterpret_cast<char const*>(inPrimitive.arr), inPrimitive.nCount);

    FindPrimitive(inputImage, primitive);

    outPrimitives.nCount = m_primitives.size();
    for( int index = 0; index < outPrimitives.nCount; index ++ )
    {
        outPrimitives.arr[index].nCount = m_primitives[index].size();
        for( int index2 = 0; index2 < outPrimitives.arr[index].nCount; index2++ )
        {
            outPrimitives.arr[index].arr[index2] = m_primitives[index][index2];
        }
    }

    m_primitives.clear();
}

void BasicPrimitiveFinder::FindPrimitive(const cv::Mat& inputImage, const std::string & primitiveName)
{
    //Convert to grey
    cv::Mat grey;
    if( inputImage.type() != CV_8UC1 )
    {
        cv::cvtColor(inputImage, grey, cv::COLOR_BGR2GRAY);
    }
    else
    {
        grey = inputImage;
    }

    if( primitiveName == ::CIRCLE )
    {
        FindCircles(grey);
    }
    else if( primitiveName == ::LINE )
    {
        FindLines(grey);
    }
    else if( primitiveName == ::ELLIPSE )
    {
        FindEllipses(grey);
    }
}

void BasicPrimitiveFinder::FindCircles(const cv::Mat& inputImage)
{
    std::vector<cv::Vec3f> circles;
    cv::HoughCircles(inputImage, circles, CV_HOUGH_GRADIENT,
            parameters.houghCirclesParameters.dp,
            parameters.houghCirclesParameters.minDist,
            parameters.houghCirclesParameters.param1,
            parameters.houghCirclesParameters.param2,
            parameters.houghCirclesParameters.minRadius,
            parameters.houghCirclesParameters.maxRadius);


    //Filter incomplete circles
    circles.erase(std::remove_if(circles.begin(), circles.end(),
                                 [inputImage](cv::Vec3f circle) {
                                     bool circle_inside_image =
                                             (circle[0] + circle[2] <= inputImage.cols) &&
                                             (circle[1] + circle[2] <= inputImage.rows) &&
                                             (circle[0] - circle[2] >= 0) &&
                                             (circle[1] - circle[2] >= 0);

                                     auto center_is_in_background = static_cast<int>(inputImage.at<uchar>(cv::Point(circle[0], circle[1]))) == 0;
                                     return !circle_inside_image || center_is_in_background;
                                 }), circles.end());

    //Add circles to output
    for(auto index = 0; index < circles.size(); index ++ )
    {
        std::vector<double> data;
        data.push_back(circles[index][0]);
        data.push_back(circles[index][1]);
        data.push_back(circles[index][2]);
        m_primitives.push_back(data);
    }
}

void BasicPrimitiveFinder::FindEllipses(const cv::Mat& inputImage)
{
    std::vector<std::vector<double>> ellipses;
    std::vector<std::vector<cv::Point> > contours;
    std::vector<cv::Vec4i> hierarchy;
    cv::Mat threshold;

    cv::adaptiveThreshold(inputImage, threshold, 255,CV_ADAPTIVE_THRESH_GAUSSIAN_C, CV_THRESH_BINARY_INV,11, 1);
    cv::findContours(threshold, contours, hierarchy, CV_RETR_TREE, CV_CHAIN_APPROX_SIMPLE);

    std::sort(contours.begin(), contours.end(), [](std::vector<cv::Point> a, std::vector<cv::Point> b) {
        return cv::contourArea(a) > cv::contourArea(b);
    });

    int min_area = parameters.ellipsesParameters.minArea;
    if (contours.size() > 1) {
        contours.erase(std::remove_if(contours.begin(), contours.end(),
                                      [min_area](std::vector<cv::Point> contour) {
                                          return cv::contourArea(contour) <= min_area;
                                      }), contours.end());
    }

    for ( int index = 0; index < contours.size(); index ++)
    {
        std::vector<cv::Point> hull;
        cv::convexHull( cv::Mat(contours[index]), hull, false );

        if(hull.size() > 5 )
        {
            std::map<std::string, std::vector<cv::Point>>::iterator it;

            if(m_ellipse_contour.empty() == false)
            {
                auto similarity_ratio = cv::matchShapes(hull, m_ellipse_contour, 1, 0.0);
                if( similarity_ratio < parameters.ellipsesParameters.similarityRatio )
                {
                    auto ellipse_data = fitEllipse(hull);
                    std::vector<double> data;
                    data.push_back(ellipse_data.center.x);
                    data.push_back(ellipse_data.center.y);
                    data.push_back(ellipse_data.size.width);
                    data.push_back(ellipse_data.size.height);
                    data.push_back(ellipse_data.angle);

                    m_primitives.push_back(data);
                    ellipses.push_back(data);
                }
            }
        }
    }
}

void BasicPrimitiveFinder::FindLines(const cv::Mat& inputImage)
{
    std::vector<cv::Vec4i> lines;
    cv::Mat edges;
    cv::Canny(inputImage, edges, 50, 200, 3);
    cv::HoughLinesP( edges, lines, 1, CV_PI/180, 80, parameters.houghLinesParameters.minLineLenght, parameters.houghLinesParameters.maxLineGap);

    for(auto index = 0; index < lines.size(); index ++ )
    {
        std::vector<double> data;
        data.push_back(lines[index][0]);
        data.push_back(lines[index][1]);
        data.push_back(lines[index][2]);
        data.push_back(lines[index][3]);
        m_primitives.push_back(data);
    }
}

}
}
}

/** @} */
