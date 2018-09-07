/**
 * @author Irene Sanz
 */

/**
 * @addtogroup DFNs
 * @{
 */

#include "HuInvariants.hpp"

#include <opencv2/highgui/highgui.hpp>
#include <opencv2/imgproc/imgproc.hpp>
#include <opencv2/core/core.hpp>

#include <Errors/Assert.hpp>
#include <Macros/YamlcppMacros.hpp>

#include <stdlib.h>
#include <fstream>
#include <dirent.h>

namespace
{
    //=====================================================================================================================
    std::string extractFileName(std::string filePath)
    {
        std::size_t dot_pos = filePath.rfind('.');
        std::size_t separator_pos = filePath.rfind("/");

        if(separator_pos != std::string::npos)
        {
            return filePath.substr(separator_pos + 1, dot_pos - (separator_pos+1));
        }
        return filePath;
    }
}

namespace CDFF
{
namespace DFN
{
namespace PrimitiveMatching
{

//=====================================================================================================================
HuInvariants::HuInvariants()
{
    parametersHelper.AddParameter<int>("GeneralParameters", "MinimumArea", parameters.minimumArea, DEFAULT_PARAMETERS.minimumArea);
    parametersHelper.AddParameter<std::string>("GeneralParameters", "TemplatesFolder", parameters.templatesFolder, DEFAULT_PARAMETERS.templatesFolder);

	configurationFilePath = "";
}

//=====================================================================================================================
void HuInvariants::configure()
{
	parametersHelper.ReadFile(configurationFilePath);
	ValidateParameters();
}

//=====================================================================================================================
void HuInvariants::process()
{
    m_matched_contour.clear();

	// Read data from input port
	cv::Mat inputImage = frameToMat.Convert(&inImage);

	// Process data
	ValidateInputs(inputImage);
	std::string matched_primitive = Match(inputImage);

	// Write data to output port
    outPrimitiveMatched.nCount = matched_primitive.size();
    memcpy(outPrimitiveMatched.arr, matched_primitive.data(), matched_primitive.length());

    // Create input image and draw the matched contour on it
    cv::Mat image_with_contour;
    inputImage.copyTo(image_with_contour);

    if( m_matched_contour.empty() == false )
    {
        cv::drawContours(image_with_contour, m_matched_contour, 0, cv::Scalar(0, 255, 0), 3);
    }

    // Write data to output port
    FrameWrapper::FrameConstPtr tmp = matToFrame.Convert(image_with_contour);
    FrameWrapper::Copy(*tmp, outImageWithMatchedContour);
    delete(tmp);

}

//=====================================================================================================================
const HuInvariants::HuInvariantsOptionsSet HuInvariants::DEFAULT_PARAMETERS =
{
	/*minimumArea =*/ 0,
	/*templatesFolder =*/ "../../tests/Data/Images/primitive_matching/templates/"
};

//=====================================================================================================================
std::string HuInvariants::Match(cv::Mat inputImage)
{
    //Get all .jpg files in template folder
    std::vector<std::string> template_files = getTemplateFiles();

    //Extract template contours
    std::vector<std::vector<cv::Point> > template_contours = getTemplateContours(template_files);

    //Extract new image contours
    std::vector<std::vector<cv::Point> > input_image_contours = extractAndFilterContours(inputImage);

    //Check similarity between template contours and new image contours using Hu Invariants
    return matchTemplatesAndImage(input_image_contours, template_contours, template_files);
}

//=====================================================================================================================
std::vector<std::string> HuInvariants::getTemplateFiles()
{
    std::vector<std::string> template_files;
    DIR *dir;
    struct dirent *ent;
    if ((dir = opendir (parameters.templatesFolder.c_str())) != NULL)
    {
        while ((ent = readdir (dir)) != NULL)
        {
            std::string file = ent->d_name;
            if( file.find(".jpg") != std::string::npos )
            {
                template_files.push_back(parameters.templatesFolder+file);
            }
        }
        closedir (dir);
    }

    return template_files;
}

//=====================================================================================================================
std::vector<std::vector<cv::Point> > HuInvariants::getTemplateContours(std::vector<std::string> template_files)
{
    std::vector<std::vector<cv::Point> > template_contours;
    for ( int index = 0; index< template_files.size(); index++ )
    {
        cv::Mat img = cv::imread(template_files[index]);
        std::vector<std::vector<cv::Point> > contours = extractContours(img);

        std::sort(contours.begin(), contours.end(), [](std::vector<cv::Point> a, std::vector<cv::Point> b) {
            return cv::contourArea(a) > cv::contourArea(b);
        });

        template_contours.push_back(contours[0]);
    }
    return template_contours;
}

//=====================================================================================================================
std::vector<std::vector<cv::Point> > HuInvariants::extractContours(const cv::Mat img)
{
    cv::Mat grey, threshold;
    std::vector<std::vector<cv::Point> > contours;
    std::vector<cv::Vec4i> hierarchy;

    cv::cvtColor(img, grey, cv::COLOR_BGR2GRAY);
    cv::threshold(grey, threshold, 90, 255, 0);

    cv::findContours(threshold, contours, hierarchy, cv::RETR_TREE, cv::CHAIN_APPROX_SIMPLE);
    return contours;
}

//=====================================================================================================================
std::vector<std::vector<cv::Point> > HuInvariants::extractAndFilterContours(const cv::Mat img)
{
    std::vector<std::vector<cv::Point> > input_image_contours = extractContours(img);

    int min_area = parameters.minimumArea;
    if (input_image_contours.size() > 1) {
        input_image_contours.erase(std::remove_if(input_image_contours.begin(), input_image_contours.end(),
                                                  [min_area](std::vector<cv::Point> contour) {
                                                      return cv::contourArea(contour) <= min_area;
                                                  }), input_image_contours.end());
    }
    return input_image_contours;
}

//=====================================================================================================================
std::string HuInvariants::matchTemplatesAndImage(std::vector<std::vector<cv::Point> > input_image_contours, std::vector<std::vector<cv::Point> > template_contours, std::vector<std::string> template_files)
{
    std::vector<double> similarity;
    std::vector<int> indexes_max_similarity_input_image_contour;
    for (std::vector<cv::Point> template_contour : template_contours) {
        std::vector<double> template_similarity;
        for (std::vector<cv::Point> input_image_contour : input_image_contours) {
            template_similarity.push_back(cv::matchShapes(input_image_contour, template_contour, 1, 0.0));
        }

        int index_max_element = std::distance(template_similarity.begin(),
                                              std::min_element(template_similarity.begin(),
                                                               template_similarity.end()));
        similarity.push_back(template_similarity[index_max_element]);
        indexes_max_similarity_input_image_contour.push_back(index_max_element);
    }

    int index_maximum_similarity = std::distance(similarity.begin(),
                                                 std::min_element(similarity.begin(), similarity.end()));

    m_matched_contour.push_back(
            input_image_contours[indexes_max_similarity_input_image_contour[index_maximum_similarity]]);

    auto file_name = ::extractFileName(template_files[index_maximum_similarity]);
    return file_name;
}

//=====================================================================================================================
void HuInvariants::ValidateParameters()
{
    ASSERT( parameters.templatesFolder.empty() == false, "Hu Invariants Configuration error: templates folder not defined");
}

//=====================================================================================================================
void HuInvariants::ValidateInputs(cv::Mat inputImage)
{
	ASSERT(inputImage.type() == CV_8UC3 || inputImage.type() == CV_8UC1, "Hu Invariants error: input image is not of type CV_8UC3 or CV_8UC1");
	ASSERT(inputImage.rows > 0 && inputImage.cols > 0, "Hu Invariants error: input image is empty");
}

}
}
}

/** @} */
