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

    //=====================================================================================================================
    std::vector<std::string> getTemplateFiles(const std::string & path)
    {
        std::vector<std::string> template_files;
        DIR *dir;
        struct dirent *ent;
        if ((dir = opendir (path.c_str())) != NULL)
        {
            while ((ent = readdir (dir)) != NULL)
            {
                std::string file = ent->d_name;
                if( file.find(".jpg") != std::string::npos )
                {
                    template_files.push_back(path+file);
                }
            }
            closedir (dir);
        }

        return template_files;
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
    parameters = DEFAULT_PARAMETERS;

    parametersHelper.AddParameter<int>("GeneralParameters", "MinimumArea", parameters.minimumArea, DEFAULT_PARAMETERS.minimumArea);
    parametersHelper.AddParameter<std::string>("GeneralParameters", "TemplatesFolder", parameters.templatesFolder, DEFAULT_PARAMETERS.templatesFolder);
    parametersHelper.AddParameter<double>("GeneralParameters", "MaximumSimilarityRatio", parameters.maximumSimilarityRatio, DEFAULT_PARAMETERS.maximumSimilarityRatio);

	configurationFilePath = "";
}

//=====================================================================================================================
void HuInvariants::configure()
{
	parametersHelper.ReadFile(configurationFilePath);
	ValidateParameters();

    //Get all .jpg files in template folder and extract template contours
    m_template_files = ::getTemplateFiles(parameters.templatesFolder);
    m_template_contours = getTemplateContours();
}

//=====================================================================================================================
void HuInvariants::process()
{
	// Read data from input port
	cv::Mat inputImage = frameToMat.Convert(&inImage);

	// Process data
	ValidateInputs(inputImage);

	// Write data to output port
    outPrimitives = Converters::StdVectorOfStringsToStringSequenceConverter().Convert( Match(inputImage) );


    // Create input image and draw the matched contour on it
    cv::Mat image_with_contour = drawContoursAndInformationOnOutputImage(inputImage);

    // Write data to output port
    FrameWrapper::FrameConstPtr tmp = matToFrame.Convert(image_with_contour);
    FrameWrapper::Copy(*tmp, outImage);
    delete(tmp);

}

//=====================================================================================================================
cv::Mat HuInvariants::drawContoursAndInformationOnOutputImage(const cv::Mat& inputImage)
{
    cv::Mat image_with_contour;
    inputImage.copyTo(image_with_contour);

    // Draw the contours and matched primitive on the original image
    for (auto info : m_matching_info)
    {
        if (info.matched_contour.empty() == false)
        {
            std::vector<std::vector<cv::Point> > contours;
            contours.push_back(info.matched_contour);
            auto box = cv::boundingRect(info.matched_contour);
            cv::Point text_position(box.x, box.y + (box.height / 2));
            cv::drawContours(image_with_contour, contours, 0, cv::Scalar(0, 255, 0), 3);
            cv::putText(image_with_contour, info.primitive, text_position, 3, cv::FONT_HERSHEY_PLAIN, cv::Scalar(255, 0, 0));
        }
    }
    return image_with_contour;
}

//=====================================================================================================================
const HuInvariants::HuInvariantsOptionsSet HuInvariants::DEFAULT_PARAMETERS =
{
	/*minimumArea =*/ 0,
	/*templatesFolder =*/ "../../tests/Data/Images/primitive_matching/templates/",
    /*maximumSimilarityRatio =*/ 0.2

};

//=====================================================================================================================
std::vector< std::string > HuInvariants::Match(const cv::Mat& inputImage)
{
    //Extract new image contours
    std::vector<std::vector<cv::Point> > input_image_contours = extractContours(inputImage);
    filterContours(input_image_contours);

    //Check similarity between template contours and new image contours using Hu Invariants
    matchTemplatesAndImage(input_image_contours);

    std::vector<std::string> primitives_ordered_by_matching_probability;
    for( auto info: m_matching_info )
    {
        primitives_ordered_by_matching_probability.push_back(info.primitive);
    }

    return primitives_ordered_by_matching_probability;
}

//=====================================================================================================================
std::vector<std::vector<cv::Point> > HuInvariants::getTemplateContours()
{
    std::vector<std::vector<cv::Point> > template_contours;
    for ( const std::string & template_file : m_template_files )
    {
        cv::Mat img = cv::imread(template_file);
        std::vector<std::vector<cv::Point> > contours = extractContours(img);

        std::sort(contours.begin(), contours.end(), [](std::vector<cv::Point> a, std::vector<cv::Point> b) {
            return cv::contourArea(a) > cv::contourArea(b);
        });

        template_contours.push_back(contours[0]);
    }
    return template_contours;
}

//=====================================================================================================================
std::vector<std::vector<cv::Point> > HuInvariants::extractContours(const cv::Mat& img)
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
void HuInvariants::filterContours(std::vector<std::vector<cv::Point> > & input_image_contours)
{
    int min_area = parameters.minimumArea;
    if (input_image_contours.size() > 1) {
        input_image_contours.erase(std::remove_if(input_image_contours.begin(), input_image_contours.end(),
                                                  [min_area](std::vector<cv::Point> contour) {
                                                      return cv::contourArea(contour) <= min_area;
                                                  }), input_image_contours.end());
    }
}


//=====================================================================================================================
void HuInvariants::matchTemplatesAndImage(const std::vector<std::vector<cv::Point> >& input_image_contours)
{
    std::map<std::string, std::vector<cv::Point> > templates = getTemplatesToMatch();

    if( templates.empty() == false )
    {
        m_matching_info.clear();

        std::map<std::string, std::vector<cv::Point>>::iterator it;
        for ( it = templates.begin(); it != templates.end(); ++it )
        {
            auto template_contour = it->second;
            auto template_name = it->first;

            for (const std::vector<cv::Point> &input_image_contour : input_image_contours)
            {
                auto similarity_ratio = cv::matchShapes(input_image_contour, template_contour, 1, 0.0);
                if( similarity_ratio < parameters.maximumSimilarityRatio )
                {
                    PrimitiveMatchingInfo info;
                    info.similarity_ratio = similarity_ratio;
                    info.matched_contour = input_image_contour;
                    info.primitive = ::extractFileName(template_name);

                    m_matching_info.push_back(info);
                }
            }
        }

        // Sort elements from min to max similarity
        std::sort( m_matching_info.begin(), m_matching_info.end(), [](PrimitiveMatchingInfo a, PrimitiveMatchingInfo b){
           return a.similarity_ratio < b.similarity_ratio;
        });

        // Leave only the match per primitive that has the highest similarity ratio
        std::vector<std::string> primitives;
        auto matching_info_it = m_matching_info.begin();
        while( matching_info_it != m_matching_info.end() )
        {
            if(std::find(std::begin(primitives), std::end(primitives), matching_info_it->primitive) != std::end(primitives))
            {
                matching_info_it= m_matching_info.erase(matching_info_it);
            }
            else
            {
                primitives.push_back(matching_info_it->primitive);
                matching_info_it++;
            }
        }
    }
}

//=====================================================================================================================
std::map<std::string, std::vector<cv::Point> > HuInvariants::getTemplatesToMatch()
{
    std::map<std::string, std::vector<cv::Point> > template_contours_to_match;
    std::vector<std::string> primitive_names = Converters::StringSequenceToStdVectorOfStringsConverter().Convert(inPrimitives);

    int size = m_template_files.size();
    for( unsigned int index = 0; index < size; index ++ )
    {
        for( auto primitive : primitive_names )
        {
            if( ::extractFileName(m_template_files[index]) == primitive )
            {
                template_contours_to_match.insert(std::make_pair(m_template_files[index], m_template_contours[index]));
                break;
            }
        }
    }

    return template_contours_to_match;
}

//=====================================================================================================================
void HuInvariants::ValidateParameters()
{
    ASSERT( parameters.templatesFolder.empty() == false, "Hu Invariants Configuration error: templates folder not defined");
}

//=====================================================================================================================
void HuInvariants::ValidateInputs(const cv::Mat& inputImage)
{
	ASSERT(inputImage.type() == CV_8UC3 || inputImage.type() == CV_8UC1, "Hu Invariants error: input image is not of type CV_8UC3 or CV_8UC1");
	ASSERT(inputImage.rows > 0 && inputImage.cols > 0, "Hu Invariants error: input image is empty");
}

}
}
}

/** @} */
