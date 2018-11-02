/**
 * @author Irene Sanz
 */

/**
 * @addtogroup DFNs
 * @{
 */

#ifndef PRIMITIVEMATCHING_HUINVARIANTS_HPP
#define PRIMITIVEMATCHING_HUINVARIANTS_HPP

#include "PrimitiveMatchingInterface.hpp"

#include <Frame.hpp>
#include <Converters/FrameToMatConverter.hpp>
#include <Converters/MatToFrameConverter.hpp>
#include <Helpers/ParametersListHelper.hpp>
#include <Converters/StringSequenceToStdVectorOfStringsConverter.hpp>
#include <Converters/StdVectorOfStringsToStringSequenceConverter.hpp>

#include <opencv2/core/core.hpp>
#include <opencv2/imgproc/imgproc.hpp>
#include <yaml-cpp/yaml.h>

namespace CDFF
{
namespace DFN
{
namespace PrimitiveMatching
{

    typedef struct
    {
        std::string primitive;
        double similarity_ratio;
        std::vector<cv::Point> matched_contour;
    } PrimitiveMatchingInfo;


    /**
	 * Match primitives using HU Invariants.
	 *
	 * @param minimumArea
	 *        minimum area that the contours need to have to be considered for primitive matching
	 * @param templatesFolder
	 *        folder where the primitive templates are located
	 */
	class HuInvariants : public PrimitiveMatchingInterface
	{
		public:

			HuInvariants();
			virtual ~HuInvariants() = default;

			virtual void configure();
			virtual void process();

	private:

		struct HuInvariantsOptionsSet
		{
			int minimumArea;
			std::string templatesFolder;
            double maximumSimilarityRatio;
        };

		Helpers::ParametersListHelper parametersHelper;
		HuInvariantsOptionsSet parameters;
		static const HuInvariantsOptionsSet DEFAULT_PARAMETERS;

		Converters::FrameToMatConverter frameToMat;
        Converters::MatToFrameConverter matToFrame;

        std::vector< std::string > Match(const cv::Mat& inputImage);

		void ValidateParameters();
		void ValidateInputs(const cv::Mat& inputImage);

        std::vector<std::vector<cv::Point> > extractContours(const cv::Mat& img);
        std::vector<std::vector<cv::Point> > getTemplateContours();
        void filterContours(std::vector<std::vector<cv::Point> > & input_image_contours);
        void matchTemplatesAndImage(const std::vector<std::vector<cv::Point> >& input_image_contours);
        std::map<std::string, std::vector<cv::Point> > getTemplatesToMatch();
        cv::Mat drawContoursAndInformationOnOutputImage(const cv::Mat& inputImage);


        std::vector<PrimitiveMatchingInfo> m_matching_info;
		std::vector<std::string> m_template_files;
		std::vector<std::vector<cv::Point> > m_template_contours;
    };
}
}
}

#endif // PRIMITIVEMATCHING_HUINVARIANTS_HPP

/** @} */
