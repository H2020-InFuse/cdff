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
#include <FrameToMatConverter.hpp>
#include <MatToFrameConverter.hpp>
#include <Helpers/ParametersListHelper.hpp>

#include <opencv2/core/core.hpp>
#include <opencv2/imgproc/imgproc.hpp>
#include <yaml-cpp/yaml.h>

namespace CDFF
{
namespace DFN
{
namespace PrimitiveMatching
{
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
		};

		Helpers::ParametersListHelper parametersHelper;
		HuInvariantsOptionsSet parameters;
		static const HuInvariantsOptionsSet DEFAULT_PARAMETERS;

		Converters::FrameToMatConverter frameToMat;
        Converters::MatToFrameConverter matToFrame;

        std::string Match(const cv::Mat& inputImage);

		void ValidateParameters();
		void ValidateInputs(const cv::Mat& inputImage);

		void Configure(const YAML::Node& configurationNode);

        std::vector<std::vector<cv::Point> > extractContours(const cv::Mat& img);
        std::vector<std::string> getTemplateFiles();
        std::vector<std::vector<cv::Point> > getTemplateContours();
        std::vector<std::vector<cv::Point> > extractAndFilterContours(const cv::Mat& img);
        std::string matchTemplatesAndImage(const std::vector<std::vector<cv::Point> >& input_image_contours);


        std::vector< std::vector<cv::Point> > m_matched_contour;
		std::vector<std::string> m_template_files;
		std::vector<std::vector<cv::Point> > m_template_contours;
    };
}
}
}

#endif // PRIMITIVEMATCHING_HUINVARIANTS_HPP

/** @} */
