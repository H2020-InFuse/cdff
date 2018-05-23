#ifndef CANNYEDGEEXTRACTOR2D_HPP
#define CANNYEDGEEXTRACTOR2D_HPP

#include <EdgeMap2D/EdgeMap2DInterface.hpp>
#include <Frame.hpp>

#include <opencv2/core/core.hpp>
#include <opencv2/imgproc/imgproc.hpp>
#include <yaml-cpp/yaml.h>
#include <Helpers/ParametersListHelper.hpp>


namespace dfn_ci {


    class CannyEdgeExtractor2D : public EdgeMap2DInterface
    {
        public:
            CannyEdgeExtractor2D();
            virtual ~CannyEdgeExtractor2D();
            virtual void process();
            virtual void configure();

        private:

		// calling all optionsSet
		struct CannyOptionsSet
			{
				int kernelSize;
				int threshold;
			};

		Helpers::ParametersListHelper parametersHelper;
		CannyOptionsSet parameters;
		static const CannyOptionsSet DEFAULT_PARAMETERS;

		cv::Mat ComputeCannyEdge(cv::Mat inputImage, cv::Mat& EdgeImage);

		void ValidateParameters();
		void ValidateInputs(cv::Mat inputImage);

		void Configure(const YAML::Node& configurationNode);
    };
}
#endif