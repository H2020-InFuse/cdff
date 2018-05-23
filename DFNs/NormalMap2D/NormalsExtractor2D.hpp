#ifndef NORMALSEXTRACTOR2D_HPP
#define NORMALSEXTRACTOR2D_HPP

#include <NormalMap2D/NormalMap2DInterface.hpp>
#include <Frame.hpp>

#include <opencv2/core/core.hpp>
#include <opencv2/imgproc/imgproc.hpp>
#include <yaml-cpp/yaml.h>
#include <Helpers/ParametersListHelper.hpp>


namespace dfn_ci {


    class NormalsExtractor2D : public NormalMap2DInterface
    {
        public:
            NormalsExtractor2D();
            virtual ~NormalsExtractor2D();
            virtual void process();
            virtual void configure();

        private:

		// calling all optionsSet
		struct NormalOptionsSet
			{
				int scale;
			};

		Helpers::ParametersListHelper parametersHelper;
		NormalOptionsSet parameters;
		static const NormalOptionsSet DEFAULT_PARAMETERS;

		cv::Mat ComputeNormals(cv::Mat depthImage, cv::Mat& NormalsImage);

		void ValidateParameters();
		void ValidateInputs(cv::Mat depthImage);

		void Configure(const YAML::Node& configurationNode);
    };
}
#endif