#ifndef KMEANS_HPP
#define KMEANS_HPP

#include <KMeans2D/KMeans2DInterface.hpp>
#include <Frame.hpp>

#include <opencv2/core/core.hpp>
#include <opencv2/imgproc/imgproc.hpp>
#include <yaml-cpp/yaml.h>
#include <Helpers/ParametersListHelper.hpp>


namespace dfn_ci {


    class Kmeans : public KMeans2DInterface
    {
        public:
            Kmeans();
            virtual ~Kmeans();
            virtual void process();
            virtual void configure();

        private:

		// calling all optionsSet
		struct KmeansOptionsSet
			{
				int nbZonesK;
			};

		Helpers::ParametersListHelper parametersHelper;
		KmeansOptionsSet parameters;
		static const KmeansOptionsSet DEFAULT_PARAMETERS;

		cv::Mat ComputeKMeans(cv::Mat inputImage, cv::Mat& KMeansImage);

		void ValidateParameters();
		void ValidateInputs(cv::Mat inputImage);

		void Configure(const YAML::Node& configurationNode);
    };
}
#endif