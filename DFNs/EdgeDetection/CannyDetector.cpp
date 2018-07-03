/**
 * @author Nassir W. Oumer
 */

/**
 * @addtogroup DFNs
 * @{
 */

#include "CannyDetector.hpp"
#include <Frame.hpp>
#include <Errors/Assert.hpp>

#include <opencv2/imgproc/imgproc.hpp>
#include <opencv2/imgcodecs/imgcodecs.hpp>

using namespace FrameWrapper;
using namespace Converters;
using namespace Helpers;

namespace dfn_ci
{

CannyDetector::CannyDetector()
{
	parametersHelper.AddParameter<float>("CannyParameters", "LowThreshold", parameters.cannyParameters.lowThreshold, DEFAULT_PARAMETERS.cannyParameters.lowThreshold);
	parametersHelper.AddParameter<float>("CannyParameters", "HighThreshold", parameters.cannyParameters.highThreshold, DEFAULT_PARAMETERS.cannyParameters.highThreshold);
	parametersHelper.AddParameter<float>("CannyParameters", "KernelSize", parameters.cannyParameters.kernelSize, DEFAULT_PARAMETERS.cannyParameters.kernelSize);

	configurationFilePath = "";
}

CannyDetector::~CannyDetector()
{
}

void CannyDetector::configure()
{
	parametersHelper.ReadFile(configurationFilePath);
	ValidateParameters();
}

void CannyDetector::process()
{
	// Read data from input port
	cv::Mat inputImage = frameToMat.Convert(&inImage);
	ValidateInput(inputImage);

	// Process data
	cv::Mat edges = Canny(inputImage);

	// Write data to output port
	const Frame* tmp = matToFrame.Convert(edges);
	Copy(*tmp, outEdgeMap);
	delete tmp;
}

const CannyDetector::CannyDetectorOptionsSet CannyDetector::DEFAULT_PARAMETERS =
{
	.cannyParameters =
	{
		.lowThreshold  = 50.0,
		.highThreshold = 80.0,
		.kernelSize    = 3
	}
};

cv::Mat CannyDetector::Canny(cv::Mat inputImage)
{
	double t1, t2;
	int k_size;

	t1 = (double)parameters.cannyParameters.lowThreshold;
	t2 = (double)parameters.cannyParameters.highThreshold;
	k_size = parameters.cannyParameters.kernelSize;

	cv::Mat edges;
	cv::Canny(inputImage, edges, t1, t2, k_size);

	return edges;
}

void CannyDetector::ValidateParameters()
{
	ASSERT(parameters.cannyParameters.lowThreshold > 0 && parameters.cannyParameters.highThreshold > 0,
		"CannyDetector: configuration error: thresholds must be strictly positive");
	ASSERT(parameters.cannyParameters.lowThreshold < parameters.cannyParameters.highThreshold,
		"CannyDetector: configuration error: lower threshold must be lower than higher threshold");
}

void CannyDetector::ValidateInput(cv::Mat inputImage)
{
	ASSERT(inputImage.type() == CV_8UC1,
		"CannyDetector: error: input image must be of type CV_8UC1");
	ASSERT(inputImage.rows > 0 && inputImage.cols > 0,
		"CannyDetector: error: input image cannot be empty");
}

}

/** @} */
