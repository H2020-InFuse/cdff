/**
 * @author Irene Sanz
 */

/**
 * @addtogroup DFNs
 * @{
 */

#include "Filter.hpp"
#include <Errors/Assert.hpp>
#include <stdlib.h>
#include <fstream>

namespace CDFF
{
namespace DFN
{
namespace DepthFiltering
{

//=====================================================================================================================
Filter::Filter()
{
    parametersHelper.AddParameter<int>("GeneralParameters", "KernelSize", parameters.kernelSize, DEFAULT_PARAMETERS.kernelSize);
	configurationFilePath = "";
}

//=====================================================================================================================
const Filter::FilterOptionsSet Filter::DEFAULT_PARAMETERS =
{
    .kernelSize = 5
};

//=====================================================================================================================
void Filter::configure()
{
	parametersHelper.ReadFile(configurationFilePath);
	ValidateParameters();
}

//=====================================================================================================================
void Filter::process()
{
	// Read data from input port
	cv::Mat inputImage = frameToMat.Convert(&inFrame);

	// Process data
	ValidateInputs(inputImage);
	cv::Mat outImage = ApplyFilter(inputImage);

	// Write data to output port
    FrameWrapper::FrameConstPtr tmp = matToFrame.Convert(outImage);
    FrameWrapper::Copy(*tmp, outFrame);
	delete(tmp);
}

//=====================================================================================================================
cv::Mat Filter::ApplyFilter(cv::Mat inputImage)
{
	cv::Mat outputImage;
	cv::Mat kernel = cv::Mat::ones(parameters.kernelSize,parameters.kernelSize, CV_32F)/ (float)(parameters.kernelSize*parameters.kernelSize);

	cv::filter2D(inputImage, outputImage, -1, kernel );
	return outputImage;
}


//=====================================================================================================================
void Filter::ValidateParameters()
{
	ASSERT(parameters.kernelSize > 0, "Depth Filtering Configuration Error: kernel size should be strictly positive");
}

//=====================================================================================================================
void Filter::ValidateInputs(cv::Mat inputImage)
{
	ASSERT(inputImage.rows > 0 && inputImage.cols > 0, "Filter error: input image is empty");
}


}
}
}

/** @} */
