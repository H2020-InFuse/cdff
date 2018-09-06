/**
 * @author Irene Sanz
 */

/**
 * @addtogroup DFNs
 * @{
 */

#include "ConvolutionFilter.hpp"
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
ConvolutionFilter::ConvolutionFilter()
{
    parametersHelper.AddParameter<int>("GeneralParameters", "KernelSize", parameters.kernelSize, DEFAULT_PARAMETERS.kernelSize);
	configurationFilePath = "";
}

//=====================================================================================================================
const ConvolutionFilter::ConvolutionFilterOptionsSet ConvolutionFilter::DEFAULT_PARAMETERS =
{
    /*.kernelSize =*/ 5
};

//=====================================================================================================================
void ConvolutionFilter::configure()
{
	parametersHelper.ReadFile(configurationFilePath);
	ValidateParameters();
}

//=====================================================================================================================
void ConvolutionFilter::process()
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
cv::Mat ConvolutionFilter::ApplyFilter(cv::Mat inputImage)
{
	cv::Mat outputImage;
	cv::Mat kernel = cv::Mat::ones(parameters.kernelSize,parameters.kernelSize, CV_32F)/ (float)(parameters.kernelSize*parameters.kernelSize);

	cv::filter2D(inputImage, outputImage, -1, kernel );
	return outputImage;
}


//=====================================================================================================================
void ConvolutionFilter::ValidateParameters()
{
	ASSERT(parameters.kernelSize > 0, "Depth ConvolutionFiltering Configuration Error: kernel size should be strictly positive");
}

//=====================================================================================================================
void ConvolutionFilter::ValidateInputs(cv::Mat inputImage)
{
	ASSERT(inputImage.rows > 0 && inputImage.cols > 0, "ConvolutionFilter error: input image is empty");
}


}
}
}

/** @} */
