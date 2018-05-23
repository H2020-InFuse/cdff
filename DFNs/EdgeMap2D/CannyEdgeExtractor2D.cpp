/* --------------------------------------------------------------------------
 *
 * Includes
 *
 * --------------------------------------------------------------------------
 */
#include "CannyEdgeExtractor2D.hpp"
#include <Errors/Assert.hpp>
#include <FrameToMatConverter.hpp>
#include <MatToFrameConverter.hpp>
#include <Macros/YamlcppMacros.hpp>
#include <ConversionCache/ConversionCache.hpp>
#include <iostream>

#include <opencv2/core/core.hpp>
#include <opencv2/highgui/highgui.hpp>

#include <stdlib.h>
#include <fstream>

using namespace Helpers;
using namespace Converters;
using namespace Common;
using namespace cv;
using namespace std;

namespace dfn_ci {

using namespace FrameWrapper;

CannyEdgeExtractor2D::CannyEdgeExtractor2D()
{
	parametersHelper.AddParameter<int>("GeneralParameters", "kernelSize", parameters.kernelSize, DEFAULT_PARAMETERS.kernelSize);
	parametersHelper.AddParameter<int>("GeneralParameters", "threshold", parameters.threshold, DEFAULT_PARAMETERS.threshold);
	configurationFilePath = "";
}

CannyEdgeExtractor2D::~CannyEdgeExtractor2D()
{
}

void CannyEdgeExtractor2D::configure() 
{
    parametersHelper.ReadFile(configurationFilePath);
	ValidateParameters();
}


void CannyEdgeExtractor2D::process()
{
    cv::Mat inputImage = ConversionCache<FrameConstPtr, cv::Mat, FrameToMatConverter>::Convert(inImage);
	ValidateInputs(inputImage);
	cv::Mat EdgeImage;
	
	cv::Mat cannyImage = ComputeCannyEdge(inputImage, EdgeImage);

	imshow("extracted edges", cannyImage);

}

const CannyEdgeExtractor2D::CannyOptionsSet CannyEdgeExtractor2D::DEFAULT_PARAMETERS =
	{
	.kernelSize = 3,
	.threshold = 90
	};

cv::Mat CannyEdgeExtractor2D::ComputeCannyEdge(cv::Mat inputImage, cv::Mat& EdgeImage)
	{

	cv::Mat grayImage;
	cv::cvtColor(inputImage, grayImage, CV_BGR2GRAY);

	// ----------------------- operations here ---------------------
	cv::Mat cannyImage = grayImage;

	// init 
	cv::Mat detected_edges;

	// Reduce noise with a kernel 3x3
	blur( cannyImage, detected_edges, Size(parameters.kernelSize,parameters.kernelSize) );

	// Canny detector
	Canny( detected_edges, detected_edges, parameters.threshold, 3*parameters.threshold, parameters.kernelSize );

	// Create a matrix of the same type and size as img (for canny_img)
	EdgeImage.create( cannyImage.size(), cannyImage.type() );

	// Using Canny's output as a mask, we display our result
	EdgeImage = Scalar::all(0);

	cannyImage.copyTo(EdgeImage, detected_edges);	

	// ----------------------- output computed -------------------
	return EdgeImage;
	}


void CannyEdgeExtractor2D::ValidateParameters()
	{
	ASSERT
		(
		parameters.kernelSize > 0 || parameters.threshold > 0, 
		"Hough Extraction Configuration error: rho and theta step sizes should be positive"
		);
	}

void CannyEdgeExtractor2D::ValidateInputs(cv::Mat inputImage)
	{
	ASSERT(inputImage.type() == CV_8UC3 || inputImage.type() == CV_8UC1, "CannyEdgeExtractor2D error: input image is not of type CV_8UC3 or CV_8UC1");
	ASSERT(inputImage.rows > 0 && inputImage.cols > 0, "CannyEdgeExtractor2D error: input image is empty");
	}

}


/** @} */
