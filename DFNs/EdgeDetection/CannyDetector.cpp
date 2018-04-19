/* --------------------------------------------------------------------------
*
* (C) Copyright …
*
* ---------------------------------------------------------------------------
*/

/*!
 * @file CannyDetector.cpp
 * @date 11/04/2018
 * @author Nassir W. Oumer
 */

/*!
 * @addtogroup DFNs
 * 
 * CannyDetector Implementation .
 * 
 * 
 * @{
 */

/* --------------------------------------------------------------------------
 *
 * Includes
 *
 * --------------------------------------------------------------------------
 */
#include "CannyDetector.hpp"
#include <Errors/Assert.hpp>
#include <FrameToMatConverter.hpp>
#include <ConversionCache/ConversionCache.hpp>
#include <Macros/YamlcppMacros.hpp>

#include <stdlib.h>
#include <fstream>

using namespace Helpers;
using namespace Converters;
using namespace Common;

namespace dfn_ci {

using namespace FrameWrapper;

/* --------------------------------------------------------------------------
 *
 * Public Member Functions
 *
 * --------------------------------------------------------------------------
 */
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
	cv::Mat inputImage = ConversionCache<FrameConstPtr, cv::Mat, FrameToMatConverter>::Convert(inImage);
	ValidateInputs(inputImage);
	cv::Mat edges = Canny(inputImage);
	outEdgeMap = ConversionCache<cv::Mat, FrameConstPtr, MatToFrameConverter>::Convert(edges);
	}

const CannyDetector::CannyDetectorOptionsSet CannyDetector::DEFAULT_PARAMETERS =
	{
	.cannyParameters =
		{
		.lowThreshold =50.0,
		.highThreshold=80.0,
		.kernelSize = 3,
		
		}
	
	};


cv::Mat CannyDetector::Canny(cv::Mat inputImage)
	{
	
	double t1,t2;
	int k_size;

	t1= (double)parameters.cannyParameters.lowThreshold;
	t2= (double)parameters.cannyParameters.highThreshold;
	k_size=parameters.cannyParameters.kernelSize;

	cv::Mat edges;
	cv::Canny(inputImage,edges, t1, t2, k_size );
	return edges;
	}


void CannyDetector::ValidateParameters()
	{
	ASSERT(parameters.cannyParameters.lowThreshold > 0 && parameters.cannyParameters.highThreshold > 0, "Canny Detector Configuration error: thresholds have to be positive");
	
	}

void CannyDetector::ValidateInputs(cv::Mat inputImage)
	{
	ASSERT(inputImage.type() == CV_8UC1, "Canny Detector error: input image is not of type CV_8UC1");
	ASSERT(inputImage.rows > 0 && inputImage.cols > 0, "Canny Detector error: input image is empty");
	}

}


/** @} */
