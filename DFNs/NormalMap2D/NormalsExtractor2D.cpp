/* --------------------------------------------------------------------------
 *
 * Includes
 *
 * --------------------------------------------------------------------------
 */
#include "NormalsExtractor2D.hpp"
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

NormalsExtractor2D::NormalsExtractor2D()
{
	parametersHelper.AddParameter<int>("GeneralParameters", "scale", parameters.scale, DEFAULT_PARAMETERS.scale);
	configurationFilePath = "";
}

NormalsExtractor2D::~NormalsExtractor2D()
{
}

void NormalsExtractor2D::configure() 
{
    parametersHelper.ReadFile(configurationFilePath);
	ValidateParameters();
}


void NormalsExtractor2D::process()
{
    cv::Mat depthImage = ConversionCache<FrameConstPtr, cv::Mat, FrameToMatConverter>::Convert(inImage);
	ValidateInputs(depthImage);
	cv::Mat NormalsImage;
	
	cv::Mat normalsImage = ComputeNormals(depthImage, NormalsImage);

	imshow("Extracted normals", normalsImage);

}

const NormalsExtractor2D::NormalOptionsSet NormalsExtractor2D::DEFAULT_PARAMETERS =
	{
	 .scale = 1,
	};

cv::Mat NormalsExtractor2D::ComputeNormals(cv::Mat depthImage, cv::Mat& NormalsImage)
	{

	// ----- greyscale it
	cvtColor(depthImage, depthImage, COLOR_BGR2GRAY);

	// ----------------------- operations here ---------------------
	if(depthImage.type() != CV_32FC1)
		        depthImage.convertTo(depthImage, CV_32FC1);

	Mat Init(depthImage.size(), CV_32FC3);
	NormalsImage = Init;

	for(int x = 1; x < depthImage.rows-1; ++x)
	{
	    for(int y = 1; y < depthImage.cols-1; ++y)
	    {
	        // use float instead of double otherwise you will not get the correct result
	        // check my updates in the original post. I have not figure out yet why this
	        // is happening.
	        float dzdx = (depthImage.at<float>(x+1, y) - depthImage.at<float>(x-1, y)) / 2.0;
	        float dzdy = (depthImage.at<float>(x, y+1) - depthImage.at<float>(x, y-1)) / 2.0;

	        Vec3f d(-dzdx, -dzdy, 1.0f);

	        Vec3f n = normalize(d);
	        NormalsImage.at<Vec3f>(x, y) = n;
	    }
	}

	NormalsImage.convertTo(NormalsImage, CV_32FC3);

	// ----------------------- output computed -------------------
	return NormalsImage;
	}


void NormalsExtractor2D::ValidateParameters()
	{
	// ASSERT
	// 	(
	// 	parameters.scale > 0
	// 	"Error, parameter should be positive"
	// 	);
	}

void NormalsExtractor2D::ValidateInputs(cv::Mat depthImage)
	{
	ASSERT(depthImage.type() == CV_8UC3 || depthImage.type() == CV_8UC1, "NormalsExtractor2D error: input image is not of type CV_8UC3 or CV_8UC1");
	ASSERT(depthImage.rows > 0 && depthImage.cols > 0, "NormalsExtractor2D error: input image is empty");
	}

}


/** @} */
