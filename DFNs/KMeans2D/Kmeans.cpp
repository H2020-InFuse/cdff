/* --------------------------------------------------------------------------
 *
 * Includes
 *
 * --------------------------------------------------------------------------
 */
#include "Kmeans.hpp"
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

Kmeans::Kmeans()
{
	parametersHelper.AddParameter<int>("GeneralParameters", "nbZonesK", parameters.nbZonesK, DEFAULT_PARAMETERS.nbZonesK);
	configurationFilePath = "";
}

Kmeans::~Kmeans()
{
}

void Kmeans::configure() 
{
    parametersHelper.ReadFile(configurationFilePath);
	ValidateParameters();
}


void Kmeans::process()
{
    cv::Mat inputImage = ConversionCache<FrameConstPtr, cv::Mat, FrameToMatConverter>::Convert(inImage);
	ValidateInputs(inputImage);
	cv::Mat KMeansImage;
	
	cv::Mat kmeansImage = ComputeKMeans(inputImage, KMeansImage);

	imshow("extracted edges", kmeansImage);

}

const Kmeans::KmeansOptionsSet Kmeans::DEFAULT_PARAMETERS =
	{
	.nbZonesK = 3
	};

cv::Mat Kmeans::ComputeKMeans(cv::Mat inputImage, cv::Mat& KMeansImage)
	{

	// ----------------------- operations here ---------------------
	// convert to float & reshape to a [3 x W*H] Mat 
	//  (so every pixel is on a row of it's own)
	Mat data;
	inputImage.convertTo(data,CV_32F);
	data = data.reshape(1,data.total());

	// do kmeans
	Mat labels, centers;
	kmeans(data, parameters.nbZonesK , labels, TermCriteria(CV_TERMCRIT_ITER, 10, 1.0), 3, 
	     KMEANS_PP_CENTERS, centers);

	// reshape both to a single row of Vec3f pixels:
	centers = centers.reshape(3,centers.rows);
	data = data.reshape(3,data.rows);

	// replace pixel values with their center value:
	Vec3f *p = data.ptr<Vec3f>();
	for (size_t i=0; i< data.rows; i++) {
	 int center_id = labels.at<int>(i);
	 p[i] = centers.at<Vec3f>(center_id);
	}

	// back to 2d, and uchar:
	inputImage = data.reshape(3, inputImage.rows);
	inputImage.convertTo(KMeansImage, CV_8U);

	// ----------------------- output computed -------------------
	return KMeansImage;
	}


void Kmeans::ValidateParameters()
	{
	ASSERT
		(
		parameters.nbZonesK > 1, 
		"Kmeans error: nbZonesK should be bigger than 1"
		);
	}

void Kmeans::ValidateInputs(cv::Mat inputImage)
	{
	ASSERT(inputImage.type() == CV_8UC3 || inputImage.type() == CV_8UC1, "Kmeans error: input image is not of type CV_8UC3 or CV_8UC1");
	ASSERT(inputImage.rows > 0 && inputImage.cols > 0, "Kmeans error: input image is empty");
	}

}


/** @} */
