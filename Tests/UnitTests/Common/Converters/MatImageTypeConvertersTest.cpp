/* --------------------------------------------------------------------------
*
* (C) Copyright …
*
* ---------------------------------------------------------------------------
*/

/*!
 * @file MatImageTypeConvertersTest.cpp
 * @date 28/11/2017
 * @author Alessandro Bianco
 */

/*!
 * @addtogroup CommonTests
 * 
 * Testing conversion from Mat to ImageType and viceversa.
 * 
 * 
 * @{
 */

/* --------------------------------------------------------------------------
 *
 * Definitions
 * Catch definition must be before the includes, otherwise catch will not compile.
 *
 * --------------------------------------------------------------------------
 */
#define CATCH_CONFIG_MAIN


/* --------------------------------------------------------------------------
 *
 * Includes
 *
 * --------------------------------------------------------------------------
 */
#include <ImageTypeToMatConverter.hpp>
#include <MatToImageTypeConverter.hpp>
#include <ImageType.h>
#include <Catch/catch.h>
#include <Errors/Assert.hpp>

using namespace Types;

TEST_CASE( "Mat to ImageType and Back Square Matrix", "[MatToImageTypeSquare]" )
	{
	MatToImageTypeConverter firstConverter;
	ImageTypeToMatConverter secondConverter;

	cv::Mat inputMatrix(500, 500, CV_8UC3, cv::Scalar(0,0,0));
	for(int rowIndex = 0; rowIndex < inputMatrix.rows; rowIndex++)
		{
		for(int columnIndex = 0; columnIndex < inputMatrix.cols; columnIndex++)
			{
			inputMatrix.at<cv::Vec3b>(rowIndex, columnIndex)[0] = rowIndex % 256;
			inputMatrix.at<cv::Vec3b>(rowIndex, columnIndex)[1] = columnIndex % 256;
			inputMatrix.at<cv::Vec3b>(rowIndex, columnIndex)[2] = (rowIndex + columnIndex) % 256;
			}
		}

	ImageType* asnImage = firstConverter.Convert(inputMatrix);
	cv::Mat outputMatrix = secondConverter.Convert(asnImage);

	REQUIRE(outputMatrix.rows == inputMatrix.rows);
	REQUIRE(outputMatrix.cols == inputMatrix.cols);
	REQUIRE(outputMatrix.type() == inputMatrix.type());
	for(int rowIndex = 0; rowIndex < inputMatrix.rows; rowIndex++)
		{
		for(int columnIndex = 0; columnIndex < inputMatrix.cols; columnIndex++)
			{
			cv::Vec3b& outputPixel = outputMatrix.at<cv::Vec3b>(rowIndex, columnIndex);
			cv::Vec3b& inputPixel = inputMatrix.at<cv::Vec3b>(rowIndex, columnIndex);
			REQUIRE(inputPixel[0] == outputPixel[0]);	
			REQUIRE(inputPixel[1] == outputPixel[1]);			 
			REQUIRE(inputPixel[2] == outputPixel[2]);			 		 
			}
		}

	//delete [] (asnImage->data.arr);
	delete(asnImage);	
	} 

TEST_CASE( "Mat to ImageType and Back Non-Square Matrix", "[MatToImageTypeNonSquare]" )
	{
	MatToImageTypeConverter firstConverter;
	ImageTypeToMatConverter secondConverter;

	cv::Mat inputMatrix(500, 800, CV_8UC3, cv::Scalar(0,0,0));
	for(int rowIndex = 0; rowIndex < inputMatrix.rows; rowIndex++)
		{
		for(int columnIndex = 0; columnIndex < inputMatrix.cols; columnIndex++)
			{
			inputMatrix.at<cv::Vec3b>(rowIndex, columnIndex)[0] = rowIndex % 256;
			inputMatrix.at<cv::Vec3b>(rowIndex, columnIndex)[1] = columnIndex % 256;
			inputMatrix.at<cv::Vec3b>(rowIndex, columnIndex)[2] = (rowIndex + columnIndex) % 256;
			}
		}

	ImageType* asnImage = firstConverter.Convert(inputMatrix);
	cv::Mat outputMatrix = secondConverter.Convert(asnImage);

	REQUIRE(outputMatrix.rows == inputMatrix.rows);
	REQUIRE(outputMatrix.cols == inputMatrix.cols);
	REQUIRE(outputMatrix.type() == inputMatrix.type());
	for(int rowIndex = 0; rowIndex < inputMatrix.rows; rowIndex++)
		{
		for(int columnIndex = 0; columnIndex < inputMatrix.cols; columnIndex++)
			{
			cv::Vec3b& outputPixel = outputMatrix.at<cv::Vec3b>(rowIndex, columnIndex);
			cv::Vec3b& inputPixel = inputMatrix.at<cv::Vec3b>(rowIndex, columnIndex);
			REQUIRE(inputPixel[0] == outputPixel[0]);	
			REQUIRE(inputPixel[1] == outputPixel[1]);			 
			REQUIRE(inputPixel[2] == outputPixel[2]);			 		 
			}
		}

	//delete [] (asnImage->data.arr);	
	delete(asnImage);	
	} 


TEST_CASE( "Multiple conversions", "[MultipleConversions]" )
	{
	MatToImageTypeConverter firstConverter;
	ImageTypeToMatConverter secondConverter;

	cv::Mat inputMatrix(500, 800, CV_8UC3, cv::Scalar(0,0,0));
	for(int rowIndex = 0; rowIndex < inputMatrix.rows; rowIndex++)
		{
		for(int columnIndex = 0; columnIndex < inputMatrix.cols; columnIndex++)
			{
			inputMatrix.at<cv::Vec3b>(rowIndex, columnIndex)[0] = rowIndex % 256;
			inputMatrix.at<cv::Vec3b>(rowIndex, columnIndex)[1] = columnIndex % 256;
			inputMatrix.at<cv::Vec3b>(rowIndex, columnIndex)[2] = (rowIndex + columnIndex) % 256;
			}
		}

	for(int i=0; i<5; i++)
		{
		ImageType* asnImage = firstConverter.Convert(inputMatrix);
		cv::Mat outputMatrix = secondConverter.Convert(asnImage);

		REQUIRE(outputMatrix.rows == inputMatrix.rows);
		REQUIRE(outputMatrix.cols == inputMatrix.cols);
		REQUIRE(outputMatrix.type() == inputMatrix.type());
		for(int rowIndex = 0; rowIndex < inputMatrix.rows; rowIndex++)
			{
			for(int columnIndex = 0; columnIndex < inputMatrix.cols; columnIndex++)
				{
				cv::Vec3b& outputPixel = outputMatrix.at<cv::Vec3b>(rowIndex, columnIndex);
				cv::Vec3b& inputPixel = inputMatrix.at<cv::Vec3b>(rowIndex, columnIndex);
				REQUIRE(inputPixel[0] == outputPixel[0]);	
				REQUIRE(inputPixel[1] == outputPixel[1]);			 
				REQUIRE(inputPixel[2] == outputPixel[2]);			 		 
				}
			}
		
		//delete [] (asnImage->data.arr);
		delete(asnImage);
		}
	}
