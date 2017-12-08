/* --------------------------------------------------------------------------
*
* (C) Copyright …
*
* ---------------------------------------------------------------------------
*/

/*!
 * @file MatFrameConvertersTest.cpp
 * @date 01/12/2017
 * @author Alessandro Bianco
 */

/*!
 * @addtogroup CommonTests
 * 
 * Testing conversion from Mat to Frame and viceversa.
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
#include <FrameToMatConverter.hpp>
#include <MatToFrameConverter.hpp>
#include <Frame.hpp>
#include <Catch/catch.h>
#include <Errors/Assert.hpp>

using namespace Converters;

TEST_CASE( "Mat to Frame and Back Square Matrix", "[MatToFrameSquare]" )
	{
	MatToFrameConverter firstConverter;
	FrameToMatConverter secondConverter;

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

	CppTypes::Frame::ConstPtr asnFrame = firstConverter.Convert(inputMatrix);
	cv::Mat outputMatrix = secondConverter.Convert(asnFrame);

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

	asnFrame.reset();
	} 

TEST_CASE( "Mat to Frame and Back Non-Square Matrix", "[MatToFrameNonSquare]" )
	{
	MatToFrameConverter firstConverter;
	FrameToMatConverter secondConverter;

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

	CppTypes::Frame::ConstPtr asnFrame = firstConverter.Convert(inputMatrix);
	cv::Mat outputMatrix = secondConverter.Convert(asnFrame);

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

	asnFrame.reset();
	} 


TEST_CASE( "Multiple conversions", "[MultipleConversions]" )
	{
	MatToFrameConverter firstConverter;
	FrameToMatConverter secondConverter;

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
		CppTypes::Frame::ConstPtr asnFrame = firstConverter.Convert(inputMatrix);
		cv::Mat outputMatrix = secondConverter.Convert(asnFrame);

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
		
		asnFrame.reset();
		}
	}


TEST_CASE( "Frame to Mat conversion", "[FrameToMatConversion]")
	{
	MatToFrameConverter firstConverter;
	FrameToMatConverter secondConverter;

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

	CppTypes::Frame::ConstPtr asnFrame = firstConverter.Convert(inputMatrix);
	cv::Mat outputMatrix = secondConverter.Convert(asnFrame);
	CppTypes::Frame::ConstPtr outputFrame = firstConverter.Convert(outputMatrix);

	REQUIRE(outputFrame->GetFrameWidth() == asnFrame->GetFrameWidth());
	REQUIRE(outputFrame->GetFrameHeight() == asnFrame->GetFrameHeight());
	REQUIRE(outputFrame->GetFrameMode() == asnFrame->GetFrameMode());
	REQUIRE(outputFrame->GetNumberOfDataBytes() == asnFrame->GetNumberOfDataBytes());
	for(int byteIndex = 0; byteIndex < asnFrame->GetNumberOfDataBytes(); byteIndex++)
		{
		REQUIRE(outputFrame->GetDataByte(byteIndex) == asnFrame->GetDataByte(byteIndex));				 		 
		}

	asnFrame.reset();
	outputFrame.reset();
	}
