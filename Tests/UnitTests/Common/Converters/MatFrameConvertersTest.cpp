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
 * Includes
 *
 * --------------------------------------------------------------------------
 */
#include <catch.hpp>
#include <FrameToMatConverter.hpp>
#include <MatToFrameConverter.hpp>
#include <Frame.hpp>
#include <Errors/Assert.hpp>

using namespace Converters;
using namespace FrameWrapper;

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

	FrameSharedConstPtr asnFrame = firstConverter.ConvertShared(inputMatrix);
	REQUIRE(GetFrameWidth(*asnFrame) == static_cast<int>(inputMatrix.cols) );
	REQUIRE(GetFrameHeight(*asnFrame) == static_cast<int>(inputMatrix.rows) );
	REQUIRE(GetFrameMode(*asnFrame) == FrameWrapper::MODE_RGB );
	REQUIRE(GetNumberOfDataBytes(*asnFrame) == inputMatrix.cols * inputMatrix.rows * 3);
	for(int byteIndex = 0; byteIndex < GetNumberOfDataBytes(*asnFrame); byteIndex += 3)
		{
		int rowIndex = (byteIndex / 3) / inputMatrix.cols;
		int columnIndex = (byteIndex / 3) % inputMatrix.cols;
		REQUIRE(GetDataByte(*asnFrame, byteIndex) == inputMatrix.at<cv::Vec3b>(rowIndex, columnIndex)[0] );
		REQUIRE(GetDataByte(*asnFrame, byteIndex+1) == inputMatrix.at<cv::Vec3b>(rowIndex, columnIndex)[1] );				 		 
		REQUIRE(GetDataByte(*asnFrame, byteIndex+2) == inputMatrix.at<cv::Vec3b>(rowIndex, columnIndex)[2] );
		}

	cv::Mat outputMatrix = secondConverter.ConvertShared(asnFrame);
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

	FrameSharedConstPtr asnFrame = firstConverter.ConvertShared(inputMatrix);
	cv::Mat outputMatrix = secondConverter.ConvertShared(asnFrame);

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
		FrameSharedConstPtr asnFrame = firstConverter.ConvertShared(inputMatrix);
		cv::Mat outputMatrix = secondConverter.ConvertShared(asnFrame);

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

	FrameSharedConstPtr asnFrame = firstConverter.ConvertShared(inputMatrix);
	cv::Mat outputMatrix = secondConverter.ConvertShared(asnFrame);
	FrameSharedConstPtr outputFrame = firstConverter.ConvertShared(outputMatrix);

	REQUIRE(GetFrameWidth(*outputFrame) == GetFrameWidth(*asnFrame));
	REQUIRE(GetFrameHeight(*outputFrame) == GetFrameHeight(*asnFrame));
	REQUIRE(GetFrameMode(*outputFrame) == GetFrameMode(*asnFrame));
	REQUIRE(GetNumberOfDataBytes(*outputFrame) == GetNumberOfDataBytes(*asnFrame));
	for(int byteIndex = 0; byteIndex < GetNumberOfDataBytes(*asnFrame); byteIndex++)
		{
		REQUIRE(GetDataByte(*outputFrame, byteIndex) == GetDataByte(*asnFrame, byteIndex));				 		 
		}

	asnFrame.reset();
	outputFrame.reset();
	}



TEST_CASE( "Empty Matrix Conversion", "[EmptyMatrix]" )
	{
	MatToFrameConverter firstConverter;
	FrameToMatConverter secondConverter;

	cv::Mat inputMatrix;
	FrameSharedConstPtr asnFrame = firstConverter.ConvertShared(inputMatrix);
	cv::Mat outputMatrix = secondConverter.ConvertShared(asnFrame);
	FrameSharedConstPtr outputFrame = firstConverter.ConvertShared(outputMatrix);

       //TODO: debug if  the real problem is with Catch Bug
	//REQUIRE(outputMatrix.cols == 0);
	//REQUIRE(outputMatrix.rows == 0);
        //REQUIRE(GetNumberOfDataBytes(*asnFrame) == 0);
	//REQUIRE(GetNumberOfDataBytes(*outputFrame) == 0);	
	}


TEST_CASE( "Mat to Frame Gray scale", "[MatToFrameGrayScale]" )
	{
	MatToFrameConverter firstConverter;
	FrameToMatConverter secondConverter;

	cv::Mat inputMatrix(500, 500, CV_8UC1, cv::Scalar(0));
	for(int rowIndex = 0; rowIndex < inputMatrix.rows; rowIndex++)
		{
		for(int columnIndex = 0; columnIndex < inputMatrix.cols; columnIndex++)
			{
			inputMatrix.at<uchar>(rowIndex, columnIndex) = rowIndex % 256;
			
			}
		}

	FrameSharedConstPtr asnFrame = firstConverter.ConvertShared(inputMatrix);
	REQUIRE(GetFrameWidth(*asnFrame) == static_cast<int>(inputMatrix.cols) );
	REQUIRE(GetFrameHeight(*asnFrame) == static_cast<int>(inputMatrix.rows) );
	REQUIRE(GetFrameMode(*asnFrame) == FrameWrapper::MODE_GRAYSCALE);
	REQUIRE(GetNumberOfDataBytes(*asnFrame) == inputMatrix.cols * inputMatrix.rows);
	for(int byteIndex = 0; byteIndex < GetNumberOfDataBytes(*asnFrame); byteIndex += 1)
		{
		int rowIndex = (byteIndex) / inputMatrix.cols;
		int columnIndex = (byteIndex) % inputMatrix.cols;
		REQUIRE(GetDataByte(*asnFrame, byteIndex) == inputMatrix.at<uchar>(rowIndex, columnIndex));
		}

	cv::Mat outputMatrix = secondConverter.ConvertShared(asnFrame);
	REQUIRE(outputMatrix.rows == inputMatrix.rows);
	REQUIRE(outputMatrix.cols == inputMatrix.cols);
	REQUIRE(outputMatrix.type() == inputMatrix.type());
	for(int rowIndex = 0; rowIndex < inputMatrix.rows; rowIndex++)
		{
		for(int columnIndex = 0; columnIndex < inputMatrix.cols; columnIndex++)
			{
			int outputPixel = outputMatrix.at<uchar>(rowIndex, columnIndex);
			int inputPixel = inputMatrix.at<uchar>(rowIndex, columnIndex);
			REQUIRE(inputPixel == outputPixel);	
					 		 
			}
		}

	asnFrame.reset();
	} 
