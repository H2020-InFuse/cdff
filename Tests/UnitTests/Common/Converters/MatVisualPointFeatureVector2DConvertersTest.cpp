/* --------------------------------------------------------------------------
*
* (C) Copyright …
*
* ---------------------------------------------------------------------------
*/

/*!
 * @file MatVisualPointFeatureVector2DTest.cpp
 * @date 28/11/2017
 * @author Alessandro Bianco
 */

/*!
 * @addtogroup CommonTests
 * 
 * Testing conversion from Mat to VisualPointFeaturesVector2D and viceversa.
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
#include <VisualPointFeatureVector2DToMatConverter.hpp>
#include <MatToVisualPointFeatureVector2DConverter.hpp>
#include <VisualPointFeatureVector2D.hpp>
#include <Errors/Assert.hpp>

using namespace Converters;
using namespace VisualPointFeatureVector2DWrapper;

TEST_CASE( "Mat to VisualPointFeatureVector2D", "[MatToVisualPointFeatureVector2D]" )
	{
	MatToVisualPointFeatureVector2DConverter firstConverter;
	VisualPointFeatureVector2DToMatConverter secondConverter;

	cv::Mat inputMatrix(100, 4, CV_32FC1, cv::Scalar(0));
	for(int rowIndex = 0; rowIndex < inputMatrix.rows; rowIndex++)
		{
		for(int columnIndex = 0; columnIndex < inputMatrix.cols; columnIndex++)
			{
			inputMatrix.at<float>(rowIndex, columnIndex) = (float) (rowIndex + columnIndex);
			}
		}

	VisualPointFeatureVector2DSharedConstPtr asnVector = firstConverter.ConvertShared(inputMatrix);
	REQUIRE(GetNumberOfPoints(*asnVector) == inputMatrix.rows);
	int descriptorSize = GetNumberOfDescriptorComponents(*asnVector, 0);
	REQUIRE(descriptorSize == inputMatrix.cols - 2);
	for(int rowIndex = 0; rowIndex < inputMatrix.rows; rowIndex++)
		{
		REQUIRE(GetXCoordinate(*asnVector, rowIndex) == inputMatrix.at<float>(rowIndex, 0) );
		REQUIRE(GetYCoordinate(*asnVector, rowIndex) == inputMatrix.at<float>(rowIndex, 1) );
		REQUIRE(descriptorSize == GetNumberOfDescriptorComponents(*asnVector, rowIndex) );
		for(int columnIndex = 2; columnIndex < inputMatrix.cols; columnIndex++)
			{
			REQUIRE(GetDescriptorComponent(*asnVector, rowIndex, columnIndex-2) == inputMatrix.at<float>(rowIndex, columnIndex) );
			}		
		}

	cv::Mat outputMatrix = secondConverter.ConvertShared(asnVector);
	REQUIRE(outputMatrix.rows == inputMatrix.rows);
	REQUIRE(outputMatrix.cols == inputMatrix.cols);
	REQUIRE(outputMatrix.type() == inputMatrix.type());
	for(int rowIndex = 0; rowIndex < inputMatrix.rows; rowIndex++)
		{
		for(int columnIndex = 0; columnIndex < inputMatrix.cols; columnIndex++)
			{
			REQUIRE(outputMatrix.at<float>(rowIndex, columnIndex) == inputMatrix.at<float>(rowIndex, columnIndex));		 		 
			}
		}	

	asnVector.reset();	
	} 


TEST_CASE( "VisualPointFeatureVector2D to Mat", "[VisualPointFeatureVector2DToMat]" )
	{
	MatToVisualPointFeatureVector2DConverter firstConverter;
	VisualPointFeatureVector2DToMatConverter secondConverter;

	cv::Mat inputMatrix(100, 4, CV_32FC1, cv::Scalar(0));
	for(int rowIndex = 0; rowIndex < inputMatrix.rows; rowIndex++)
		{
		for(int columnIndex = 0; columnIndex < inputMatrix.cols; columnIndex++)
			{
			inputMatrix.at<float>(rowIndex, columnIndex) = (float) (rowIndex + columnIndex);
			}
		}

	VisualPointFeatureVector2DSharedConstPtr asnVector = firstConverter.ConvertShared(inputMatrix);
	cv::Mat intermediateMatrix = secondConverter.ConvertShared(asnVector);
	VisualPointFeatureVector2DSharedConstPtr outputVector = firstConverter.ConvertShared(intermediateMatrix);	

	REQUIRE(GetNumberOfPoints(*asnVector) == GetNumberOfPoints(*outputVector));
	REQUIRE(GetNumberOfPoints(*asnVector) > 0);
	int descriptorSize = GetNumberOfDescriptorComponents(*asnVector, 0);
	for(int pointIndex = 0; pointIndex < GetNumberOfPoints(*asnVector); pointIndex++)
		{
		REQUIRE(GetXCoordinate(*asnVector, pointIndex) == GetXCoordinate(*outputVector, pointIndex) );
		REQUIRE(GetYCoordinate(*asnVector, pointIndex) == GetYCoordinate(*outputVector, pointIndex) );
		REQUIRE(GetNumberOfDescriptorComponents(*asnVector, pointIndex) == descriptorSize );
		REQUIRE(GetNumberOfDescriptorComponents(*outputVector, pointIndex) == descriptorSize );
		
		for(int componentIndex = 0; componentIndex < descriptorSize; componentIndex++)
			{
			REQUIRE(GetDescriptorComponent(*asnVector, pointIndex, componentIndex) == GetDescriptorComponent(*outputVector, pointIndex, componentIndex) );
			}
		}

	asnVector.reset();
	outputVector.reset();
	}

TEST_CASE( "Attempt conversion of a Bad VisualPointFeatureVector2D", "[BadVisualPointFeatureVector2D]")
	{
	VisualPointFeatureVector2DSharedPtr asnVector = std::make_shared<VisualPointFeatureVector2D>();
	
	AddPoint(*asnVector, 0, 0);
	AddDescriptorComponent(*asnVector, 0, 0);

	AddPoint(*asnVector, 1, 1);
	AddDescriptorComponent(*asnVector, 1, 1);
	AddDescriptorComponent(*asnVector, 1, 2);

	VisualPointFeatureVector2DToMatConverter converter;
	REQUIRE_THROWS_AS( converter.ConvertShared(asnVector), AssertException);	

	asnVector.reset();
	}

TEST_CASE( "2D points with empty descriptors", "[EmptyDescriptors2D]")
	{
	MatToVisualPointFeatureVector2DConverter firstConverter;
	VisualPointFeatureVector2DToMatConverter secondConverter;

	cv::Mat inputMatrix(100, 2, CV_32FC1, cv::Scalar(0));
	for(int rowIndex = 0; rowIndex < inputMatrix.rows; rowIndex++)
		{
		for(int columnIndex = 0; columnIndex < inputMatrix.cols; columnIndex++)
			{
			inputMatrix.at<float>(rowIndex, columnIndex) = (float) (rowIndex + columnIndex);
			}
		}

	VisualPointFeatureVector2DSharedConstPtr asnVector = firstConverter.ConvertShared(inputMatrix);
	cv::Mat outputMatrix = secondConverter.ConvertShared(asnVector);

	REQUIRE(outputMatrix.rows == inputMatrix.rows);
	REQUIRE(outputMatrix.cols == inputMatrix.cols);
	REQUIRE(outputMatrix.type() == inputMatrix.type());
	for(int rowIndex = 0; rowIndex < inputMatrix.rows; rowIndex++)
	{
		for(int columnIndex = 0; columnIndex < inputMatrix.cols; columnIndex++)
			{
			REQUIRE(outputMatrix.at<float>(rowIndex, columnIndex) == inputMatrix.at<float>(rowIndex, columnIndex));		 		 
			}
		}	

	asnVector.reset();
	}

TEST_CASE("Empty Features Vector 2D", "[EmptyFeaturesVector2D]")
	{
	MatToVisualPointFeatureVector2DConverter firstConverter;
	VisualPointFeatureVector2DToMatConverter secondConverter;

	cv::Mat inputMatrix;
	VisualPointFeatureVector2DSharedConstPtr asnVector = firstConverter.ConvertShared(inputMatrix);
	cv::Mat outputMatrix = secondConverter.ConvertShared(asnVector);

	REQUIRE(outputMatrix.cols == 0);
	REQUIRE(outputMatrix.rows == 0);

	asnVector.reset();		
	}
