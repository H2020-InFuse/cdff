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
 * Testing conversion from Mat to VisualPointFeaturesVector and viceversa.
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
#include <VisualPointFeatureVector2DToMatConverter.hpp>
#include <MatToVisualPointFeatureVector2DConverter.hpp>
#include <VisualPointFeatureVector2D.hpp>
#include <Catch/catch.h>
#include <Errors/Assert.hpp>

using namespace Converters;
using namespace CppTypes;

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

	VisualPointFeatureVector2D::ConstPtr asnVector = firstConverter.Convert(inputMatrix);
	REQUIRE(asnVector->GetNumberOfPoints() == inputMatrix.rows);
	int descriptorSize = asnVector->GetNumberOfDescriptorComponents(0);
	REQUIRE(descriptorSize == inputMatrix.cols - 2);
	for(int rowIndex = 0; rowIndex < inputMatrix.rows; rowIndex++)
		{
		REQUIRE(asnVector->GetXCoordinate(rowIndex) == inputMatrix.at<float>(rowIndex, 0) );
		REQUIRE(asnVector->GetYCoordinate(rowIndex) == inputMatrix.at<float>(rowIndex, 1) );
		REQUIRE(descriptorSize == asnVector->GetNumberOfDescriptorComponents(rowIndex) );
		for(int columnIndex = 2; columnIndex < inputMatrix.cols; columnIndex++)
			{
			REQUIRE(asnVector->GetDescriptorComponent(rowIndex, columnIndex-2) == inputMatrix.at<float>(rowIndex, columnIndex) );
			}		
		}

	cv::Mat outputMatrix = secondConverter.Convert(asnVector);
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

	VisualPointFeatureVector2D::ConstPtr asnVector = firstConverter.Convert(inputMatrix);
	cv::Mat intermediateMatrix = secondConverter.Convert(asnVector);
	VisualPointFeatureVector2D::ConstPtr outputVector = firstConverter.Convert(intermediateMatrix);	

	REQUIRE(asnVector->GetNumberOfPoints() == outputVector->GetNumberOfPoints());
	REQUIRE(asnVector->GetNumberOfPoints() > 0);
	int descriptorSize = asnVector->GetNumberOfDescriptorComponents(0);
	for(int pointIndex = 0; pointIndex < asnVector->GetNumberOfPoints(); pointIndex++)
		{
		REQUIRE(asnVector->GetXCoordinate(pointIndex) == outputVector->GetXCoordinate(pointIndex) );
		REQUIRE(asnVector->GetYCoordinate(pointIndex) == outputVector->GetYCoordinate(pointIndex) );
		REQUIRE(asnVector->GetNumberOfDescriptorComponents(pointIndex) == descriptorSize );
		REQUIRE(outputVector->GetNumberOfDescriptorComponents(pointIndex) == descriptorSize );
		
		for(int componentIndex = 0; componentIndex < descriptorSize; componentIndex++)
			{
			REQUIRE(asnVector->GetDescriptorComponent(pointIndex, componentIndex) == outputVector->GetDescriptorComponent(pointIndex, componentIndex) );
			}
		}

	asnVector.reset();
	outputVector.reset();
	}

TEST_CASE( "Attempt conversion of a Bad VisualPointFeatureVector2D", "[BadVisualPointFeatureVector2D]")
	{
	VisualPointFeatureVector2D::Ptr asnVector = std::make_shared<VisualPointFeatureVector2D>();
	
	asnVector->AddPoint(0, 0);
	asnVector->AddDescriptorComponent(0, 0);

	asnVector->AddPoint(1, 1);
	asnVector->AddDescriptorComponent(1, 1);
	asnVector->AddDescriptorComponent(1, 2);

	VisualPointFeatureVector2DToMatConverter converter;
	REQUIRE_THROWS_AS( converter.Convert(asnVector), AssertException);	

	asnVector.reset();
	}

TEST_CASE( "Points with empty descriptors", "[EmptyDescriptors2D]")
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

	VisualPointFeatureVector2D::ConstPtr asnVector = firstConverter.Convert(inputMatrix);
	cv::Mat outputMatrix = secondConverter.Convert(asnVector);

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
	VisualPointFeatureVector2D::ConstPtr asnVector = firstConverter.Convert(inputMatrix);
	cv::Mat outputMatrix = secondConverter.Convert(asnVector);

	REQUIRE(outputMatrix.cols == 0);
	REQUIRE(outputMatrix.rows == 0);

	asnVector.reset();		
	}
