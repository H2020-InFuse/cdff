/* --------------------------------------------------------------------------
*
* (C) Copyright …
*
* ---------------------------------------------------------------------------
*/

/*!
 * @file FundamentalMatrixRansac.cpp
 * @date 26/01/2018
 * @author Alessandro Bianco
 */

/*!
 * @addtogroup DFNs
 * 
 * Implementation of the Ransac Method for fundamental matrix computation class.
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
#include "FundamentalMatrixRansac.hpp"
#include <Errors/Assert.hpp>
#include <ConversionCache/ConversionCache.hpp>
#include <Macros/YamlcppMacros.hpp>
#include "opencv2/calib3d.hpp"

#include <stdlib.h>
#include <fstream>

using namespace Common;

namespace dfn_ci {

using namespace MatrixWrapper;
using namespace CorrespondenceMap2DWrapper;

/* --------------------------------------------------------------------------
 *
 * Public Member Functions
 *
 * --------------------------------------------------------------------------
 */
FundamentalMatrixRansac::FundamentalMatrixRansac()
	{
	parametersHelper.AddParameter<double>("GeneralParameters", "OutlierThreshold", parameters.outlierThreshold, DEFAULT_PARAMETERS.outlierThreshold);
	parametersHelper.AddParameter<double>("GeneralParameters", "Confidence", parameters.confidence, DEFAULT_PARAMETERS.confidence);

	configurationFilePath = "";
	}

FundamentalMatrixRansac::~FundamentalMatrixRansac()
	{

	}

void FundamentalMatrixRansac::configure()
	{
	parametersHelper.ReadFile(configurationFilePath);
	ValidateParameters();
	}


void FundamentalMatrixRansac::process() 
	{
	std::vector<cv::Point2d> firstImagePointsVector;
	std::vector<cv::Point2d> secondImagePointsVector;
	Convert(inCorrespondenceMap, firstImagePointsVector, secondImagePointsVector);
	ValidateInputs(firstImagePointsVector, secondImagePointsVector);

	cv::Mat fundamentalMatrix = ComputeFundamentalMatrix(firstImagePointsVector, secondImagePointsVector);
	
	if (fundamentalMatrix.rows == 0 && fundamentalMatrix.cols == 0)
		{
		outFundamentalMatrix =  NewMatrix3d();
		outSuccess = false;
		}	
	else
		{
		outFundamentalMatrix = Convert(fundamentalMatrix);
		outSuccess = true;
		}
	}

void FundamentalMatrixRansac::Convert(CorrespondenceMap2DConstPtr correspondenceMap, std::vector<cv::Point2d>& firstImagePointsVector, std::vector<cv::Point2d>& secondImagePointsVector)
	{
	for(int correspondenceIndex = 0; correspondenceIndex < GetNumberOfCorrespondences(*correspondenceMap); correspondenceIndex++)
		{
		BaseTypesWrapper::Point2D firstPoint = GetSource(*correspondenceMap, correspondenceIndex);
		BaseTypesWrapper::Point2D secondPoint = GetSink(*correspondenceMap, correspondenceIndex);
		cv::Point2d firstCvPoint(firstPoint.x, firstPoint.y);
		cv::Point2d secondCvPoint(secondPoint.x, secondPoint.y);
		firstImagePointsVector.push_back(firstCvPoint);
		secondImagePointsVector.push_back(secondCvPoint);
		}
	}

Matrix3dConstPtr FundamentalMatrixRansac::Convert(cv::Mat matrix)
	{
	ASSERT( matrix.cols == 3 && matrix.rows == 3, "FundamentalMatrixRansac: unexpected matrix size in output");
	Matrix3dPtr conversion = NewMatrix3d();

	for(unsigned rowIndex = 0; rowIndex < 3; rowIndex++)
		{
		for(unsigned columnIndex = 0; columnIndex < 3; columnIndex++)
			{
			switch( matrix.type() )
				{
				case CV_32FC1:
					{
					SetElement(*conversion, rowIndex, columnIndex, matrix.at<float>(rowIndex, columnIndex));
					break;
					}
				case CV_64FC1:
					{
					SetElement(*conversion, rowIndex, columnIndex, matrix.at<double>(rowIndex, columnIndex));
					break;
					}
				default:
					{
					ASSERT(false, "FundamentalMatrixRansac: unexpected matrix type in output");
					break;
					}	
				}
			}
		}

	return conversion;
	}


const FundamentalMatrixRansac::FundamentalMatrixRansacOptionsSet FundamentalMatrixRansac::DEFAULT_PARAMETERS =
	{
	.outlierThreshold = 1.3,
	.confidence = 0.99
	};


cv::Mat FundamentalMatrixRansac::ComputeFundamentalMatrix(const std::vector<cv::Point2d>& firstImagePointsVector, const std::vector<cv::Point2d>& secondImagePointsVector)
	{
	static const int METHOD = CV_FM_RANSAC;	
	
	cv::Mat fundamentalMatrix = cv::findFundamentalMat
		(
		firstImagePointsVector,
		secondImagePointsVector,
		METHOD,
		parameters.outlierThreshold,
		parameters.confidence
		);

	return fundamentalMatrix;
	}


void FundamentalMatrixRansac::ValidateParameters()
	{
	ASSERT(parameters.outlierThreshold >= 0, "FundamentalMatrixRansac Configuration Error: outlierThreshold is negative");
	ASSERT(parameters.confidence >= 0 && parameters.confidence <= 1, "FundamentalMatrixRansac Configuration Error: confidence should be a probability between 0 and 1");
	}

void FundamentalMatrixRansac::ValidateInputs(const std::vector<cv::Point2d>& firstImagePointsVector, const std::vector<cv::Point2d>& secondImagePointsVector)
	{
	ASSERT(firstImagePointsVector.size() == secondImagePointsVector.size(), "FundamentalMatrixRansac Error: Points vector do not have the same size");
	}

}


/** @} */
