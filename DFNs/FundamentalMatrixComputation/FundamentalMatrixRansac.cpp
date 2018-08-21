/**
 * @author Alessandro Bianco
 */

/**
* @addtogroup DFNs
* @{
*/

#include "FundamentalMatrixRansac.hpp"

#include <Macros/YamlcppMacros.hpp>
#include <Errors/Assert.hpp>

#include <opencv2/calib3d.hpp>

#include <stdlib.h>
#include <fstream>

using namespace MatrixWrapper;
using namespace CorrespondenceMap2DWrapper;
using namespace BaseTypesWrapper;

namespace CDFF
{
namespace DFN
{
namespace FundamentalMatrixComputation
{

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
	// Read data from input port
	std::vector<cv::Point2d> firstImagePointsVector;
	std::vector<cv::Point2d> secondImagePointsVector;
	Convert(&inMatches, firstImagePointsVector, secondImagePointsVector);

	// Process data
	ValidateInputs(firstImagePointsVector, secondImagePointsVector);
	cv::Mat fundamentalMatrix = ComputeFundamentalMatrix(firstImagePointsVector, secondImagePointsVector);

	// Write data to output port
	if (fundamentalMatrix.rows == 0 && fundamentalMatrix.cols == 0)
	{
		SetZeroMatrix(outFundamentalMatrix);
		outSuccess = false;
	}
	else
	{
		Matrix3dConstPtr tmp = Convert(fundamentalMatrix);
		Copy(*tmp, outFundamentalMatrix);
		delete(tmp);
		outSuccess = true;
		ComputeInliers(fundamentalMatrix);
	}
}

void FundamentalMatrixRansac::Convert(CorrespondenceMap2DConstPtr correspondenceMap, std::vector<cv::Point2d>& firstImagePointsVector, std::vector<cv::Point2d>& secondImagePointsVector)
{
	for (int correspondenceIndex = 0; correspondenceIndex < GetNumberOfCorrespondences(*correspondenceMap); correspondenceIndex++)
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

	for (unsigned rowIndex = 0; rowIndex < 3; rowIndex++)
	{
		for (unsigned columnIndex = 0; columnIndex < 3; columnIndex++)
		{
			switch (matrix.type())
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

	cv::Mat fundamentalMatrix = cv::findFundamentalMat(
		firstImagePointsVector,
		secondImagePointsVector,
		METHOD,
		parameters.outlierThreshold,
		parameters.confidence
	);

	return fundamentalMatrix;
}

/**
 * Compute the epipole e' of the second camera from equation e' F = 0,
 * by using result 9.13 of Richard Hartley and Andrew Zisserman, "Multiple
 * View Geometry in Computer Vision"
 */
Point2DConstPtr FundamentalMatrixRansac::ComputeSecondEpipole(const std::vector<cv::Point2d>& firstImagePointsVector, const std::vector<cv::Point2d>& secondImagePointsVector, cv::Mat fundamentalMatrix)
{
	cv::Mat solution;
	cv::Mat coeffiecientsMatrix, scalarsMatrix;
	cv::transpose ( fundamentalMatrix(cv::Rect(0, 0, 3, 2)), coeffiecientsMatrix);
	cv::transpose (-fundamentalMatrix(cv::Rect(0, 2, 3, 1)), scalarsMatrix);
	cv::solve(coeffiecientsMatrix, scalarsMatrix, solution, cv::DECOMP_SVD);

	Point2DPtr secondEpipolePtr = new Point2D();
	if (solution.at<double>(2,0) > 1e-7)
	{
		secondEpipolePtr->x = solution.at<double>(0,0) / solution.at<double>(2,0);
		secondEpipolePtr->y = solution.at<double>(1,0) / solution.at<double>(2,0);
	}
	else
	{
		secondEpipolePtr->x = 0;
		secondEpipolePtr->y = 0;
	}

	return secondEpipolePtr;
}

void FundamentalMatrixRansac::ComputeInliers(cv::Mat fundamentalMatrix)
	{
	ClearCorrespondences(outInlierMatches);
	int numberOfInputCorrespondences = GetNumberOfCorrespondences(inMatches);
	for(int correspondenceIndex = 0; correspondenceIndex < numberOfInputCorrespondences; correspondenceIndex++)
		{
		BaseTypesWrapper::Point2D sourcePoint = GetSource(inMatches, correspondenceIndex);
		BaseTypesWrapper::Point2D sinkPoint = GetSink(inMatches, correspondenceIndex);
		cv::Mat source = (cv::Mat_<double>(3, 1, CV_32FC1) << sourcePoint.x, sourcePoint.y, 1.0);
		cv::Mat sink = (cv::Mat_<double>(3, 1, CV_32FC1) << sinkPoint.x, sinkPoint.y, 1.0);
		cv::Mat error = sink.t() * fundamentalMatrix * source;
		if (error.at<double>(0,0) < parameters.outlierThreshold)
			{
			AddCorrespondence(outInlierMatches, sourcePoint, sinkPoint, 1);
			}
		}
	}

void FundamentalMatrixRansac::ValidateParameters()
{
	ASSERT(parameters.outlierThreshold >= 0, "FundamentalMatrixRansac Configuration Error: outlierThreshold is negative");
	ASSERT(parameters.confidence >= 0 && parameters.confidence <= 1, "FundamentalMatrixRansac Configuration Error: confidence should be a probability value between 0 and 1");
}

void FundamentalMatrixRansac::ValidateInputs(const std::vector<cv::Point2d>& firstImagePointsVector, const std::vector<cv::Point2d>& secondImagePointsVector)
{
	ASSERT(firstImagePointsVector.size() == secondImagePointsVector.size(), "FundamentalMatrixRansac Error: Points vector do not have the same size");
}

}
}
}

/** @} */
