/**
 * @author Alessandro Bianco
 */

/**
 * @addtogroup DFNs
 * @{
 */

#include "CeresEstimation.hpp"

#include <Errors/Assert.hpp>
#include <Macros/YamlcppMacros.hpp>

#include <opencv2/calib3d.hpp>
#include <Eigen/Geometry>

#include <stdlib.h>
#include <fstream>

using namespace PoseWrapper;
using namespace MatrixWrapper;
using namespace CorrespondenceMap3DWrapper;
using namespace Helpers;
using namespace BaseTypesWrapper;

namespace CDFF
{
namespace DFN
{
namespace Transform3DEstimation
{

CeresEstimation::CeresEstimation()
{
	parametersHelper.AddParameter<float>("GeneralParameters", "MaximumAllowedError", parameters.maximumAllowedError, DEFAULT_PARAMETERS.maximumAllowedError);
	parametersHelper.AddParameter<float>("GeneralParameters", "MaximumAllowedDeterminantError", parameters.maximumAllowedDeterminantError, DEFAULT_PARAMETERS.maximumAllowedDeterminantError);

	configurationFilePath = "";
}

CeresEstimation::~CeresEstimation()
{
}

void CeresEstimation::configure()
{
	parametersHelper.ReadFile(configurationFilePath);
	ValidateParameters();
}

void CeresEstimation::process()
{
	int numberOfCorrespondenceMaps = GetNumberOfCorrespondenceMaps(inMatches);
	int numberOfCameras = ComputeNumberOfCameras(numberOfCorrespondenceMaps);

	//If there are not enough correspondences, process fails.
	for(int mapIndex = 0; mapIndex < numberOfCorrespondenceMaps; mapIndex++)
		{
		if ( GetNumberOfCorrespondences ( GetCorrespondenceMap(inMatches, mapIndex) ) < 4 )
			{
			outError = -1;
			outSuccess = false;
			return;
			}
		}

	std::vector<Transform3d> transformList(numberOfCorrespondenceMaps);
	InitializeTransforms(transformList);
	outError = SolveEstimation(inMatches, numberOfCameras, transformList);
	
	if (outError > parameters.maximumAllowedError || outError < 0)
		{
		outSuccess = false;
		return;
		}

	outSuccess = SetOutputPoses(transformList);
}

CeresEstimation::Transform3DCostFunctor::Transform3DCostFunctor(Point3D source, Point3D sink)
	{
	this->source = source;
	this->sink = sink;
	}

template <typename T>
bool CeresEstimation::Transform3DCostFunctor::operator()(const T* const cameraTransform, T* residual) const 
	{
	T originalPoint[3] = {T(source.x), T(source.y), T(source.z)};
	T transformedPoint[3];
	TransformPoint(cameraTransform, originalPoint, transformedPoint);

	residual[0] = T(sink.x) - transformedPoint[0];
	residual[1] = T(sink.y) - transformedPoint[1];
	residual[2] = T(sink.z) - transformedPoint[2];

	return true;
	}

//There is an extra computation repeated at this step (the first order transform is computed twice)
//Moreover, we are actually adding a method for each order transform. And we will need to create a functor for each combination, which means exponential data.
//Consider refactoring in one single functor that takes the full list of camera transform.
template <typename T>
bool CeresEstimation::Transform3DCostFunctor::operator()(const T* const firstCameraTransform, const T* const secondCameraTransform, T* residual) const 
	{
	T originalPoint[3] = {T(source.x), T(source.y), T(source.z)};
	T firstOrderTransformedPoint[3];
	TransformPoint(firstCameraTransform, originalPoint, firstOrderTransformedPoint);
	T secondOrderTransformedPoint[3];
	TransformPoint(secondCameraTransform, firstOrderTransformedPoint, secondOrderTransformedPoint);

	residual[0] = T(sink.x) - secondOrderTransformedPoint[0];
	residual[1] = T(sink.y) - secondOrderTransformedPoint[1];
	residual[2] = T(sink.z) - secondOrderTransformedPoint[2];

	return true;
	}

template <typename T>
void CeresEstimation::Transform3DCostFunctor::TransformPoint(const T* const cameraTransform, const T* const originalPoint, T* transformedPoint) const
	{

	transformedPoint[0] = cameraTransform[0]*originalPoint[0] + cameraTransform[1]*originalPoint[1] + cameraTransform[2]*originalPoint[2] + cameraTransform[3];
	transformedPoint[1] = cameraTransform[4]*originalPoint[0] + cameraTransform[5]*originalPoint[1] + cameraTransform[6]*originalPoint[2] + cameraTransform[7];
	transformedPoint[2] = cameraTransform[8]*originalPoint[0] + cameraTransform[9]*originalPoint[1] + cameraTransform[10]*originalPoint[2] + cameraTransform[11];

	}

ceres::CostFunction* CeresEstimation::Transform3DCostFunctor::Create(Point3D source, Point3D sink, int transformChainLength)
	{
	if (transformChainLength == 1)
		{
		return 
			(
			new ceres::AutoDiffCostFunction<Transform3DCostFunctor, 3, 12>
				(
				new Transform3DCostFunctor(source, sink)
				)
			);
		}
	else if (transformChainLength == 2)
		{
		return 
			(
			new ceres::AutoDiffCostFunction<Transform3DCostFunctor, 3, 12, 12>
				(
				new Transform3DCostFunctor(source, sink)
				)
			);
		}
	else
		{
		ASSERT(false, "Ceres Estimation error, unhandled chain length");
		}
	}

const CeresEstimation::CeresEstimationOptionsSet CeresEstimation::DEFAULT_PARAMETERS =
{
	.maximumAllowedError = 0.01,
	.maximumAllowedDeterminantError = 0.05
};

int CeresEstimation::ComputeNumberOfCameras(int numberOfCorrespondenceMaps)
	{
	const int MAXIMUM_NUMBER_OF_CAMERAS = 8;
	for(int candidateNumber = 0; candidateNumber < MAXIMUM_NUMBER_OF_CAMERAS; candidateNumber++)
		{
		int expectedCorrespondencesNumber = (candidateNumber * (candidateNumber - 1)) / 2;
		if (expectedCorrespondencesNumber == numberOfCorrespondenceMaps )
			{
			return candidateNumber;
			}
		ASSERT(expectedCorrespondencesNumber < numberOfCorrespondenceMaps, "CorrespondenceMaps3DSequenceToMat error, number of correspondences is not as expected");
		}
	
	ASSERT(false, "CorrespondenceMaps3DSequenceToMatConverter error, number of images is too large");
	return 0;
	}

void CeresEstimation::InitializeTransforms(std::vector<Transform3d>& transformList)
	{	
	for(int mapIndex = 0; mapIndex < GetNumberOfCorrespondenceMaps(inMatches); mapIndex++)
		{
		const CorrespondenceMap3D& map = GetCorrespondenceMap(inMatches, mapIndex);
		Transform3d& transform = transformList.at(mapIndex);

		cv::Mat coefficientMatrix, valueMatrix;
		bool success = CreateLinearSystem(map, coefficientMatrix, valueMatrix);

		float error = 0;
		cv::Mat solution;
		if (success)
			{
			solution = SolveLinearSystem(coefficientMatrix, valueMatrix, error);
			}
		for(int componentIndex = 0; componentIndex < 12; componentIndex++)
			{
			transform[componentIndex] = success ? solution.at<float>(componentIndex) : 0;
			}
		}
	}

float CeresEstimation::SolveEstimation(const CorrespondenceMap3DWrapper::CorrespondenceMaps3DSequence& sequence, int numberOfCameras, std::vector<Transform3d>& transformList)
	{
	ceres::Problem transformEstimation;
	int numberOfResiduals = 0;
	int sourceIndex = 0;
	int sinkIndex = 1;
	int firstMapIndexWithSinkAsSource = numberOfCameras - 1; //This is the first map where the current sink becomes source
	for(int mapIndex = 0; mapIndex < GetNumberOfCorrespondenceMaps(sequence); mapIndex++)
		{
		const CorrespondenceMap3D& map = GetCorrespondenceMap(sequence, mapIndex);
		for(int correspondenceIndex = 0; correspondenceIndex < GetNumberOfCorrespondences(map); correspondenceIndex++)
			{
			Point3D sourcePoint = GetSource(map, correspondenceIndex);
			Point3D sinkPoint = GetSink(map, correspondenceIndex);

			ceres::CostFunction* transform3DCostFunctor = Transform3DCostFunctor::Create ( sourcePoint, sinkPoint, 1);
			transformEstimation.AddResidualBlock( transform3DCostFunctor, NULL, transformList.at(mapIndex) );
			numberOfResiduals += 3;

			//These are the constraints for the double transform sourcePoint -> sinkPoint -> secondSinkPoint
			for(int secondSinkIndex = sinkIndex + 1; secondSinkIndex < numberOfCameras; secondSinkIndex++)
				{
				int secondMapIndex = firstMapIndexWithSinkAsSource + secondSinkIndex - sinkIndex - 1;
				const CorrespondenceMap3D& secondMap = GetCorrespondenceMap(sequence, secondMapIndex);
				for(int secondCorrespondenceIndex = 0; secondCorrespondenceIndex < GetNumberOfCorrespondences(secondMap); secondCorrespondenceIndex++)
					{
					Point3D secondSourcePoint = GetSource(secondMap, secondCorrespondenceIndex);
					Point3D secondSinkPoint = GetSink(secondMap, secondCorrespondenceIndex);
					if (sinkPoint.x == secondSourcePoint.x && sinkPoint.y == secondSourcePoint.y && sinkPoint.y == secondSourcePoint.y)
						{
						ceres::CostFunction* transform3DCostFunctor = Transform3DCostFunctor::Create ( sourcePoint, secondSinkPoint, 2);
						transformEstimation.AddResidualBlock( transform3DCostFunctor, NULL, transformList.at(mapIndex), transformList.at(secondMapIndex) );
						numberOfResiduals += 3;
						}
					}
				}

			}

		if (sinkIndex == numberOfCameras - 1)
			{
			sourceIndex++;
			sinkIndex = sourceIndex+1;
			firstMapIndexWithSinkAsSource = mapIndex + 1 + (numberOfCameras - sinkIndex);
			}
		else
			{
			sinkIndex++;
			firstMapIndexWithSinkAsSource += numberOfCameras - sinkIndex;
			}
		}

	if (numberOfResiduals < 6 * numberOfCameras)
		{
		return -1;
		}

	//Calling the solver
	ceres::Solver::Options ceresOptions;
	ceresOptions.linear_solver_type = ceres::DENSE_SCHUR;
	ceresOptions.minimizer_progress_to_stdout = true;
	ceresOptions.logging_type = ceres::SILENT;
	ceres::Solver::Summary summary;
	ceres::Solve(ceresOptions, &transformEstimation, &summary);
	return summary.final_cost / static_cast<float>(numberOfResiduals);
	}

bool CeresEstimation::SetOutputPoses(const std::vector<Transform3d>& transformList)
	{
	Clear(outTransforms);
	int validTransformCount = 0;
	for(int poseIndex = 0; poseIndex < transformList.size(); poseIndex++)
		{
		const Transform3d& transform = transformList.at(poseIndex);
		Pose3D pose;

		cv::Mat rotation = (cv::Mat_<double>(3, 3) <<
			transform[0], transform[1], transform[2],
			transform[4], transform[5], transform[6],
			transform[8], transform[9], transform[10] );
		double rotationDeterminant = cv::determinant(rotation);
		
		if (std::abs(rotationDeterminant) >= 1 - parameters.maximumAllowedDeterminantError && std::abs(rotationDeterminant) <= 1 + parameters.maximumAllowedDeterminantError)
			{
			cv::Mat translation = (cv::Mat_<double>(3, 1) << transform[3], transform[7], transform[11] );
			cv::Mat position = - rotation.inv() * translation;
			SetPosition(pose, position.at<double>(0,0), position.at<double>(1,0), position.at<double>(2,0));

			double qw = std::sqrt(1.00 + rotation.at<double>(0,0) + rotation.at<double>(1,1) + rotation.at<double>(2,2)) / 2;
			double qx = (rotation.at<double>(2,1) - rotation.at<double>(1,2)) / ( 4 * qw );
			double qy = (rotation.at<double>(0,2) - rotation.at<double>(2,0)) / ( 4 * qw );
			double qz = (rotation.at<double>(1,0) - rotation.at<double>(0,1)) / ( 4 * qw );
			SetOrientation(pose, qx, qy, qz, qw);

			validTransformCount++;
			}
		else
			{
			SetPosition(pose, 0, 0, 0);
			SetOrientation(pose, 0, 0, 0, 0);
			}

		AddPose(outTransforms, pose);		
		}
	return (validTransformCount > 0);
	}


bool CeresEstimation::CreateLinearSystem(const CorrespondenceMap3D& map, cv::Mat& coefficientMatrix, cv::Mat& valueMatrix)
	{
	const float EPSILON = 0.00001;
	const int NUMBER_OF_DEGREES_OF_FREEDOM = 6;
	const int NUMBER_OF_VARIABLES = 12;

	coefficientMatrix = cv::Mat();
	valueMatrix = cv::Mat();
	for(int correspondenceIndex = 0; correspondenceIndex < GetNumberOfCorrespondences(map); correspondenceIndex++)
		{
		Point3D source = GetSource(map, correspondenceIndex);
		Point3D sink = GetSink(map, correspondenceIndex);
	
		if (source.x != source.x || source.y != source.y || source.z != source.z || sink.x != sink.x || sink.y != sink.y || sink.z != sink.z)
			{
			continue;
			}

		cv::Mat coefficientMatrixPart = ( cv::Mat_<float>(3, 12) << 
			source.x, source.y, source.z, 1,    0, 0, 0, 0,    0, 0, 0, 0,
			0, 0, 0, 0,   source.x, source.y, source.z, 1,     0, 0, 0, 0,
			0, 0, 0, 0,   0, 0, 0, 0,     source.x, source.y, source.z, 1 );
		cv::Mat valueMatrixPart = ( cv::Mat_<float>(3, 1) << sink.x, sink.y, sink.z);
		
		if (coefficientMatrix.rows == 0)
			{
			coefficientMatrix = coefficientMatrixPart;
			valueMatrix = valueMatrixPart;
			}		
		else
			{
			cv::Mat matrixList[2] = {coefficientMatrix, coefficientMatrixPart};
			cv::vconcat(matrixList, 2, coefficientMatrix);
			matrixList[0] = valueMatrix;
			matrixList[1] = valueMatrixPart;
			cv::vconcat(matrixList, 2, valueMatrix);
			}
		}

	if ( coefficientMatrix.rows < NUMBER_OF_VARIABLES )
		{
		return false;
		}

	cv::Mat singulaValueMatrix;
	cv::SVD::compute(coefficientMatrix, singulaValueMatrix);
	int rank = cv::countNonZero( singulaValueMatrix > EPSILON );
	return ( rank >= NUMBER_OF_DEGREES_OF_FREEDOM );
	}

cv::Mat CeresEstimation::SolveLinearSystem(cv::Mat coefficientMatrix, cv::Mat valueMatrix, float& error)
	{
	cv::Mat pseudoInverse;
	cv::invert(coefficientMatrix, pseudoInverse, cv::DECOMP_SVD);
	cv::Mat transformMatrix = pseudoInverse * valueMatrix;

	cv::Mat errorMatrix = coefficientMatrix * transformMatrix - valueMatrix;
	error = cv::norm(errorMatrix);
	return transformMatrix;
	}

void CeresEstimation::ValidateParameters()
{
	ASSERT(parameters.maximumAllowedError > 0, "LeastSquaresMinimization Configuration Error: maximumAllowedError has to be positive");
}

void CeresEstimation::ValidateInputs(const CorrespondenceMap3D& map)
{
	
}

}
}
}

/** @} */
