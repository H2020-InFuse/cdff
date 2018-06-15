/**
 * @author Alessandro Bianco
 */

/**
 * @addtogroup DFNs
 * @{
 */

#include "CeresAdjustment.hpp"
#include <FrameToMatConverter.hpp>
#include <Macros/YamlcppMacros.hpp>
#include <Errors/Assert.hpp>
#include <stdlib.h>
#include <fstream>
#include <ceres/rotation.h>

using namespace Helpers;
using namespace Converters;
using namespace CorrespondenceMap2DWrapper;
using namespace PoseWrapper;
using namespace BaseTypesWrapper;

namespace dfn_ci
{

CeresAdjustment::CeresAdjustment()
{
	parametersHelper.AddParameter<float>("LeftCameraMatrix", "FocalLengthX", parameters.leftCameraMatrix.focalLengthX, DEFAULT_PARAMETERS.leftCameraMatrix.focalLengthX);
	parametersHelper.AddParameter<float>("LeftCameraMatrix", "FocalLengthY", parameters.leftCameraMatrix.focalLengthY, DEFAULT_PARAMETERS.leftCameraMatrix.focalLengthY);
	parametersHelper.AddParameter<float>("LeftCameraMatrix", "PrincipalPointX", parameters.leftCameraMatrix.principalPointX, DEFAULT_PARAMETERS.leftCameraMatrix.principalPointX);
	parametersHelper.AddParameter<float>("LeftCameraMatrix", "PrincipalPointY", parameters.leftCameraMatrix.principalPointY, DEFAULT_PARAMETERS.leftCameraMatrix.principalPointY);

	parametersHelper.AddParameter<float>("RightCameraMatrix", "FocalLengthX", parameters.rightCameraMatrix.focalLengthX, DEFAULT_PARAMETERS.rightCameraMatrix.focalLengthX);
	parametersHelper.AddParameter<float>("RightCameraMatrix", "FocalLengthY", parameters.rightCameraMatrix.focalLengthY, DEFAULT_PARAMETERS.rightCameraMatrix.focalLengthY);
	parametersHelper.AddParameter<float>("RightCameraMatrix", "PrincipalPointX", parameters.rightCameraMatrix.principalPointX, DEFAULT_PARAMETERS.rightCameraMatrix.principalPointX);
	parametersHelper.AddParameter<float>("RightCameraMatrix", "PrincipalPointY", parameters.rightCameraMatrix.principalPointY, DEFAULT_PARAMETERS.rightCameraMatrix.principalPointY);

	parametersHelper.AddParameter<float>("GeneralParameters", "Baseline", parameters.baseline, DEFAULT_PARAMETERS.baseline);
	parametersHelper.AddParameter<double>("GeneralParameters", "SquaredPixelErrorTolerance", parameters.squaredPixelErrorTolerance, DEFAULT_PARAMETERS.squaredPixelErrorTolerance);

	configurationFilePath = "";
}

CeresAdjustment::~CeresAdjustment()
{
}

void CeresAdjustment::configure()
{
	parametersHelper.ReadFile(configurationFilePath);
	ValidateParameters();

	leftCameraMatrix = CameraMatrixToCvMatrix(parameters.leftCameraMatrix);
	rightCameraMatrix = CameraMatrixToCvMatrix(parameters.rightCameraMatrix);
}

void CeresAdjustment::process()
{
	cv::Mat measurementMatrix = ComputeMeasurementMatrix(inCorrespondenceMapsSequence);
	if (measurementMatrix.cols < 4) //Not enough points available.
		{
		outSuccess = false;
		return;
		}

	std::vector<cv::Mat> projectionMatricesList = SolveBundleAdjustment(measurementMatrix, outSuccess);

	if (outSuccess)
		{
		ConvertProjectionMatricesListToPosesSequence(projectionMatricesList, outPosesSequence);
		}
}

const CeresAdjustment::CeresAdjustmentOptionsSet CeresAdjustment::DEFAULT_PARAMETERS =
{
	.leftCameraMatrix =
	{
		.focalLengthX = 1,
		.focalLengthY = 1,
		.principalPointX = 0,
		.principalPointY = 0,
	},
	.rightCameraMatrix =
	{
		.focalLengthX = 1,
		.focalLengthY = 1,
		.principalPointX = 0,
		.principalPointY = 0,
	},
	.baseline = 1.0,
	.squaredPixelErrorTolerance = 1e-4
};

CeresAdjustment::StereoImagePointCostFunctor::StereoImagePointCostFunctor(cv::Mat leftCameraMatrix, cv::Mat rightCameraMatrix, cv::Mat pointMeasuresMatrix, float baseline)
	{
	this->leftCameraMatrix = leftCameraMatrix;
	this->rightCameraMatrix = rightCameraMatrix;
	this->pointMeasuresMatrix = pointMeasuresMatrix;
	this->baseline = baseline;
	}

template <typename T>
bool CeresAdjustment::StereoImagePointCostFunctor::operator()(const T* const leftCameraTransform, const T* const point3d, T* residual) const 
	{
	static const T TOLERANCE_W = T(0.01);
	static const T TOLERANCE_COST_FACTOR = T(1000000);

	T leftFx = T( leftCameraMatrix.at<float>(0, 0) );
	T leftFy = T( leftCameraMatrix.at<float>(1, 1) );
	T leftPx = T( leftCameraMatrix.at<float>(0, 2) );
	T leftPy = T( leftCameraMatrix.at<float>(1, 2) );
	T rightFx = T( rightCameraMatrix.at<float>(0, 0) );
	T rightFy = T( rightCameraMatrix.at<float>(1, 1) );
	T rightPx = T( rightCameraMatrix.at<float>(0, 2) );
	T rightPy = T( rightCameraMatrix.at<float>(1, 2) );

	T leftObservedX = T( pointMeasuresMatrix.at<float>(0, 0) );
	T leftObservedY = T( pointMeasuresMatrix.at<float>(1, 0) );

	T rightObservedX = T( pointMeasuresMatrix.at<float>(2, 0) );
	T rightObservedY = T( pointMeasuresMatrix.at<float>(3, 0) );
	
	T rotation[3];
	rotation[0] = leftCameraTransform[3];
	rotation[1] = leftCameraTransform[4];
	rotation[2] = leftCameraTransform[5];

	T rotatedPoint[3];
	ceres::AngleAxisRotatePoint(rotation, point3d, rotatedPoint);
	
	T transformedPoint[3];
	transformedPoint[0] = rotatedPoint[0] + leftCameraTransform[0];
	transformedPoint[1] = rotatedPoint[1] + leftCameraTransform[1];
	transformedPoint[2] = rotatedPoint[2] + leftCameraTransform[2];

	T leftProjectedPoint[3];
	leftProjectedPoint[0] = leftFx * transformedPoint[0] + leftPx * transformedPoint[2];
	leftProjectedPoint[1] = leftFy * transformedPoint[1] + leftPy * transformedPoint[2];
	leftProjectedPoint[2] = transformedPoint[2];

	// rightCameraTransform = leftCameraTransform - (baseline, 0, 0).
	T rightProjectedPoint[3];
	rightProjectedPoint[0] = rightFx * (transformedPoint[0] - T(baseline)) + rightPx * transformedPoint[2];
	rightProjectedPoint[1] = rightFy * transformedPoint[1] + rightPy * transformedPoint[2];
	rightProjectedPoint[2] = transformedPoint[2];

	residual[0] = (transformedPoint[2] > TOLERANCE_W) ? T(0) : TOLERANCE_COST_FACTOR*(transformedPoint[2] - TOLERANCE_W);
	residual[1] = leftProjectedPoint[0] / leftProjectedPoint[2] - leftObservedX;
	residual[2] = leftProjectedPoint[1] / leftProjectedPoint[2] - leftObservedY;
	residual[3] = rightProjectedPoint[0] / rightProjectedPoint[2] - rightObservedX;
	residual[4] = rightProjectedPoint[1] / rightProjectedPoint[2] - rightObservedY;

	return true;
	}

ceres::CostFunction* CeresAdjustment::StereoImagePointCostFunctor::Create(cv::Mat leftCameraMatrix, cv::Mat rightCameraMatrix, cv::Mat pointMeasuresMatrix, float baseline)
	{
	return 
		(
		new ceres::AutoDiffCostFunction<StereoImagePointCostFunctor, 5, 6, 3>
			(
			new StereoImagePointCostFunctor(leftCameraMatrix, rightCameraMatrix, pointMeasuresMatrix, baseline)
			)
		);
	}

std::vector<cv::Mat> CeresAdjustment::SolveBundleAdjustment(cv::Mat measurementMatrix, bool& success)
	{
	int numberOfImages = measurementMatrix.rows/2;
	int numberOfPoints = measurementMatrix.cols;

	//Setting Data Structure
	typedef double Point3d[3];
	typedef double Transform3d[6];
	std::vector<Point3d> mutablePoints3dStructure(numberOfPoints);
	std::vector<Transform3d> mutableTransforms3dStructure(numberOfImages/2);
	for(int pointIndex = 0; pointIndex < numberOfPoints; pointIndex++)
		{
		for(int pointElement = 0; pointElement < 3; pointElement++)
			{
			mutablePoints3dStructure.at(pointIndex)[pointElement] = (pointElement == 2) ? 1 : 0;
			}
		}
	for(int stereoIndex = 0; stereoIndex < numberOfImages/2; stereoIndex++)
		{
		for(int transformElement = 0; transformElement < 6; transformElement++)
			{
			mutableTransforms3dStructure.at(stereoIndex)[transformElement] = 0;
			}
		}
	
	// Setting the problem
	ceres::Problem bundleAdjustment;
	for(int stereoIndex = 0; stereoIndex < numberOfImages/2; stereoIndex++)
		{
		for(int pointIndex = 0; pointIndex < numberOfPoints; pointIndex++)
			{
			ceres::CostFunction* stereoImagePointCostFunctor = StereoImagePointCostFunctor::Create
				(
				leftCameraMatrix,
				rightCameraMatrix,
				measurementMatrix(cv::Rect(pointIndex, 4*stereoIndex, 1, 4)),
				parameters.baseline
				);
			
			bundleAdjustment.AddResidualBlock
				(
				stereoImagePointCostFunctor, 
				NULL, 
				mutableTransforms3dStructure.at(stereoIndex), 
				mutablePoints3dStructure.at(pointIndex)
				);		
			}
		}
	bundleAdjustment.SetParameterBlockConstant(mutableTransforms3dStructure.at(0));

	//Calling the solver
	ceres::Solver::Options ceresOptions;
	ceresOptions.linear_solver_type = ceres::DENSE_SCHUR;
	ceresOptions.minimizer_progress_to_stdout = true;
	ceresOptions.logging_type = ceres::SILENT;
	ceres::Solver::Summary summary;
	ceres::Solve(ceresOptions, &bundleAdjustment, &summary);

	//Converting in expected output form
	std::vector<cv::Mat> projectionMatricesList(numberOfImages);
	for(int imageIndex = 0; imageIndex < numberOfImages; imageIndex++)
		{
		Transform3d& transform3d =  mutableTransforms3dStructure.at(imageIndex/2);
		double rotation[3] = { transform3d[3], transform3d[4], transform3d[5] };
		double rotationMatrix[9];
		ceres::AngleAxisToRotationMatrix(rotation, rotationMatrix);

		float baselineDisplacement = (imageIndex % 2 == 0) ? 0 : parameters.baseline;
		projectionMatricesList.at(imageIndex) = (cv::Mat_<float>(3, 4, CV_32FC1) <<
			rotationMatrix[0], rotationMatrix[3], rotationMatrix[6], transform3d[0] - baselineDisplacement,
			rotationMatrix[1], rotationMatrix[4], rotationMatrix[7], transform3d[1],
			rotationMatrix[2], rotationMatrix[5], rotationMatrix[8], transform3d[2]);
		}

	//Check convergence
	for (int pointIndex = 0; pointIndex < numberOfPoints; pointIndex++)
		{
		cv::Mat point3d = (cv::Mat_<float>(4, 1, CV_32FC1) << mutablePoints3dStructure.at(pointIndex)[0], mutablePoints3dStructure.at(pointIndex)[1], mutablePoints3dStructure.at(pointIndex)[2], 1);
		for(int imageIndex = 0; imageIndex < numberOfImages; imageIndex++)
			{
			cv::Mat projectionMatrix = projectionMatricesList.at(imageIndex);
			cv::Mat cameraMatrix = (imageIndex % 2 == 0) ? leftCameraMatrix : rightCameraMatrix;
			cv::Mat point2d = cameraMatrix * projectionMatrix * point3d;
			float estimatedX = point2d.at<float>(0, 0) / point2d.at<float>(2, 0);
			float estimatedY = point2d.at<float>(1, 0) / point2d.at<float>(2, 0);	
			float errorX = estimatedX - measurementMatrix.at<float>(2*imageIndex, pointIndex);
			float errorY = estimatedY - measurementMatrix.at<float>(2*imageIndex+1, pointIndex);
			float squaredPixelError = errorX*errorX + errorY*errorY;
			if (squaredPixelError > parameters.squaredPixelErrorTolerance)
				{
				success = false;
				return projectionMatricesList;
				}	
			}
		}
	success = true;
	return projectionMatricesList;
	}

void CeresAdjustment::ConvertProjectionMatricesListToPosesSequence(std::vector<cv::Mat> projectionMatricesList, PoseWrapper::Poses3DSequence& posesSequence)
	{
	for(int imageIndex = 0; imageIndex < projectionMatricesList.size(); imageIndex++)
		{
		cv::Mat projectionMatrix = projectionMatricesList.at(imageIndex);
		Eigen::Matrix3f eigenRotationMatrix;
		eigenRotationMatrix << projectionMatrix.at<float>(0,0), projectionMatrix.at<float>(0,1), projectionMatrix.at<float>(0,2),
			projectionMatrix.at<float>(1,0), projectionMatrix.at<float>(1,1), projectionMatrix.at<float>(1,2), 
			projectionMatrix.at<float>(2,0), projectionMatrix.at<float>(2,1), projectionMatrix.at<float>(2,2);
		Eigen::Quaternion<float> eigenRotation(eigenRotationMatrix);
		//eigenRotation = eigenRotation.inverse();
		eigenRotation.normalize();
		Eigen::Vector3f translation( projectionMatrix.at<float>(0,3), projectionMatrix.at<float>(1,3), projectionMatrix.at<float>(2,3) );
		Eigen::Vector3f position = - (eigenRotation.inverse() * translation);

		Pose3D newPose;
		SetPosition(newPose, position(0), position(1), position(2));
		SetOrientation(newPose, eigenRotation.x(), eigenRotation.y(), eigenRotation.z(), eigenRotation.w());

		AddPose(posesSequence, newPose);	
		}
	}

int CeresAdjustment::ComputeNumberOfImages(CorrespondenceMap2DWrapper::CorrespondenceMaps2DSequence& correspondenceMapsSequence)
	{
	if ( GetNumberOfCorrespondenceMaps(correspondenceMapsSequence) == 6 )
		{
		return 4;
		}
	else if ( GetNumberOfCorrespondenceMaps(correspondenceMapsSequence) == 15 )
		{
		return 6;
		}
	else if ( GetNumberOfCorrespondenceMaps(correspondenceMapsSequence) == 28 )
		{
		return 8;
		}
	else
		{
		ASSERT(false, "Unhandled number of correspondenceMaps");
		}
	return 0;
	}

cv::Mat CeresAdjustment::ComputeMeasurementMatrix(CorrespondenceMaps2DSequence& correspondenceMapsSequence)
	{
	int numberOfImages = ComputeNumberOfImages(correspondenceMapsSequence);
	std::vector<Point2D> usedPointsList;
	cv::Mat measurementMatrix;

	//Iterating on the points detected on the first image, we need to consider the first N-1 maps that involve the first image
	for(int mapIndex = 0; mapIndex < numberOfImages-1; mapIndex++)
		{
		const CorrespondenceMap2D& correspondenceMap = GetCorrespondenceMap(correspondenceMapsSequence, mapIndex);
		//Then for each such map we iterate on the correspondence and we examine the points of the first image
		for(int correspondenceIndex = 0; correspondenceIndex < GetNumberOfCorrespondences(correspondenceMap); correspondenceIndex++)
			{
			Point2D sourcePoint = GetSource(correspondenceMap, correspondenceIndex);
			Point2D sinkPoint = GetSink(correspondenceMap, correspondenceIndex);

			//We make sure that we do not examine the same point twice.
			if (!PointIsNotInVector(sourcePoint, usedPointsList))
				{
				continue;
				}
			//We compute a list of points that should match to the same 3D points as 'point' does in the first image
			ImagePoint sourceImagePoint(sourcePoint.x, sourcePoint.y, 0, false);
			ImagePoint sinkImagePoint(sinkPoint.x, sinkPoint.y, mapIndex+1, false);
			std::vector<ImagePoint> matchesChain = ComputeChainOfMatchingPoints(correspondenceMapsSequence, numberOfImages, sourceImagePoint, sinkImagePoint);

			//If the chain is maximal, i.e. the 3d point is visible from all images, we add it to the measurement matrix
			if (matchesChain.size() == numberOfImages && AllImagesAreRepresented(matchesChain, numberOfImages))
				{
				AddChainToMeasurementMatrix(matchesChain, measurementMatrix);
				usedPointsList.push_back(sourcePoint);
				}
			}
		}

	return measurementMatrix;
	}

std::vector<CeresAdjustment::ImagePoint> CeresAdjustment::ComputeChainOfMatchingPoints(CorrespondenceMaps2DSequence& correspondenceMapsSequence, int numberOfImages, ImagePoint point1, ImagePoint point2)
	{
	std::vector<ImagePoint> chain;
	chain.push_back(point1);
	chain.push_back(point2);

	int chainIndexToExplore;
	while( ThereAreUnexploredPoints(chain, chainIndexToExplore) )
		{
		ImagePoint pointToExplore = chain.at(chainIndexToExplore);
		int separationMapIndex; //before this index, all the maps treat pointToExplore.image as sink, after that index they treat pointToExplore.image as source.
		std::vector<int> mapsToExplore = ComputeMapsToExplore(numberOfImages, pointToExplore.image, separationMapIndex);

		for(std::vector<int>::iterator mapIndex = mapsToExplore.begin(); mapIndex != mapsToExplore.end(); mapIndex++)
			{
			const CorrespondenceMap2D& correspondenceMap = GetCorrespondenceMap(correspondenceMapsSequence, *mapIndex);
			for(int correspondenceIndex = 0; correspondenceIndex < GetNumberOfCorrespondences(correspondenceMap); correspondenceIndex++)
				{
				Point2D sourcePoint = GetSource(correspondenceMap, correspondenceIndex);
				Point2D sinkPoint = GetSink(correspondenceMap, correspondenceIndex);

				if (*mapIndex <= separationMapIndex && sinkPoint.x == pointToExplore.x && sinkPoint.y == pointToExplore.y)
					{
					int sourceImageIndex = GetSourceIndex(numberOfImages, *mapIndex, pointToExplore.image);
					if ( ImageIsNotInChain(chain, sourceImageIndex))
						{
						ImagePoint newPoint(sourcePoint.x, sourcePoint.y, sourceImageIndex, false);
						chain.push_back(newPoint);
						}
					}
				else if (*mapIndex > separationMapIndex && sourcePoint.x == pointToExplore.x && sourcePoint.y == pointToExplore.y)
					{
					int sinkImageIndex = GetSinkIndex(numberOfImages, *mapIndex, pointToExplore.image);
					if ( ImageIsNotInChain(chain, sinkImageIndex))
						{
						ImagePoint newPoint(sinkPoint.x, sinkPoint.y, sinkImageIndex, false);
						chain.push_back(newPoint);
						}
					}
				}
			}
		chain.at(chainIndexToExplore).explored = true;		
		}

	return chain;
	}

void CeresAdjustment::AddChainToMeasurementMatrix(const std::vector<ImagePoint> chain, cv::Mat& measurementMatrix)
	{
	cv::Mat chainMatrix(2*chain.size(), 1, CV_32FC1);
	for(int imageIndex = 0; imageIndex < chain.size(); imageIndex++)
		{
		chainMatrix.at<float>(2*imageIndex, 0) = chain.at(imageIndex).x;
		chainMatrix.at<float>(2*imageIndex+1, 0) = chain.at(imageIndex).y; 
		}

	if (measurementMatrix.cols == 0 && measurementMatrix.rows == 0)
		{
		measurementMatrix = chainMatrix;
		}
	else
		{
		cv::Mat matricesList[2] = {measurementMatrix, chainMatrix};
		cv::hconcat(matricesList, 2, measurementMatrix);
		}
	}

bool CeresAdjustment::PointIsNotInVector(Point2D point, const std::vector<Point2D>& vector)
	{
	for(int pointIndex = 0; pointIndex < vector.size(); pointIndex++)
		{
		const Point2D& vectorPoint = vector.at(pointIndex);
		if (point.x == vectorPoint.x && point.y == vectorPoint.y)
			{
			return false;
			}
		}
	return true;
	}

void CeresAdjustment::ValidateParameters()
{
	ASSERT(parameters.leftCameraMatrix.focalLengthX > 0 && parameters.leftCameraMatrix.focalLengthY > 0, "CeresAdjustment Configuration error: left focal length has to be positive");
	ASSERT(parameters.rightCameraMatrix.focalLengthX > 0 && parameters.rightCameraMatrix.focalLengthY > 0, "CeresAdjustment Configuration error: right focal length has to be positive");
	ASSERT(parameters.baseline > 0, "CeresAdjustment Configuration error: stereo camera baseline has to be positive");
	ASSERT(parameters.squaredPixelErrorTolerance > 0, "CeresAdjustment Configuration error: error tolerance has to be positive");
}

void CeresAdjustment::ValidateInputs()
{
	int n = GetNumberOfCorrespondenceMaps(inCorrespondenceMapsSequence);
	ASSERT( n == 6 || n == 15 || n == 28, "CeresAdjustment Error: you should provide correspondence maps for either 2, 3 or 4 pairs of stereo camera images");
}


bool CeresAdjustment::ThereAreUnexploredPoints(const std::vector<ImagePoint>& chain, int& chainIndexToExplore)
	{
	for(int pointIndex = 0; pointIndex < chain.size(); pointIndex++)
		{
		if (!chain.at(pointIndex).explored)
			{
			chainIndexToExplore = pointIndex;
			return true;
			}
		}
	return false;
	}

std::vector<int> CeresAdjustment::ComputeMapsToExplore(int numberOfImages, int imageIndex, int& separationPoint)
	{
	std::vector<int> mapsToExplore;

	int mapIndex = 0;
	for(int firstImage = 0; firstImage < numberOfImages; firstImage++)
		{
		for(int secondImage = firstImage+1; secondImage < numberOfImages; secondImage++)
			{
			if (firstImage == imageIndex)
				{
				mapsToExplore.push_back(mapIndex);
				}
			else if (secondImage == imageIndex)
				{
				mapsToExplore.push_back(mapIndex);
				separationPoint = mapIndex; //The last time the second sink image is the imageIndex, that is the separation point.
				}
			mapIndex++;
			}
		}

	return mapsToExplore;
	}

int CeresAdjustment::GetSourceIndex(int numberOfImages, int mapIndex, int imageIndex)
	{
	int startIndex = 0;
	int nextStartIndex = numberOfImages - 1;
	for(int sourceImage = 0; sourceImage < numberOfImages; sourceImage++)
		{
		if ( mapIndex >= startIndex && mapIndex < nextStartIndex)
			{
			return sourceImage;
			}
		startIndex = nextStartIndex;
		nextStartIndex += numberOfImages - 1 - sourceImage;
		}
	ASSERT(false, "CeresAdjustment::GetSourceIndex, no source index found");
	return 0;
	}

int CeresAdjustment::GetSinkIndex(int numberOfImages, int mapIndex, int imageIndex)
	{
	int startIndex = 0;
	int sourceImage = 0;
	for(sourceImage = 0; sourceImage < imageIndex; sourceImage++)
		{
		startIndex += numberOfImages - 1 - sourceImage;
		}
	return ( mapIndex - startIndex + imageIndex + 1);
	}

bool CeresAdjustment::AllImagesAreRepresented(const std::vector<ImagePoint>& chain, int numberOfImages)
	{
	std::vector<bool> representationsList(numberOfImages);
	for(int imageIndex = 0; imageIndex < numberOfImages; imageIndex++)
		{
		representationsList.at(imageIndex) = false;
		}
	for(int pointIndex = 0; pointIndex < chain.size(); pointIndex++)
		{
		const ImagePoint& point = chain.at(pointIndex);
		ASSERT(point.image < numberOfImages, "CeresAdjustment::AllImagesAreRepresented, point.image is greated than numberOfImages");
		representationsList.at(point.image) = true;
		}
	for(int imageIndex = 0; imageIndex < numberOfImages; imageIndex++)
		{
		if (!representationsList.at(imageIndex))
			{
			return false;
			}
		}
	return true;
	}

bool CeresAdjustment::ImageIsNotInChain(const std::vector<ImagePoint>& chain, int imageIndex)
	{
	for(int pointIndex = 0; pointIndex < chain.size(); pointIndex++)
		{
		if (chain.at(pointIndex).image == imageIndex)
			{
			return false;
			}
		}
	return true;
	}

cv::Mat CeresAdjustment::CameraMatrixToCvMatrix(const CameraMatrix& cameraMatrix)
	{
	cv::Mat cvCameraMatrix(3, 3, CV_32FC1, cv::Scalar(0));
	cvCameraMatrix.at<float>(0,0) = cameraMatrix.focalLengthX;
	cvCameraMatrix.at<float>(1,1) = cameraMatrix.focalLengthY;
	cvCameraMatrix.at<float>(2,0) = cameraMatrix.principalPointX;
	cvCameraMatrix.at<float>(2,1) = cameraMatrix.principalPointY;
	cvCameraMatrix.at<float>(2,2) = 1.0;

	return cvCameraMatrix;
	}
}

/** @} */
