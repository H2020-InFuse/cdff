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
using namespace PointCloudWrapper;

namespace CDFF
{
namespace DFN
{
namespace BundleAdjustment
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
	ValidateInputs();

	cv::Mat measurementMatrix = correspondencesSequenceConverter.Convert(&inCorrespondenceMapsSequence);
	if (measurementMatrix.cols < 4) //Not enough points available.
		{
		outSuccess = false;
		return;
		}
	ValidateInitialEstimations(measurementMatrix.rows/2);

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

	// rightCameraTransform = leftCameraTransform - (baseline, 0, 0). This is a comment stating the formula.
	T rightProjectedPoint[3];
	rightProjectedPoint[0] = rightFx * (transformedPoint[0] - T(baseline)) + rightPx * transformedPoint[2];
	rightProjectedPoint[1] = rightFy * transformedPoint[1] + rightPy * transformedPoint[2];
	rightProjectedPoint[2] = transformedPoint[2];

	residual[0] = (transformedPoint[2] > TOLERANCE_W) ? T(0) : TOLERANCE_COST_FACTOR*(TOLERANCE_W - transformedPoint[2]);
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
	std::vector<Point3d> mutablePoints3dStructure(numberOfPoints);
	std::vector<Transform3d> mutableTransforms3dStructure(numberOfImages/2);
	InitializePoints(mutablePoints3dStructure, measurementMatrix);
	InitializePoses(mutableTransforms3dStructure, numberOfImages);

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
	/*for (int pointIndex = 0; pointIndex < numberOfPoints; pointIndex++)
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
	success = true;*/
	float numberOfResiduals = static_cast<float>(5 * numberOfPoints * numberOfImages/2);
	outError = summary.final_cost / numberOfResiduals;
	success = outError < parameters.squaredPixelErrorTolerance;
	return projectionMatricesList;
	}

void CeresAdjustment::ConvertProjectionMatricesListToPosesSequence(std::vector<cv::Mat> projectionMatricesList, PoseWrapper::Poses3DSequence& posesSequence)
	{
	Clear(posesSequence);
	DEBUG_PRINT_TO_LOG("Projection matrices list size", projectionMatricesList.size() );
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
	DEBUG_PRINT_TO_LOG("pose vector size", GetNumberOfPoses(posesSequence) );
	}

void CeresAdjustment::InitializePoints(std::vector<Point3d>& pointCloud, cv::Mat measurementMatrix)
	{
	//Initialization without initial estimation, all point (x, y, z) are (pixelX, pixely, 1).
	if (!initialPointEstimationIsAvailable)
		{
		for(int pointIndex = 0; pointIndex < measurementMatrix.cols; pointIndex++)
			{
			for(int pointElement = 0; pointElement < 3; pointElement++)
				{
				pointCloud.at(pointIndex)[pointElement] = (pointElement == 2) ? 1 : measurementMatrix.at<float>(pointElement, pointIndex);
				}
			}
		return;
		}

	//Initialization with initial estimation, this works under the assumption that the order of points in the correspondenceMap and the measurement Matrix is preserved.
	CorrespondenceMap2D firstCorrespondenceMap = GetCorrespondenceMap(inCorrespondenceMapsSequence, 0);	

	for(int measurementIndex = 0; measurementIndex < measurementMatrix.cols; measurementIndex++)
		{
		float measureX = measurementMatrix.at<float>(0, measurementIndex);
		float measureY = measurementMatrix.at<float>(1, measurementIndex);
		bool found = false;
		for(int correspondenceIndex = 0; correspondenceIndex < GetNumberOfCorrespondences(firstCorrespondenceMap) && !found; correspondenceIndex++)
			{
			Point2D correspondingPoint = GetSource(firstCorrespondenceMap, correspondenceIndex);
			if (measureX == correspondingPoint.x && measureY == correspondingPoint.y)
				{
				found = true;
				}
			}		
		}


	int startingMeasurementIndex = 0;
	int measureCounter = 0;
	for(int pointIndex = 0; pointIndex < GetNumberOfPoints(inGuessedPointCloud); pointIndex++)
		{
		Point2D correspondingPoint = GetSource(firstCorrespondenceMap, pointIndex);
		bool measurementFound = false;
		for(int measurementIndex = startingMeasurementIndex; measurementIndex < measurementMatrix.cols && !measurementFound; measurementIndex++)
			{
			float measureX = measurementMatrix.at<float>(0, measurementIndex);
			float measureY = measurementMatrix.at<float>(1, measurementIndex);
			if (measureX == correspondingPoint.x && measureY == correspondingPoint.y)
				{
				pointCloud.at(measurementIndex)[0] = GetXCoordinate(inGuessedPointCloud, pointIndex);
				pointCloud.at(measurementIndex)[1] = GetYCoordinate(inGuessedPointCloud, pointIndex);
				pointCloud.at(measurementIndex)[2] = GetZCoordinate(inGuessedPointCloud, pointIndex);

				measurementFound = true;
				startingMeasurementIndex = measurementIndex + 1;
				measureCounter++;
				}
			}
		}

	//numberOfValidElementsInFirstCorrespondenceMap information is lost in the conversion.
	//ASSERT( measureCounter == numberOfValidElementsInFirstCorrespondenceMap, "Ceres adjustment error: some 3d points were not matched with a correspondence in the first map");

	//Some points may not be initialized, due to the fact that they do not appear in the first correspondence map. 
	//These points will appear last in the measurement, to avoid problems in convergence they will removed.
	measurementMatrix = measurementMatrix( cv::Rect(0, 0, measureCounter, measurementMatrix.rows) );
	}

void CeresAdjustment::InitializePoses(std::vector<Transform3d>& posesSequence, int numberOfImages)
	{
	//Initialization without initial estimation, all transforms are (0,0,0) position and (0,0,0) rotation angles
	if (!initialPoseEstimationIsAvailable)
		{
		for(int stereoIndex = 0; stereoIndex < numberOfImages/2; stereoIndex++)
			{
			for(int transformElement = 0; transformElement < 6; transformElement++)
				{
				posesSequence.at(stereoIndex)[transformElement] = 0;
				}
			}
		return;
		}

	//Initialization with initial estimation
	int stereoIndex = 0;
	for(int transformElement = 0; transformElement < 6; transformElement++)
		{
		posesSequence.at(stereoIndex)[transformElement] = 0;
		}

	for(stereoIndex = 1; stereoIndex < numberOfImages/2; stereoIndex++)
		{
		const Pose3D& guessedPose = GetPose(inGuessedPosesSequence, stereoIndex - 1);
		posesSequence.at(stereoIndex)[0] = GetXPosition(guessedPose);
		posesSequence.at(stereoIndex)[1] = GetYPosition(guessedPose);
		posesSequence.at(stereoIndex)[2] = GetZPosition(guessedPose);

		//Computing Roll
		double rollSine = 2.0 * ( GetWOrientation(guessedPose) * GetXOrientation(guessedPose) + GetYOrientation(guessedPose) * GetZOrientation(guessedPose) );
		double rollCosine = 1.0 - 2.0 * ( GetXOrientation(guessedPose) * GetXOrientation(guessedPose) + GetYOrientation(guessedPose) * GetYOrientation(guessedPose) );
		posesSequence.at(stereoIndex)[3] = std::atan2(rollSine, rollCosine);

		// Computing pitch
		double pitchSine = 2.0 * ( GetWOrientation(guessedPose) * GetYOrientation(guessedPose) - GetZOrientation(guessedPose) * GetXOrientation(guessedPose) );
		if (std::fabs(pitchSine) >= 1)
			{
			posesSequence.at(stereoIndex)[4] = std::copysign(M_PI / 2, pitchSine); // use 90 degrees if out of range
			}
		else
			{
			posesSequence.at(stereoIndex)[4] = std::asin(pitchSine);
			}

		// Computing Yaw
		double yawSine = 2.0 * ( GetWOrientation(guessedPose) * GetZOrientation(guessedPose) + GetXOrientation(guessedPose) * GetYOrientation(guessedPose) );
		double yawCosine = 1.0 - 2.0 * ( GetYOrientation(guessedPose) * GetYOrientation(guessedPose) + GetZOrientation(guessedPose) * GetZOrientation(guessedPose) );  
		posesSequence.at(stereoIndex)[5] = std::atan2(yawSine, yawCosine);
		}
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

	for(int mapIndex = 0; mapIndex < n; mapIndex++)
		{
		const CorrespondenceMap2D& correspondenceMap = GetCorrespondenceMap(inCorrespondenceMapsSequence, mapIndex);
		for(int correspondenceIndex = 0; correspondenceIndex < GetNumberOfCorrespondences(correspondenceMap); correspondenceIndex++)
			{
			BaseTypesWrapper::Point2D sourcePoint = GetSource(correspondenceMap, correspondenceIndex);
			BaseTypesWrapper::Point2D sinkPoint = GetSink(correspondenceMap, correspondenceIndex);
			bool validPoint = sourcePoint.x == sourcePoint.x && sourcePoint.y == sourcePoint.y;		
			validPoint = validPoint && sinkPoint.x == sinkPoint.x && sinkPoint.y == sinkPoint.y;	
			ASSERT(validPoint, "Ceres Adjustment error, invalid correspondence in input");		
			for(int secondCorrespondenceIndex = correspondenceIndex+1; secondCorrespondenceIndex < GetNumberOfCorrespondences(correspondenceMap); secondCorrespondenceIndex++)
				{
				BaseTypesWrapper::Point2D secondSourcePoint = GetSource(correspondenceMap, secondCorrespondenceIndex);
				BaseTypesWrapper::Point2D secondSinkPoint = GetSink(correspondenceMap, secondCorrespondenceIndex);
				bool notRepeatedSource = sourcePoint.x != secondSourcePoint.x || sourcePoint.y != secondSourcePoint.y;
				bool notRepeatedSink = sinkPoint.x != secondSinkPoint.x || sinkPoint.y != secondSinkPoint.y;
				ASSERT(notRepeatedSource && notRepeatedSink, "Ceres Adjustment error, repeated source or sink points within the same correspondence map");			
				}
			}
		}
}

void CeresAdjustment::ValidateInitialEstimations(int numberOfCameras)
	{
	initialPoseEstimationIsAvailable = ( GetNumberOfPoses(inGuessedPosesSequence) > 0 );
	initialPointEstimationIsAvailable = ( GetNumberOfPoints(inGuessedPointCloud) > 0 );

	if (initialPoseEstimationIsAvailable && GetNumberOfPoses(inGuessedPosesSequence) != (numberOfCameras/2 - 1) )
		{
		initialPoseEstimationIsAvailable = false;
		PRINT_WARNING("Ceres adjustment, initial pose estimation does not match number of cameras, initial poses estimation is ignored");
		}

	CorrespondenceMap2D firstCorrespondenceMap = GetCorrespondenceMap(inCorrespondenceMapsSequence, 0);
	if (initialPointEstimationIsAvailable && GetNumberOfPoints(inGuessedPointCloud) != GetNumberOfCorrespondences(firstCorrespondenceMap) )
		{
		initialPointEstimationIsAvailable = false;
		PRINT_WARNING("Ceres adjustment, initial point estimation does not match number of first camera correspondences, initial point estimation is ignored");
		}

	if (initialPointEstimationIsAvailable)
		{
		for(int pointIndex = 0; pointIndex < GetNumberOfPoints(inGuessedPointCloud); pointIndex++)
			{	
			bool validPoint = GetXCoordinate(inGuessedPointCloud, pointIndex) == GetXCoordinate(inGuessedPointCloud, pointIndex);
			validPoint = validPoint && GetYCoordinate(inGuessedPointCloud, pointIndex) == GetYCoordinate(inGuessedPointCloud, pointIndex);
			validPoint = validPoint && GetZCoordinate(inGuessedPointCloud, pointIndex) == GetZCoordinate(inGuessedPointCloud, pointIndex);
			ASSERT(validPoint, "Ceres adjustment error, an invalid 3d point was provided as a guess");
			}
		}

	if (initialPoseEstimationIsAvailable)
		{
		for(int poseIndex = 0; poseIndex < GetNumberOfPoses(inGuessedPosesSequence); poseIndex++)
			{
			const Pose3D& pose = GetPose(inGuessedPosesSequence, poseIndex);
			bool validPose = GetXPosition(pose) == GetXPosition(pose);
			validPose = validPose && GetYPosition(pose) == GetYPosition(pose);
			validPose = validPose && GetZPosition(pose) == GetZPosition(pose);
			validPose = validPose && GetXOrientation(pose) == GetXOrientation(pose);
			validPose = validPose && GetYOrientation(pose) == GetYOrientation(pose);
			validPose = validPose && GetZOrientation(pose) == GetZOrientation(pose);
			validPose = validPose && GetWOrientation(pose) == GetWOrientation(pose);
			ASSERT(validPose, "Ceres adjustment error, an invalid pose was provided as a guess");
			}
		}
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
}
}

/** @} */
