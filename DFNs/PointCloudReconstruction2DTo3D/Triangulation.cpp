/**
 * @author Alessandro Bianco
 */

/**
 * @addtogroup DFNs
 * @{
 */

#include "Triangulation.hpp"

using namespace Converters;
using namespace CorrespondenceMap2DWrapper;
using namespace PoseWrapper;
using namespace PointCloudWrapper;

namespace dfn_ci
{

Triangulation::Triangulation()
{
	parametersHelper.AddParameter<double>("FirstCameraMatrix", "FocalLengthX",
		parameters.firstCameraMatrix.focalLengthX, DEFAULT_PARAMETERS.firstCameraMatrix.focalLengthX);
	parametersHelper.AddParameter<double>("FirstCameraMatrix", "FocalLengthY",
		parameters.firstCameraMatrix.focalLengthY, DEFAULT_PARAMETERS.firstCameraMatrix.focalLengthY);
	parametersHelper.AddParameter<double>("FirstCameraMatrix", "PrinciplePointX",
		parameters.firstCameraMatrix.principalPoint.x, DEFAULT_PARAMETERS.firstCameraMatrix.principalPoint.x);
	parametersHelper.AddParameter<double>("FirstCameraMatrix", "PrinciplePointY",
		parameters.firstCameraMatrix.principalPoint.y, DEFAULT_PARAMETERS.firstCameraMatrix.principalPoint.y);

	parametersHelper.AddParameter<double>("SecondCameraMatrix", "FocalLengthX",
		parameters.secondCameraMatrix.focalLengthX, DEFAULT_PARAMETERS.secondCameraMatrix.focalLengthX);
	parametersHelper.AddParameter<double>("SecondCameraMatrix", "FocalLengthY",
		parameters.secondCameraMatrix.focalLengthY, DEFAULT_PARAMETERS.secondCameraMatrix.focalLengthY);
	parametersHelper.AddParameter<double>("SecondCameraMatrix", "PrinciplePointX",
		parameters.secondCameraMatrix.principalPoint.x, DEFAULT_PARAMETERS.secondCameraMatrix.principalPoint.x);
	parametersHelper.AddParameter<double>("SecondCameraMatrix", "PrinciplePointY",
		parameters.secondCameraMatrix.principalPoint.y, DEFAULT_PARAMETERS.secondCameraMatrix.principalPoint.y);

	parametersHelper.AddParameter<bool>("GeneralParameters", "OutputInvalidPoints", parameters.outputInvalidPoints, DEFAULT_PARAMETERS.outputInvalidPoints);

	firstCameraMatrix = ConvertToMat(DEFAULT_PARAMETERS.firstCameraMatrix);
	secondCameraMatrix = ConvertToMat(DEFAULT_PARAMETERS.secondCameraMatrix);

	configurationFilePath = "";
}

Triangulation::~Triangulation()
{
}

void Triangulation::configure()
{
	parametersHelper.ReadFile(configurationFilePath);
	ValidateParameters();

	firstCameraMatrix = ConvertToMat(parameters.firstCameraMatrix);
	secondCameraMatrix = ConvertToMat(parameters.secondCameraMatrix);
}

void Triangulation::process()
{
	// Read data from input ports
	ValidateInputs(inMatches, inPose);

	cv::Mat projectionMatrix = pose3DToMat.Convert(&inPose);
	cv::Mat pointsVectorAtSource = ConvertAtPose(&inMatches, SOURCE_CAMERA);
	cv::Mat pointsVectorAtSink = ConvertAtPose(&inMatches, SINK_CAMERA);

	// Process data
	cv::Mat uniformPointCloudMatrix = Triangulate(projectionMatrix, pointsVectorAtSource, pointsVectorAtSink);

	// Write data to output port
	const PointCloud* tmp = Convert(uniformPointCloudMatrix);
	Copy(*tmp, outPointcloud);
	delete tmp;
}

const Triangulation::TriangulationOptionsSet Triangulation::DEFAULT_PARAMETERS =
{
	.firstCameraMatrix =
	{
		.focalLengthX = 1.0,
		.focalLengthY = 1.0,
		.principalPoint = cv::Point2d(0, 0)
	},
	.secondCameraMatrix =
	{
		.focalLengthX = 1.0,
		.focalLengthY = 1.0,
		.principalPoint = cv::Point2d(0, 0)
	},
	.outputInvalidPoints = false
};

cv::Mat Triangulation::ConvertToMat(CameraMatrix cameraMatrix)
{
	cv::Mat conversion(3, 3, CV_32FC1, cv::Scalar(0));
	conversion.at<float>(0,0) = cameraMatrix.focalLengthX;
	conversion.at<float>(1,1) = cameraMatrix.focalLengthY;
	conversion.at<float>(0,2) = cameraMatrix.principalPoint.x;
	conversion.at<float>(1,2) = cameraMatrix.principalPoint.y;
	conversion.at<float>(2,2) = 1.0;
	return conversion;
}

cv::Mat Triangulation::Triangulate(cv::Mat projectionMatrix, cv::Mat pointsVectorAtSource, cv::Mat pointsVectorAtSink)
{
	// The projection matrix of the first camera is [I|0] according to result
	// 9.14 of Richard Hartley and Andrew Zisserman, "Multiple View Geometry
	// in Computer Vision"
	cv::Mat identityProjection(3, 4, CV_32FC1, cv::Scalar(0) );
	identityProjection.at<float>(0,0) = 1;
	identityProjection.at<float>(1,1) = 1;
	identityProjection.at<float>(2,2) = 1;

	cv::Mat homogeneousPointCloudMatrix;
	cv::triangulatePoints(
		firstCameraMatrix * identityProjection,
		secondCameraMatrix * projectionMatrix,
		pointsVectorAtSource,
		pointsVectorAtSink,
		homogeneousPointCloudMatrix);

	return homogeneousPointCloudMatrix;
}

cv::Mat Triangulation::ConvertAtPose(CorrespondenceMap2DConstPtr correspondenceMap, CAMERA_TYPE cameraPoseIdentifier)
{
	cv::Mat pointsVector(2, GetNumberOfCorrespondences(*correspondenceMap), CV_32FC1);

	for (int correspondenceIndex = 0; correspondenceIndex < GetNumberOfCorrespondences(*correspondenceMap); correspondenceIndex++)
	{
		BaseTypesWrapper::Point2D point;
		if (cameraPoseIdentifier == SOURCE_CAMERA)
		{
			point = GetSource(*correspondenceMap, correspondenceIndex);
		}
		else
		{
			point = GetSink(*correspondenceMap, correspondenceIndex);
		}
		pointsVector.at<float>(0, correspondenceIndex) = point.x;
		pointsVector.at<float>(1, correspondenceIndex) = point.y;
	}

	return pointsVector;
}

PointCloudConstPtr Triangulation::Convert(cv::Mat homogeneousPointCloudMatrix)
{
	static const float EPSILON = 1e-6;
	PointCloudPtr pointCloud = new PointCloud();
	ClearPoints(*pointCloud);

	for (unsigned pointIndex = 0; pointIndex < homogeneousPointCloudMatrix.cols; pointIndex++)
	{
		float homogeneousPointX = homogeneousPointCloudMatrix.at<float>(0, pointIndex);
		float homogeneousPointY = homogeneousPointCloudMatrix.at<float>(1, pointIndex);
		float homogeneousPointZ = homogeneousPointCloudMatrix.at<float>(2, pointIndex);
		float homogeneousPointFactor = homogeneousPointCloudMatrix.at<float>(3, pointIndex);
		if (std::abs(homogeneousPointFactor) < EPSILON && parameters.outputInvalidPoints)
		{
			AddPoint(*pointCloud, std::numeric_limits<float>::quiet_NaN(), std::numeric_limits<float>::quiet_NaN(), std::numeric_limits<float>::quiet_NaN());
		}
		else
		{
			if (homogeneousPointZ/homogeneousPointFactor > 0)
			{
				AddPoint(*pointCloud,
					homogeneousPointX/homogeneousPointFactor,
					homogeneousPointY/homogeneousPointFactor,
					homogeneousPointZ/homogeneousPointFactor);
			}
			else if (parameters.outputInvalidPoints)
			{
				AddPoint(*pointCloud, std::numeric_limits<float>::quiet_NaN(), std::numeric_limits<float>::quiet_NaN(), std::numeric_limits<float>::quiet_NaN());
			}
		}
	}
	return pointCloud;
}

void Triangulation::ValidateParameters()
{
	ASSERT(parameters.firstCameraMatrix.focalLengthX > 0 && parameters.firstCameraMatrix.focalLengthY > 0,
		"Triangulation Error: the focal length of the first camera is not strictly positive");
	ASSERT(parameters.secondCameraMatrix.focalLengthX > 0 && parameters.secondCameraMatrix.focalLengthY > 0,
		"Triangulation Error: the focal length of the second camera is not strictly positive");
}

void Triangulation::ValidateInputs(const CorrespondenceMap2D& matches, const Pose3D& pose)
{
	// TODO
}

}

/** @} */
