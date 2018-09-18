/**
 * @author Alessandro Bianco
 */

/**
 * @addtogroup DFNs
 * @{
 */

#include "CartesianSystemTransform.hpp"

#include <PointCloudToPclPointCloudConverter.hpp>
#include <MatToVisualPointFeatureVector3DConverter.hpp>
#include <Macros/YamlcppMacros.hpp>
#include <Errors/Assert.hpp>

#include <stdlib.h>
#include <fstream>

using namespace Converters;
using namespace VisualPointFeatureVector3DWrapper;
using namespace PointCloudWrapper;
using namespace PoseWrapper;
using namespace BaseTypesWrapper;

namespace CDFF
{
namespace DFN
{
namespace PointCloudTransform
{

CartesianSystemTransform::CartesianSystemTransform()
{
	parametersHelper.AddParameter<bool>("GeneralParameters", "Placeholder", parameters.placeholder, DEFAULT_PARAMETERS.placeholder);

	configurationFilePath = "";
}

CartesianSystemTransform::~CartesianSystemTransform()
{
}

void CartesianSystemTransform::configure()
{
	parametersHelper.ReadFile(configurationFilePath);
	ValidateParameters();
}

void CartesianSystemTransform::process()
{
	AffineTransform inversionTransform = ConvertCloudPoseToInversionTransform(inPose);

	ClearPoints(outTransformedPointCloud);
	int numberOfPoints = GetNumberOfPoints(inPointCloud);
	for(int pointIndex = 0; pointIndex < numberOfPoints; pointIndex++)
		{
		Point3D point;
		point.x = GetXCoordinate(inPointCloud, pointIndex);
		point.y = GetYCoordinate(inPointCloud, pointIndex);
		point.z = GetZCoordinate(inPointCloud, pointIndex);
		Point3D transformedPoint = TransformPoint(point, inversionTransform);
		AddPoint(outTransformedPointCloud, transformedPoint.x, transformedPoint.y, transformedPoint.z);
		}
}

const CartesianSystemTransform::CartesianSystemTransformOptionsSet CartesianSystemTransform::DEFAULT_PARAMETERS
{
	/*.placeholder =*/ false
};

CartesianSystemTransform::AffineTransform CartesianSystemTransform::ConvertCloudPoseToInversionTransform(const Pose3D& cloudPoseInSystem)
	{
	Eigen::Quaternion<float> rotation(GetWRotation(cloudPoseInSystem), GetXRotation(cloudPoseInSystem), GetYRotation(cloudPoseInSystem), GetZRotation(cloudPoseInSystem));
	Eigen::Translation<float, 3> translation( GetXPosition(cloudPoseInSystem), GetYPosition(cloudPoseInSystem), GetZPosition(cloudPoseInSystem));
	AffineTransform affineTransform = translation * rotation.inverse();
	return affineTransform;
	}

Point3D CartesianSystemTransform::TransformPoint(const Point3D& point, const AffineTransform& affineTransform)
	{
	Eigen::Vector3f eigenPoint(point.x, point.y, point.z);
	Eigen::Vector3f eigenTransformedPoint = affineTransform * eigenPoint;
	Point3D transformedPoint;
	transformedPoint.x = eigenTransformedPoint.x();
	transformedPoint.y = eigenTransformedPoint.y();
	transformedPoint.z = eigenTransformedPoint.z();
	return transformedPoint;
	}

void CartesianSystemTransform::ValidateParameters()
{

}

}
}
}

/** @} */
