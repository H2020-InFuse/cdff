/* --------------------------------------------------------------------------
*
* (C) Copyright …
*
* ---------------------------------------------------------------------------
*/

/*!
 * @file Icp3D.cpp
 * @date 25/01/2018
 * @author Alessandro Bianco
 */

/*!
 * @addtogroup DFNs
 * 
 * Implementation of the ICP 3D class.
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
#include "Icp3D.hpp"
#include <Errors/Assert.hpp>
#include <VisualPointFeatureVector3DToPclPointCloudConverter.hpp>
#include <EigenTransformToTransform3DConverter.hpp>
#include <ConversionCache/ConversionCache.hpp>
#include <pcl/sample_consensus/ransac.h>
#include <pcl/sample_consensus/sac_model_registration.h>
#include <pcl/registration/sample_consensus_prerejective.h>
#include <pcl/search/search.h>
#include <pcl/sample_consensus/sac_model_plane.h>
#include <pcl/kdtree/impl/kdtree_flann.hpp>
#include <SupportTypes.hpp>
#include <Macros/YamlcppMacros.hpp>
#include <pcl/registration/icp.h>

#include <stdlib.h>
#include <fstream>


using namespace Common;
using namespace Converters;
using namespace PoseWrapper;
using namespace VisualPointFeatureVector3DWrapper;
using namespace SupportTypes;

namespace dfn_ci {


/* --------------------------------------------------------------------------
 *
 * Public Member Functions
 *
 * --------------------------------------------------------------------------
 */
Icp3D::Icp3D()
	{
	parametersHelper.AddParameter<double>("GeneralParameters", "MaxCorrespondenceDistance", parameters.maxCorrespondenceDistance, DEFAULT_PARAMETERS.maxCorrespondenceDistance);
	parametersHelper.AddParameter<int>("GeneralParameters", "MaximumIterations", parameters.maximumIterations, DEFAULT_PARAMETERS.maximumIterations);
	parametersHelper.AddParameter<double>("GeneralParameters", "TransformationEpsilon", parameters.transformationEpsilon, DEFAULT_PARAMETERS.transformationEpsilon);
	parametersHelper.AddParameter<double>("GeneralParameters", "EuclideanFitnessEpsilon", parameters.euclideanFitnessEpsilon, DEFAULT_PARAMETERS.euclideanFitnessEpsilon);

	configurationFilePath = "";
	}

Icp3D::~Icp3D()
	{

	}

void Icp3D::configure()
	{
	parametersHelper.ReadFile(configurationFilePath);
	ValidateParameters();
	}


void Icp3D::process() 
	{
	if ( GetNumberOfPoints(*inSourceFeaturesVector) == 0 || GetNumberOfPoints(*inSinkFeaturesVector) == 0)
		{
		outSuccess = false;
		Transform3DPtr transform = new Transform3D();
		Reset(*transform);
		outTransform = transform;
		return;
		}

	PointCloudWithFeatures inputSourceCloud = 
		ConversionCache<VisualPointFeatureVector3DConstPtr, SupportTypes::PointCloudWithFeatures, VisualPointFeatureVector3DToPclPointCloudConverter>::Convert(inSourceFeaturesVector);
	PointCloudWithFeatures inputSinkCloud = 
		ConversionCache<VisualPointFeatureVector3DConstPtr, SupportTypes::PointCloudWithFeatures, VisualPointFeatureVector3DToPclPointCloudConverter>::Convert(inSinkFeaturesVector);
	ValidateInputs(inputSourceCloud, inputSinkCloud);
	outTransform = ComputeTransform(inputSourceCloud, inputSinkCloud);
	}

const Icp3D::IcpOptionsSet Icp3D::DEFAULT_PARAMETERS =
	{
	.maxCorrespondenceDistance = 0.05,
	.maximumIterations = 50,
	.transformationEpsilon = 1e-8,
	.euclideanFitnessEpsilon = 1.0	
	};

/**
** This is the function that computes the detect the presence of the source point cloud within the sink point cloud, and computes the trasform that allows the source to be transformed to its position in
** the sink.
** Observe that in PCL terminology the sink cloud is called target cloud.
**
**/
Transform3DConstPtr Icp3D::ComputeTransform(Converters::SupportTypes::PointCloudWithFeatures sourceCloud, Converters::SupportTypes::PointCloudWithFeatures sinkCloud)
	{
	pcl::IterativeClosestPoint<pcl::PointXYZ, pcl::PointXYZ> icp;
	icp.setInputCloud (sourceCloud.pointCloud);
	icp.setInputTarget (sinkCloud.pointCloud);

	icp.setMaxCorrespondenceDistance (parameters.maxCorrespondenceDistance);
	icp.setMaximumIterations (parameters.maximumIterations);
	icp.setTransformationEpsilon (parameters.transformationEpsilon);
	icp.setEuclideanFitnessEpsilon (parameters.euclideanFitnessEpsilon);

	pcl::PointCloud<pcl::PointXYZ>::Ptr outputCloud(new pcl::PointCloud<pcl::PointXYZ>);
	icp.align (*outputCloud);
	
	outSuccess = icp.hasConverged();
	if (outSuccess)
		{
		Eigen::Matrix4f eigenTransform = icp.getFinalTransformation();
		return ConversionCache<Eigen::Matrix4f, Transform3DConstPtr, EigenTransformToTransform3DConverter>::Convert(eigenTransform);
		}
	else
		{
		Transform3DPtr transform = new Transform3D();
		Reset(*transform);
		return transform;
		}
	
	}

PoseWrapper::Transform3DConstPtr Icp3D::Convert(Eigen::Matrix4f eigenTransform)
	{
	Transform3DPtr transform = new Transform3D();	

	Eigen::Matrix3f eigenRotationMatrix = eigenTransform.block(0,0,3,3);
	Eigen::Quaternionf eigenRotation (eigenRotationMatrix);

	SetPosition(*transform, eigenTransform(0, 3), eigenTransform(1, 3), eigenTransform(2, 3) );
	SetOrientation(*transform, eigenRotation.x(), eigenRotation.y(), eigenRotation.z(), eigenRotation.w());

	return transform;
	}


void Icp3D::ValidateParameters()
	{
	ASSERT( parameters.maxCorrespondenceDistance >= 0, "Icp3D Configuration error, Max Correspondence Distance is negative");
	ASSERT( parameters.maximumIterations >= 0, "Icp3D Configuration error, Maximum Iterations is negative");
	ASSERT( parameters.transformationEpsilon > 0, "Icp3D Configuration error, Transformation Epsilon is not strictly positive");
	ASSERT( parameters.euclideanFitnessEpsilon > 0, "Icp3D Configuration error, Euclidean Fitness Epsilon is not stricltly positive");
	}

void Icp3D::ValidateInputs(Converters::SupportTypes::PointCloudWithFeatures sourceCloud, Converters::SupportTypes::PointCloudWithFeatures sinkCloud)
	{
	ASSERT(sourceCloud.descriptorSize == sinkCloud.descriptorSize, "Icp3D Error, Source Cloud has different descriptor size than sink cloud");
	ValidateCloud(sourceCloud);
	ValidateCloud(sinkCloud);
	}

void Icp3D::ValidateCloud(Converters::SupportTypes::PointCloudWithFeatures cloud)
	{
	ASSERT(cloud.pointCloud->points.size() == cloud.featureCloud->points.size(), "Icp3D Error, point cloud has different size than feature cloud");

	for(unsigned pointIndex = 0; pointIndex < cloud.pointCloud->points.size(); pointIndex++)
		{
		pcl::PointXYZ point = cloud.pointCloud->points.at(pointIndex);
		FeatureType feature = cloud.featureCloud->points.at(pointIndex);
		ASSERT(point.x == point.x, "Icp3D Error, Cloud contains an NaN point");
		ASSERT(point.y == point.y, "Icp3D Error, Cloud contains an NaN point");
		ASSERT(point.z == point.z, "Icp3D Error, Cloud contains an NaN point");
		for(unsigned componentIndex = 0; componentIndex < cloud.descriptorSize; componentIndex++)
			{
			ASSERT(feature.histogram[componentIndex] == feature.histogram[componentIndex], "Icp3D Error, Feature Cloud contains a NaN feature");
			}
		for(unsigned componentIndex = cloud.descriptorSize; componentIndex < MAX_FEATURES_NUMBER; componentIndex++)
			{
			ASSERT(feature.histogram[componentIndex] == 0, "Icp3D Error, Feature Cloud contains an invalid feature, probably it was a conversion error");
			}
		}
	}


}


/** @} */
