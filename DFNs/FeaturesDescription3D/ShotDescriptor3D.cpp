/* --------------------------------------------------------------------------
*
* (C) Copyright …
*
* ---------------------------------------------------------------------------
*/

/*!
 * @file ShotDescriptor3D.cpp
 * @date 24/01/2018
 * @author Alessandro Bianco
 */

/*!
 * @addtogroup DFNs
 * 
 * Implementation of the Shot Descriptor 3D class.
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
#include "ShotDescriptor3D.hpp"
#include <Errors/Assert.hpp>
#include <PointCloudToPclPointCloudConverter.hpp>
#include <PointCloudToPclNormalsCloudConverter.hpp>
#include <MatToVisualPointFeatureVector3DConverter.hpp>
#include <ConversionCache/ConversionCache.hpp>
#include <Macros/YamlcppMacros.hpp>
#include <pcl/kdtree/kdtree.h>
#include <pcl/features/normal_3d.h>

#include <stdlib.h>
#include <fstream>


using namespace Common;
using namespace Converters;
using namespace VisualPointFeatureVector3DWrapper;
using namespace PointCloudWrapper;

namespace dfn_ci {


/* --------------------------------------------------------------------------
 *
 * Public Member Functions
 *
 * --------------------------------------------------------------------------
 */
ShotDescriptor3D::ShotDescriptor3D()
	{
	parametersHelper.AddParameter<float>(
		"GeneralParameters", "LocalReferenceFrameEstimationRadius", parameters.baseOptions.localReferenceFrameEstimationRadius, DEFAULT_PARAMETERS.baseOptions.localReferenceFrameEstimationRadius);
	parametersHelper.AddParameter<double>("GeneralParameters", "SearchRadius", parameters.baseOptions.searchRadius, DEFAULT_PARAMETERS.baseOptions.searchRadius);
	parametersHelper.AddParameter<OutputFormat, OutputFormatHelper>("GeneralParameters", "OutputFormat", parameters.baseOptions.outputFormat, DEFAULT_PARAMETERS.baseOptions.outputFormat);
	parametersHelper.AddParameter<bool>("GeneralParameters", "EnableNormalsEstimation", parameters.baseOptions.enableNormalsEstimation, DEFAULT_PARAMETERS.baseOptions.enableNormalsEstimation);
	parametersHelper.AddParameter<bool>("GeneralParameters", "ForceNormalsEstimation", parameters.baseOptions.forceNormalsEstimation, DEFAULT_PARAMETERS.baseOptions.forceNormalsEstimation);

	parametersHelper.AddParameter<double>("NormalEstimationParameters", "SearchRadius", parameters.normalEstimationOptions.searchRadius, DEFAULT_PARAMETERS.normalEstimationOptions.searchRadius);
	parametersHelper.AddParameter<int>(
		"NormalEstimationParameters", "NeighboursSetSize", parameters.normalEstimationOptions.neighboursSetSize, DEFAULT_PARAMETERS.normalEstimationOptions.neighboursSetSize);

	inNormalsCloud = NULL;
	configurationFilePath = "";
	}

ShotDescriptor3D::~ShotDescriptor3D()
	{

	}

void ShotDescriptor3D::configure()
	{
	parametersHelper.ReadFile(configurationFilePath);
	ValidateParameters();
	}


void ShotDescriptor3D::process() 
	{
	pcl::PointCloud<pcl::PointXYZ>::ConstPtr inputPointCloud = 
		ConversionCache<PointCloudConstPtr, pcl::PointCloud<pcl::PointXYZ>::ConstPtr, PointCloudToPclPointCloudConverter>::Convert(inPointCloud);
	pcl::IndicesConstPtr indicesList = Convert(inFeaturesSet);
	ValidateMandatoryInputs(inputPointCloud, indicesList);

	pcl::PointCloud<pcl::Normal>::ConstPtr inputNormalsCloud;
	if (inNormalsCloud != NULL)
		{
		inputNormalsCloud = ConversionCache<PointCloudConstPtr, pcl::PointCloud<pcl::Normal>::ConstPtr, PointCloudToPclNormalsCloudConverter>::Convert(inNormalsCloud);
		}
	else
		{
		inputNormalsCloud = boost::make_shared<pcl::PointCloud<pcl::Normal> >();
		}
	pcl::PointCloud<pcl::SHOT352>::ConstPtr shotPointCloud = ComputeShotDescriptors(inputPointCloud, indicesList, inputNormalsCloud);
	outFeaturesSetWithDescriptors = Convert(inputPointCloud, indicesList, shotPointCloud);
	}

ShotDescriptor3D::OutputFormatHelper::OutputFormatHelper(const std::string& parameterName, OutputFormat& boundVariable, const OutputFormat& defaultValue) :
	ParameterHelper(parameterName, boundVariable, defaultValue)
	{

	}

ShotDescriptor3D::OutputFormat ShotDescriptor3D::OutputFormatHelper::Convert(const std::string& outputFormat)
	{
	if (outputFormat == "Positions" || outputFormat == "0")
		{
		return POSITIONS_OUTPUT;
		}
	else if (outputFormat == "References" || outputFormat == "1")
		{
		return REFERENCES_OUTPUT;
		}
	ASSERT(false, "ShotDescriptor3d Error: unhandled output format");
	return POSITIONS_OUTPUT;
	}

const ShotDescriptor3D::ShotOptionsSet ShotDescriptor3D::DEFAULT_PARAMETERS
	{
	.baseOptions =
		{
		.localReferenceFrameEstimationRadius = 0.10,
		.searchRadius = 0.01,
		.outputFormat = POSITIONS_OUTPUT,
		.enableNormalsEstimation = true,
		.forceNormalsEstimation = true
		},
	.normalEstimationOptions =
		{
		.searchRadius = 0.00,
		.neighboursSetSize = 10
		}
	};


pcl::PointCloud<pcl::SHOT352>::ConstPtr ShotDescriptor3D::ComputeShotDescriptors
			(
			pcl::PointCloud<pcl::PointXYZ>::ConstPtr pointCloud, 
			pcl::IndicesConstPtr indicesList,
			pcl::PointCloud<pcl::Normal>::ConstPtr optionalNormalsCloud
			)
	{
	pcl::search::KdTree<pcl::PointXYZ>::Ptr kdTree = boost::make_shared<pcl::search::KdTree<pcl::PointXYZ> >();
	kdTree->setInputCloud(pointCloud);

	
	pcl::PointCloud<pcl::Normal>::ConstPtr normalsCloud;
	bool validNormalsCloud = IsNormalsCloudValid(pointCloud, optionalNormalsCloud);
	if (parameters.baseOptions.forceNormalsEstimation || (parameters.baseOptions.enableNormalsEstimation && !validNormalsCloud) )
		{
	 	normalsCloud = EstimateNormals(pointCloud, kdTree);
		}
	else if (validNormalsCloud)
		{
		normalsCloud = optionalNormalsCloud;
		}
	else
		{
		ASSERT(false, "ShotDescriptor3D Error: Required normals cloud is invalid");
		}
	

	pcl::SHOTEstimation<pcl::PointXYZ, pcl::Normal, pcl::SHOT352> shot;
	shot.setSearchSurface(pointCloud);
	shot.setSearchMethod(kdTree);
	shot.setLRFRadius(parameters.baseOptions.localReferenceFrameEstimationRadius);
	shot.setInputCloud(pointCloud);
	shot.setIndices(indicesList);
	shot.setInputNormals(normalsCloud);
	shot.setKSearch(0); //This must be 0
	shot.setRadiusSearch(parameters.baseOptions.searchRadius);

	pcl::PointCloud<pcl::SHOT352>::Ptr featuresCloud = boost::make_shared<pcl::PointCloud<pcl::SHOT352> >();
	shot.compute(*featuresCloud);
	ASSERT(featuresCloud->points.size() == indicesList->size(), "ShotDescriptor3D Error: feature Cloud should have same size as indices List");
	return featuresCloud;
	}

pcl::PointCloud<pcl::Normal>::ConstPtr ShotDescriptor3D::EstimateNormals(pcl::PointCloud<pcl::PointXYZ>::ConstPtr pointCloud, pcl::search::KdTree<pcl::PointXYZ>::Ptr kdTree)
	{
	pcl::NormalEstimation<pcl::PointXYZ, pcl::Normal> normalEstimation;
	normalEstimation.setInputCloud (pointCloud);
	normalEstimation.setSearchMethod (kdTree);
	if (parameters.normalEstimationOptions.searchRadius > 0)
		{
		normalEstimation.setRadiusSearch (parameters.normalEstimationOptions.searchRadius);
		}
	else
		{
		normalEstimation.setKSearch (parameters.normalEstimationOptions.neighboursSetSize);
		}

	pcl::PointCloud<pcl::Normal>::Ptr normalsCloud (new pcl::PointCloud<pcl::Normal>);
	normalEstimation.compute(*normalsCloud);
	
	return normalsCloud;
	}

pcl::IndicesConstPtr ShotDescriptor3D::Convert( const VisualPointFeatureVector3DConstPtr featuresVector)
	{
	ASSERT(GetNumberOfPoints(*featuresVector) == 0 || GetVectorType(*featuresVector) == ALL_REFERENCES_VECTOR, "ShotDescriptor3D: input features set does not contain all reference-defined points");
	pcl::IndicesPtr indicesList = boost::make_shared<std::vector<int> >();
	
	for(int pointIndex = 0; pointIndex < GetNumberOfPoints(*featuresVector); pointIndex++)
		{
		indicesList->push_back( GetReferenceIndex(*featuresVector, pointIndex) );
		}

	return indicesList;
	}

VisualPointFeatureVector3DConstPtr ShotDescriptor3D::Convert
		(
		const pcl::PointCloud<pcl::PointXYZ>::ConstPtr inputCloud,
		const pcl::IndicesConstPtr indicesList,
		const pcl::PointCloud<pcl::SHOT352>::ConstPtr shotPointCloud
		)
	{
	static const unsigned SHOT_DESCRIPTOR_SIZE = 352;
	VisualPointFeatureVector3DPtr featuresVector = new VisualPointFeatureVector3D();
	ClearPoints(*featuresVector);
	for(unsigned pointIndex = 0; pointIndex < indicesList->size(); pointIndex++)
		{
		if (parameters.baseOptions.outputFormat == POSITIONS_OUTPUT)
			{
			pcl::PointXYZ point = inputCloud->points.at(indicesList->at(pointIndex));
			AddPoint(*featuresVector, point.x, point.y, point.z);
			}
		else if (parameters.baseOptions.outputFormat == REFERENCES_OUTPUT)
			{
			AddPoint(*featuresVector, indicesList->at(pointIndex) );
			}

		pcl::SHOT352 feature = shotPointCloud->points.at(pointIndex);
		for(unsigned componentIndex = 0; componentIndex < SHOT_DESCRIPTOR_SIZE; componentIndex++)
			{
			AddDescriptorComponent(*featuresVector, pointIndex, feature.descriptor[componentIndex]);
			} 
		}
	return featuresVector;
	}


void ShotDescriptor3D::ValidateParameters()
	{
	ASSERT( parameters.baseOptions.localReferenceFrameEstimationRadius >= 0, "ShotDescriptor3D Configuration error, localReferenceFrameEstimationRadius is negative");
	ASSERT( parameters.baseOptions.searchRadius >= 0, "ShotDescriptor3D Configuration error, search radius is negative");
	ASSERT(!parameters.baseOptions.forceNormalsEstimation || parameters.baseOptions.enableNormalsEstimation, "ShotDescriptor3D error, you cannot force and not enable normals estimation");

	if (parameters.baseOptions.enableNormalsEstimation)
		{
		ASSERT( parameters.normalEstimationOptions.searchRadius >= 0, "ShotDescriptor3D Configuration error, normal estimation search radius is negative");
		ASSERT( parameters.normalEstimationOptions.neighboursSetSize >= 0, "ShotDescriptor3D Configuration error, neighbours set size is negative");
		if ( parameters.normalEstimationOptions.searchRadius > 0 )
			{
			ASSERT(parameters.normalEstimationOptions.neighboursSetSize == 0, "ShotDescriptor3D error, only one between normal estimation search radius and neighbours set size can be defined");
			}
		else
			{
			ASSERT(parameters.normalEstimationOptions.neighboursSetSize > 0, "ShotDescriptor3D error, only one between normal estimation search radius and neighbours set size can be defined");
			}
		}
	}

void ShotDescriptor3D::ValidateMandatoryInputs(pcl::PointCloud<pcl::PointXYZ>::ConstPtr pointCloud, pcl::IndicesConstPtr indicesList)
	{
	ASSERT(indicesList->size() <= pointCloud->points.size(), "ShotDescriptor3D Error: Features list should be smaller than point cloud");
	for(unsigned pointIndex; pointIndex < pointCloud->points.size(); pointIndex++)
		{
		pcl::PointXYZ point = pointCloud->points.at(pointIndex);
		if (point.x != point.x || point.y != point.y || point.z != point.z)
			{
			ASSERT(false, "ShotDescriptor3D Error: Invalid point in input point cloud");
			}
		}
	}

bool ShotDescriptor3D::IsNormalsCloudValid(pcl::PointCloud<pcl::PointXYZ>::ConstPtr pointCloud, pcl::PointCloud<pcl::Normal>::ConstPtr normalsCloud)
	{
	if (pointCloud->points.size() != normalsCloud->points.size())
		{
		ASSERT(parameters.baseOptions.enableNormalsEstimation, "ShotDescriptor3D Error: Required normals cloud has different size from input cloud");
		return false;
		}
	for(unsigned pointIndex; pointIndex < normalsCloud->points.size(); pointIndex++)
		{
		pcl::Normal normal = normalsCloud->points.at(pointIndex);
		if (normal.normal_x != normal.normal_x || normal.normal_y != normal.normal_y || normal.normal_z != normal.normal_z)
			{
			ASSERT(parameters.baseOptions.enableNormalsEstimation, "ShotDescriptor3D Error: Required normals cloud has an invalid point");
			return false;
			}
		}
	return true;
	}

}


/** @} */
