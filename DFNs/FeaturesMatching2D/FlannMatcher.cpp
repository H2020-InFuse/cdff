/* --------------------------------------------------------------------------
*
* (C) Copyright …
*
* ---------------------------------------------------------------------------
*/

/*!
 * @file FlannMatcher.cpp
 * @date 29/01/2018
 * @author Alessandro Bianco
 */

/*!
 * @addtogroup DFNs
 * 
 * Implementation of the Flann Matcher class.
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
#include "FlannMatcher.hpp"
#include <Errors/Assert.hpp>
#include <VisualPointFeatureVector2DToMatConverter.hpp>
#include <ConversionCache/ConversionCache.hpp>
#include <Macros/YamlcppMacros.hpp>
#include "opencv2/opencv_modules.hpp"
#include "opencv2/features2d/features2d.hpp"
#include <stdlib.h>
#include <fstream>

using namespace Converters;
using namespace Common;

namespace dfn_ci {

using namespace VisualPointFeatureVector2DWrapper;
using namespace CorrespondenceMap2DWrapper;

/* --------------------------------------------------------------------------
 *
 * Public Member Functions
 *
 * --------------------------------------------------------------------------
 */
FlannMatcher::FlannMatcher()
	{
	parameters.distanceThreshold = DEFAULT_PARAMETERS.distanceThreshold;
	parameters.matcherMethod = DEFAULT_PARAMETERS.matcherMethod;

	parameters.numberOfChecks = DEFAULT_PARAMETERS.numberOfChecks;
	parameters.epsilon = DEFAULT_PARAMETERS.epsilon;
	parameters.sortedSearch = DEFAULT_PARAMETERS.sortedSearch;

	parameters.kdTreeSearchOptionsSet.numberOfTrees = DEFAULT_PARAMETERS.kdTreeSearchOptionsSet.numberOfTrees;

	parameters.kMeansClusteringOptionsSet.branching = DEFAULT_PARAMETERS.kMeansClusteringOptionsSet.branching;
	parameters.kMeansClusteringOptionsSet.iterations = DEFAULT_PARAMETERS.kMeansClusteringOptionsSet.iterations;
	parameters.kMeansClusteringOptionsSet.centersInitialization = DEFAULT_PARAMETERS.kMeansClusteringOptionsSet.centersInitialization;
	parameters.kMeansClusteringOptionsSet.convertibleBoundIndex = DEFAULT_PARAMETERS.kMeansClusteringOptionsSet.convertibleBoundIndex;

	parameters.autotunedOptionsSet.targetPrecision = DEFAULT_PARAMETERS.autotunedOptionsSet.targetPrecision;
	parameters.autotunedOptionsSet.buildWeight = DEFAULT_PARAMETERS.autotunedOptionsSet.buildWeight;
	parameters.autotunedOptionsSet.memoryWeight = DEFAULT_PARAMETERS.autotunedOptionsSet.memoryWeight;
	parameters.autotunedOptionsSet.sampleFraction = DEFAULT_PARAMETERS.autotunedOptionsSet.sampleFraction;

	parameters.hierarchicalClusteringOptionsSet.branching = DEFAULT_PARAMETERS.hierarchicalClusteringOptionsSet.branching;
	parameters.hierarchicalClusteringOptionsSet.centersInitialization = DEFAULT_PARAMETERS.hierarchicalClusteringOptionsSet.centersInitialization;
	parameters.hierarchicalClusteringOptionsSet.numberOfTrees = DEFAULT_PARAMETERS.hierarchicalClusteringOptionsSet.numberOfTrees;
	parameters.hierarchicalClusteringOptionsSet.leafSize = DEFAULT_PARAMETERS.hierarchicalClusteringOptionsSet.leafSize;

	parameters.localitySensitiveHashingOptionsSet.tableNumber = DEFAULT_PARAMETERS.localitySensitiveHashingOptionsSet.tableNumber;
	parameters.localitySensitiveHashingOptionsSet.keySize = DEFAULT_PARAMETERS.localitySensitiveHashingOptionsSet.keySize;
	parameters.localitySensitiveHashingOptionsSet.multiProbeLevel = DEFAULT_PARAMETERS.localitySensitiveHashingOptionsSet.multiProbeLevel;

	parameters.compositeSearchOptionsSet.branching = DEFAULT_PARAMETERS.compositeSearchOptionsSet.branching;
	parameters.compositeSearchOptionsSet.iterations = DEFAULT_PARAMETERS.compositeSearchOptionsSet.iterations;
	parameters.compositeSearchOptionsSet.centersInitialization = DEFAULT_PARAMETERS.compositeSearchOptionsSet.centersInitialization;
	parameters.compositeSearchOptionsSet.convertibleBoundIndex = DEFAULT_PARAMETERS.compositeSearchOptionsSet.convertibleBoundIndex;
	parameters.compositeSearchOptionsSet.numberOfTrees = DEFAULT_PARAMETERS.compositeSearchOptionsSet.numberOfTrees;

	configurationFilePath = "";
	}

FlannMatcher::~FlannMatcher()
	{

	}

void FlannMatcher::configure()
	{
	try
		{
		YAML::Node configuration= YAML::LoadFile( configurationFilePath );
		for(unsigned configuationIndex=0; configuationIndex < configuration.size(); configuationIndex++)
			{
			YAML::Node configurationNode = configuration[configuationIndex];
			Configure(configurationNode);
			}
		} 
	catch(YAML::ParserException& e) 
		{
    		ASSERT(false, e.what() );
		}
	catch(YAML::RepresentationException& e)
		{
		ASSERT(false, e.what() );
		}
	}


void FlannMatcher::process() 
	{
	cv::Mat sourceFeaturesMatrix = ConversionCache<VisualPointFeatureVector2DConstPtr, cv::Mat, VisualPointFeatureVector2DToMatConverter>::Convert(inSourceFeaturesVector);
	cv::Mat sinkFeaturesMatrix = ConversionCache<VisualPointFeatureVector2DConstPtr, cv::Mat, VisualPointFeatureVector2DToMatConverter>::Convert(inSinkFeaturesVector);
	ValidateInputs(sourceFeaturesMatrix, sinkFeaturesMatrix);
	
	cv::Mat sourceDescriptorsSubmatrix = sourceFeaturesMatrix( cv::Rect(2, 0, sourceFeaturesMatrix.cols-2, sourceFeaturesMatrix.rows) ); 
	cv::Mat sinkDescriptorsSubmatrix = sinkFeaturesMatrix( cv::Rect(2, 0, sinkFeaturesMatrix.cols-2, sinkFeaturesMatrix.rows) ); 

	std::vector< cv::DMatch > matchesMatrix = ComputeMatches(sourceDescriptorsSubmatrix, sinkDescriptorsSubmatrix);
	outCorrespondenceMap = Convert(matchesMatrix, sourceFeaturesMatrix, sinkFeaturesMatrix);
	}

const FlannMatcher::FlannMatcherOptionsSet FlannMatcher::DEFAULT_PARAMETERS =
	{
	.distanceThreshold = 0.02,
	.numberOfChecks = 32,
	.epsilon = 0,
	.sortedSearch = false,
	.matcherMethod = KD_TREE_SEARCH,
	.kdTreeSearchOptionsSet = 
		{
		.numberOfTrees = 4
		},
	.kMeansClusteringOptionsSet =
		{
		.branching = 32,
		.iterations = 11,
		.centersInitialization = cvflann::FLANN_CENTERS_RANDOM,
		.convertibleBoundIndex = 0.2
		},
	.autotunedOptionsSet =
		{
		.targetPrecision = 0.8,
		.buildWeight = 0.01,
		.memoryWeight = 0,
		.sampleFraction = 0.1
		},
	.hierarchicalClusteringOptionsSet =
		{
		.branching = 32,
		.centersInitialization = cvflann::FLANN_CENTERS_RANDOM,
		.numberOfTrees = 4,
		.leafSize = 100
		},
	.localitySensitiveHashingOptionsSet =
		{
		.tableNumber = 1,
		.keySize = 4,
		.multiProbeLevel = 2
		},
	.compositeSearchOptionsSet =
		{
		.branching = 32,
		.iterations = 11,
		.centersInitialization = cvflann::FLANN_CENTERS_RANDOM,
		.convertibleBoundIndex = 0.2,
		.numberOfTrees = 4		
		}
	};

cv::Ptr<cv::flann::IndexParams> FlannMatcher::ConvertParameters()
	{
	switch(parameters.matcherMethod)
		{
		case KD_TREE_SEARCH:
			{
			return new cv::flann::KDTreeIndexParams(parameters.kdTreeSearchOptionsSet.numberOfTrees); 
			}
		case K_MEANS_CLUSTERING:
			{
			return new cv::flann::KMeansIndexParams
				(
				parameters.kMeansClusteringOptionsSet.branching,
				parameters.kMeansClusteringOptionsSet.iterations,
				parameters.kMeansClusteringOptionsSet.centersInitialization,
				parameters.kMeansClusteringOptionsSet.convertibleBoundIndex
				); 
			}
		case AUTOTUNED_SEARCH:
			{
			return new cv::flann::AutotunedIndexParams
				(
				parameters.autotunedOptionsSet.targetPrecision,
				parameters.autotunedOptionsSet.buildWeight,
				parameters.autotunedOptionsSet.memoryWeight,
				parameters.autotunedOptionsSet.sampleFraction
				);
			}
		case HIERARCHICAL_CLUSTERING:
			{
			return new cv::flann::HierarchicalClusteringIndexParams
				(
				parameters.hierarchicalClusteringOptionsSet.branching,
				parameters.hierarchicalClusteringOptionsSet.centersInitialization,
				parameters.hierarchicalClusteringOptionsSet.numberOfTrees,
				parameters.hierarchicalClusteringOptionsSet.leafSize
				);
			}
		case LOCALITY_SENSITIVE_HASHING:
			{
			return new cv::flann::LshIndexParams
				(
				parameters.localitySensitiveHashingOptionsSet.tableNumber,
				parameters.localitySensitiveHashingOptionsSet.keySize,
				parameters.localitySensitiveHashingOptionsSet.multiProbeLevel
				);
			}
		case COMPOSITE_SEARCH:
			{
			return new cv::flann::CompositeIndexParams
				(
				parameters.compositeSearchOptionsSet.numberOfTrees,
				parameters.compositeSearchOptionsSet.branching,
				parameters.compositeSearchOptionsSet.iterations,
				parameters.compositeSearchOptionsSet.centersInitialization,
				parameters.compositeSearchOptionsSet.convertibleBoundIndex
				); 
			}
		case LINEAR_SEARCH:
			{
			return new cv::flann::LinearIndexParams();
			}
		default:
			{
			ASSERT(false, "FlannMatcher Error: unhandled matcher Method");
			}			
		}
	}


std::vector< cv::DMatch > FlannMatcher::ComputeMatches(cv::Mat sourceDescriptorsMatrix, cv::Mat sinkDescriptorsMatrix)
	{
	cv::Mat sourceDescriptorsMatrixTransposed, sinkDescriptorsMatrixTransposed;
	cv::transpose(sourceDescriptorsMatrix, sourceDescriptorsMatrixTransposed);
	cv::transpose(sinkDescriptorsMatrix, sinkDescriptorsMatrixTransposed);	

	cv::Ptr<cv::flann::SearchParams> searchParams = new cv::flann::SearchParams(parameters.numberOfChecks, parameters.epsilon, parameters.sortedSearch);
	cv::Ptr<cv::flann::IndexParams> indexParams = ConvertParameters();

	cv::FlannBasedMatcher matcher(indexParams, searchParams);
	std::vector< cv::DMatch > matchesVector;
	matcher.match( sourceDescriptorsMatrixTransposed, sinkDescriptorsMatrixTransposed, matchesVector );

	std::vector< cv::DMatch > goodMatchesVector;
	for(unsigned matchIndex = 0; matchIndex < matchesVector.size(); matchIndex++)
		{
		double matchDistance = matchesVector.at(matchIndex).distance;
		if(matchDistance <= parameters.distanceThreshold)
			{
			goodMatchesVector.push_back( matchesVector.at(matchIndex) );
			}
		}

	return goodMatchesVector;
	}


CorrespondenceMap2DConstPtr FlannMatcher::Convert(std::vector< cv::DMatch > matchesVector, cv::Mat sourceFeaturesMatrix, cv::Mat sinkFeaturesMatrix)
	{
	CorrespondenceMap2DPtr correspondenceMap = new CorrespondenceMap2D();

	for(unsigned matchIndex = 0; matchIndex < matchesVector.size(); matchIndex++)
		{
		cv::DMatch currentMatch = matchesVector.at(matchIndex);
		BaseTypesWrapper::Point2D sourcePoint, sinkPoint;
		sourcePoint.x = sourceFeaturesMatrix.at<float>( currentMatch.queryIdx, 0);
		sourcePoint.y = sourceFeaturesMatrix.at<float>( currentMatch.queryIdx, 1);
		sinkPoint.x = sinkFeaturesMatrix.at<float>( currentMatch.trainIdx, 0);
		sinkPoint.y = sinkFeaturesMatrix.at<float>( currentMatch.trainIdx, 1);
		
		AddCorrespondence(*correspondenceMap, sourcePoint, sinkPoint, 1 - currentMatch.distance / parameters.distanceThreshold);
		}

	return correspondenceMap;
	}


void FlannMatcher::ValidateParameters()
	{
	ASSERT(parameters.distanceThreshold > 0, "FlannMatcher Error: distanceThreshold is not positive");
	}

void FlannMatcher::ValidateInputs(cv::Mat sourceFeaturesMatrix, cv::Mat sinkFeaturesMatrix)
	{
	ASSERT( sourceFeaturesMatrix.cols == sinkFeaturesMatrix.cols, "FlannMatcher Error: Input features vectors have different descriptors");
	}


void FlannMatcher::Configure(const YAML::Node& configurationNode)
	{
	std::string nodeName = configurationNode["Name"].as<std::string>();
	if ( nodeName == "GeneralParameters")
		{
		YAMLCPP_DFN_ASSIGN(distanceThreshold, float, configurationNode, "DistanceThreshold");
		}
	//Ignore everything else
	}

}


/** @} */
