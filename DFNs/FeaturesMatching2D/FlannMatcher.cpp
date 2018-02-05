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
	parameters.generalOptionsSet.distanceThreshold = DEFAULT_PARAMETERS.generalOptionsSet.distanceThreshold;
	parameters.generalOptionsSet.matcherMethod = DEFAULT_PARAMETERS.generalOptionsSet.matcherMethod;

	parameters.generalOptionsSet.numberOfChecks = DEFAULT_PARAMETERS.generalOptionsSet.numberOfChecks;
	parameters.generalOptionsSet.epsilon = DEFAULT_PARAMETERS.generalOptionsSet.epsilon;
	parameters.generalOptionsSet.sortedSearch = DEFAULT_PARAMETERS.generalOptionsSet.sortedSearch;

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
	.generalOptionsSet =
		{
		.distanceThreshold = 0.02,
		.numberOfChecks = 32,
		.epsilon = 0,
		.sortedSearch = false,
		.matcherMethod = KD_TREE_SEARCH
		},
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
	switch(parameters.generalOptionsSet.matcherMethod)
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
	cv::Mat validTypeSourceDescriptorsMatrix = ConvertToValidType(sourceDescriptorsMatrix);
	cv::Mat validTypeSinkDescriptorsMatrix = ConvertToValidType(sinkDescriptorsMatrix);

	cv::Ptr<cv::flann::SearchParams> searchParams = 
		new cv::flann::SearchParams(parameters.generalOptionsSet.numberOfChecks, parameters.generalOptionsSet.epsilon, parameters.generalOptionsSet.sortedSearch);
	cv::Ptr<cv::flann::IndexParams> indexParams = ConvertParameters();

	cv::FlannBasedMatcher matcher(indexParams, searchParams);

	typedef std::vector<cv::DMatch> MatchesSequence;
	const unsigned NumberOfBestMatchingSequencesToCompare = 2;
	const float ACCEPTANCE_RATIO = 0.75;

	std::vector<MatchesSequence> bestMatchingSequencesVector;
	matcher.knnMatch(validTypeSourceDescriptorsMatrix, validTypeSinkDescriptorsMatrix, bestMatchingSequencesVector, NumberOfBestMatchingSequencesToCompare);

	MatchesSequence sequenceOfSelectedMatches;
	for (unsigned sequenceIndex = 0; sequenceIndex < bestMatchingSequencesVector.size(); sequenceIndex++) 
		{
		if (bestMatchingSequencesVector[sequenceIndex][0].distance < bestMatchingSequencesVector[sequenceIndex][1].distance * ACCEPTANCE_RATIO)
			{
			sequenceOfSelectedMatches.push_back(bestMatchingSequencesVector[sequenceIndex][0]);
			}
		}

	return sequenceOfSelectedMatches;
	}

cv::Mat FlannMatcher::ConvertToValidType(cv::Mat floatDescriptorsMatrix)
	{
	if (parameters.generalOptionsSet.matcherMethod == LOCALITY_SENSITIVE_HASHING)
		{
		cv::Mat uint8DescriptorsMatrix(floatDescriptorsMatrix.rows, floatDescriptorsMatrix.cols, CV_8UC1);
		floatDescriptorsMatrix.convertTo(uint8DescriptorsMatrix, CV_8UC1);
		return uint8DescriptorsMatrix;
		}
	return floatDescriptorsMatrix;
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
		
		AddCorrespondence(*correspondenceMap, sourcePoint, sinkPoint, 1 - currentMatch.distance / parameters.generalOptionsSet.distanceThreshold);
		}

	return correspondenceMap;
	}


void FlannMatcher::ValidateParameters()
	{
	ASSERT(parameters.generalOptionsSet.distanceThreshold > 0, "FlannMatcher Error: distanceThreshold is not positive");
	ASSERT(parameters.generalOptionsSet.numberOfChecks > 0, "FlannMatcher Error: number of checks is not positive");
	ASSERT(parameters.generalOptionsSet.epsilon >= 0, "FlannMatcher Error: epsilon is negative");
	if (parameters.generalOptionsSet.matcherMethod == KD_TREE_SEARCH)
		{
		ASSERT(parameters.kdTreeSearchOptionsSet.numberOfTrees > 0, "FlannMatcher Error: kdTreeSearchOptionsSet.numberOfTrees is not positive");
		}
	else if (parameters.generalOptionsSet.matcherMethod == K_MEANS_CLUSTERING)
		{
		ASSERT(parameters.kMeansClusteringOptionsSet.branching > 0, "FlannMatcher Error: kMeansClusteringOptionsSet.branching is not positive");
		ASSERT(parameters.kMeansClusteringOptionsSet.iterations > 0, "FlannMatcher Error: kMeansClusteringOptionsSet.iterations is not positive");
		ASSERT(parameters.kMeansClusteringOptionsSet.convertibleBoundIndex > 0, "FlannMatcher Error: kMeansClusteringOptionsSet.convertibleBoundIndex is not positive");
		}
	else if (parameters.generalOptionsSet.matcherMethod == AUTOTUNED_SEARCH)
		{
		ASSERT(parameters.autotunedOptionsSet.targetPrecision > 0, "FlannMatcher Error: autotunedOptionsSet.targetPrecision is not positive");
		ASSERT(parameters.autotunedOptionsSet.buildWeight > 0, "FlannMatcher Error: autotunedOptionsSet.buildWeight is not positive");
		ASSERT(parameters.autotunedOptionsSet.memoryWeight >= 0, "FlannMatcher Error: autotunedOptionsSet.memoryWeight is negative");
		ASSERT(parameters.autotunedOptionsSet.sampleFraction > 0, "FlannMatcher Error: autotunedOptionsSet.sampleFraction is not positive");
		}
	else if (parameters.generalOptionsSet.matcherMethod == HIERARCHICAL_CLUSTERING)
		{
		ASSERT(parameters.hierarchicalClusteringOptionsSet.numberOfTrees > 0, "FlannMatcher Error: hierarchicalClusteringOptionsSet.numberOfTrees is not positive");
		ASSERT(parameters.hierarchicalClusteringOptionsSet.branching > 0, "FlannMatcher Error: hierarchicalClusteringOptionsSet.branching is not positive");
		ASSERT(parameters.hierarchicalClusteringOptionsSet.leafSize > 0, "FlannMatcher Error: hierarchicalClusteringOptionsSet.leafSize is not positive");
		}
	else if (parameters.generalOptionsSet.matcherMethod == LOCALITY_SENSITIVE_HASHING)
		{
		ASSERT(parameters.localitySensitiveHashingOptionsSet.tableNumber >= 0, "FlannMatcher Error: localitySensitiveHashingOptionsSet.tableNumber is negative");
		ASSERT(parameters.localitySensitiveHashingOptionsSet.keySize >= 0, "FlannMatcher Error: localitySensitiveHashingOptionsSet.keySize is negative");
		ASSERT(parameters.localitySensitiveHashingOptionsSet.multiProbeLevel >= 0, "FlannMatcher Error: localitySensitiveHashingOptionsSet.multiProbeLevel is negative");
		}
	else if (parameters.generalOptionsSet.matcherMethod == COMPOSITE_SEARCH)
		{
		ASSERT(parameters.compositeSearchOptionsSet.numberOfTrees > 0, "FlannMatcher Error: compositeSearchOptionsSet.numberOfTrees is not positive");
		ASSERT(parameters.compositeSearchOptionsSet.branching > 0, "FlannMatcher Error: compositeSearchOptionsSet.branching is not positive");
		ASSERT(parameters.compositeSearchOptionsSet.iterations > 0, "FlannMatcher Error: compositeSearchOptionsSet.iterations is not positive");
		ASSERT(parameters.compositeSearchOptionsSet.convertibleBoundIndex > 0, "FlannMatcher Error: compositeSearchOptionsSet.convertibleBoundIndex is not positive");
		}
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
		YAMLCPP_DFN_ASSIGN(generalOptionsSet.distanceThreshold, float, configurationNode, "DistanceThreshold");
		YAMLCPP_DFN_ASSIGN(generalOptionsSet.numberOfChecks, int, configurationNode, "NumberOfChecks");
		YAMLCPP_DFN_ASSIGN(generalOptionsSet.epsilon, float, configurationNode, "Epsilon");
		YAMLCPP_DFN_ASSIGN(generalOptionsSet.sortedSearch, bool, configurationNode, "SortedSearch");
		YAMLCPP_DFN_ASSIGN_WITH_FUNCTION(generalOptionsSet.matcherMethod, std::string, configurationNode, "MatcherMethod", ConvertToMatcherMethod);
		}
	else if (nodeName == "KdTreeSearchParameters")
		{
		YAMLCPP_DFN_ASSIGN(kdTreeSearchOptionsSet.numberOfTrees, int, configurationNode, "NumberOfTrees");
		}
	else if (nodeName == "KMeansClusteringParameters")
		{
		YAMLCPP_DFN_ASSIGN(kMeansClusteringOptionsSet.branching, int, configurationNode, "Branching");
		YAMLCPP_DFN_ASSIGN(kMeansClusteringOptionsSet.iterations, int, configurationNode, "Iterations");
		YAMLCPP_DFN_ASSIGN_WITH_FUNCTION(kMeansClusteringOptionsSet.centersInitialization, std::string, configurationNode, "CentersInitialization", ConvertToCenterInitializationMethod);
		YAMLCPP_DFN_ASSIGN(kMeansClusteringOptionsSet.convertibleBoundIndex, float, configurationNode, "ConvertibleBoundIndex");
		}
	else if (nodeName == "AutotunedSearchParameters")
		{
		YAMLCPP_DFN_ASSIGN(autotunedOptionsSet.targetPrecision, float, configurationNode, "TargetPrecision");
		YAMLCPP_DFN_ASSIGN(autotunedOptionsSet.buildWeight, float, configurationNode, "BuildWeight");
		YAMLCPP_DFN_ASSIGN(autotunedOptionsSet.memoryWeight, float, configurationNode, "MemoryWeight");
		YAMLCPP_DFN_ASSIGN(autotunedOptionsSet.sampleFraction, float, configurationNode, "SampleFraction");
		}
	else if (nodeName == "HierarchicalClusteringParameters")
		{
		YAMLCPP_DFN_ASSIGN(hierarchicalClusteringOptionsSet.branching, int, configurationNode, "Branching");
		YAMLCPP_DFN_ASSIGN(hierarchicalClusteringOptionsSet.numberOfTrees, int, configurationNode, "NumberOfTrees");
		YAMLCPP_DFN_ASSIGN_WITH_FUNCTION(hierarchicalClusteringOptionsSet.centersInitialization, std::string, configurationNode, "CentersInitialization", ConvertToCenterInitializationMethod);
		YAMLCPP_DFN_ASSIGN(hierarchicalClusteringOptionsSet.leafSize, int, configurationNode, "LeafSize");
		}
	else if (nodeName == "LocalitySensitiveHashingParameters")
		{
		YAMLCPP_DFN_ASSIGN(localitySensitiveHashingOptionsSet.tableNumber, int, configurationNode, "TableNumber");
		YAMLCPP_DFN_ASSIGN(localitySensitiveHashingOptionsSet.keySize, int, configurationNode, "KeySize");
		YAMLCPP_DFN_ASSIGN(localitySensitiveHashingOptionsSet.multiProbeLevel, int, configurationNode, "MultiProbeLevel");
		}
	else if (nodeName == "CompositeSearch")
		{
		YAMLCPP_DFN_ASSIGN(compositeSearchOptionsSet.numberOfTrees, int, configurationNode, "NumberOfTrees");
		YAMLCPP_DFN_ASSIGN(compositeSearchOptionsSet.branching, int, configurationNode, "Branching");
		YAMLCPP_DFN_ASSIGN(compositeSearchOptionsSet.iterations, int, configurationNode, "Iterations");
		YAMLCPP_DFN_ASSIGN_WITH_FUNCTION(compositeSearchOptionsSet.centersInitialization, std::string, configurationNode, "CentersInitialization", ConvertToCenterInitializationMethod);
		YAMLCPP_DFN_ASSIGN(compositeSearchOptionsSet.convertibleBoundIndex, float, configurationNode, "ConvertibleBoundIndex");
		}
	//Ignore everything else
	}

FlannMatcher::MatcherMethod FlannMatcher::ConvertToMatcherMethod(std::string matcherMethod)
	{
	if (matcherMethod == "KdTreeSearch" || matcherMethod == "0")
		{
		return KD_TREE_SEARCH;
		}
	else if (matcherMethod == "KMeansClustering" || matcherMethod == "1")
		{
		return K_MEANS_CLUSTERING;
		}
	else if (matcherMethod == "AutotunedSearch" || matcherMethod == "2")
		{
		return AUTOTUNED_SEARCH;
		}
	else if (matcherMethod == "HierarchichalClustering" || matcherMethod == "3")
		{
		return HIERARCHICAL_CLUSTERING;
		}
	else if (matcherMethod == "LocalitySensitiveHashing" || matcherMethod == "4")
		{
		return LOCALITY_SENSITIVE_HASHING;
		}
	else if (matcherMethod == "CompositeSearch" || matcherMethod == "5")
		{	
		return COMPOSITE_SEARCH;
		}
	else if (matcherMethod == "LinearSearch" || matcherMethod == "6")
		{
		return LINEAR_SEARCH;
		}
	else
		{
		std::string errorString = "FlannMatcher ConfigurationError: matcher method has to be one of ";
		errorString += "{KdTreeSearch, KMeansClustering, AutotunedSearch, HierarchichalClustering, LocalitySensitiveHashing, CompositeSearch, or LinearSearch}";
		ASSERT(false, errorString);
		}
	}

FlannMatcher::CenterInitializationMethod FlannMatcher::ConvertToCenterInitializationMethod(std::string centerInitializationMethod)
	{
	if (centerInitializationMethod == "FlannRandomCenters" || centerInitializationMethod == "0")
		{
		return cvflann::FLANN_CENTERS_RANDOM;
		}
	else if (centerInitializationMethod == "FlannGonzalesCenters" || centerInitializationMethod == "0")
		{
		return cvflann::FLANN_CENTERS_GONZALES ;
		}
	else if (centerInitializationMethod == "FlannKmeanCenters" || centerInitializationMethod == "0")
		{
		return cvflann::FLANN_CENTERS_KMEANSPP ;
		}
	else if (centerInitializationMethod == "FlannGroupwiseCenters" || centerInitializationMethod == "0")
		{
		return cvflann::FLANN_CENTERS_GROUPWISE;
		}
	else if (centerInitializationMethod == "RandomCenters" || centerInitializationMethod == "0")
		{
		return cvflann::CENTERS_RANDOM;
		}
	else if (centerInitializationMethod == "GonzalesCenters" || centerInitializationMethod == "0")
		{
		return cvflann::CENTERS_GONZALES;
		}
	else if (centerInitializationMethod == "KmeansCenters" || centerInitializationMethod == "0")
		{
		return cvflann::CENTERS_KMEANSPP;
		}
	else
		{
		std::string errorString = "FlannMatcher ConfigurationError: center initialization method has to be one of ";
		errorString += "{FlannRandomCenters, FlannGonzalesCenters, FlannKmeanCenters, FlannGroupwiseCenters, RandomCenters, GonzalesCenters, or KmeansCenters}";
		ASSERT(false, errorString);
		}
	}

}


/** @} */
