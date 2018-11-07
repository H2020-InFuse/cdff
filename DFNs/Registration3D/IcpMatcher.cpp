/**
 * @author Alessandro Bianco
 */

/**
 * @addtogroup DFNs
 * @{
 */

#include "IcpMatcher.hpp"

#include <PointCloud.hpp>
#include <PointCloudToPclPointCloudConverter.hpp>
#include <Macros/YamlcppMacros.hpp>
#include <Errors/Assert.hpp>

#include <pcl/registration/icp.h>
#include <yaml-cpp/yaml.h>

#include <pointmatcher/PointMatcher.h>
#include <stdexcept>
#include <fstream>

using namespace Converters;
using namespace PointCloudWrapper;
using namespace PoseWrapper;

namespace CDFF
{
namespace DFN
{
namespace Registration3D
{

IcpMatcher::IcpMatcher()
{
	#define ADD_PARAMETER_WITH_HELPER(type, helperType, groupName, parameterName, parameterVariable) \
		parametersHelper.AddParameter<type, helperType>(groupName, parameterName, parameters.parameterVariable, DEFAULT_PARAMETERS.parameterVariable);

	parametersHelper.AddParameter<bool>("GeneralParameters", "UseDefault", parameters.useDefault, DEFAULT_PARAMETERS.useDefault);
	parametersHelper.AddParameter<bool>("GeneralParameters", "FixRotationByNormalization", parameters.fixRotationByNormalization, DEFAULT_PARAMETERS.fixRotationByNormalization);

	parametersHelper.AddParameter<bool>("DistanceLimitPrefilter", "UseOnSource", parameters.distanceLimitPrefilter.use.useOnSource, DEFAULT_PARAMETERS.distanceLimitPrefilter.use.useOnSource);
	parametersHelper.AddParameter<int>("DistanceLimitPrefilter", "StepOnSource", parameters.distanceLimitPrefilter.use.stepOnSource, DEFAULT_PARAMETERS.distanceLimitPrefilter.use.stepOnSource);
	parametersHelper.AddParameter<bool>("DistanceLimitPrefilter", "UseOnSink", parameters.distanceLimitPrefilter.use.useOnSink, DEFAULT_PARAMETERS.distanceLimitPrefilter.use.useOnSink);
	parametersHelper.AddParameter<int>("DistanceLimitPrefilter", "StepOnSink", parameters.distanceLimitPrefilter.use.stepOnSink, DEFAULT_PARAMETERS.distanceLimitPrefilter.use.stepOnSink);
	ADD_PARAMETER_WITH_HELPER(ThresholdDimension, ThresholdDimensionHelper, "DistanceLimitPrefilter", "ThresholdDimension", distanceLimitPrefilter.thresholdDimension);
	parametersHelper.AddParameter<float>("DistanceLimitPrefilter", "DistanceThreshold", 
		parameters.distanceLimitPrefilter.distanceThreshold, DEFAULT_PARAMETERS.distanceLimitPrefilter.distanceThreshold);
	parametersHelper.AddParameter<bool>("DistanceLimitPrefilter", "RemovePointsWithinThreshold", 
		parameters.distanceLimitPrefilter.removePointsWithinThreshold, DEFAULT_PARAMETERS.distanceLimitPrefilter.removePointsWithinThreshold);

	parametersHelper.AddParameter<bool>("RandomSamplingPrefilter", "UseOnSource", parameters.randomSamplingPrefilter.use.useOnSource, DEFAULT_PARAMETERS.randomSamplingPrefilter.use.useOnSource);
	parametersHelper.AddParameter<int>("RandomSamplingPrefilter", "StepOnSource", parameters.randomSamplingPrefilter.use.stepOnSource, DEFAULT_PARAMETERS.randomSamplingPrefilter.use.stepOnSource);
	parametersHelper.AddParameter<bool>("RandomSamplingPrefilter", "UseOnSink", parameters.randomSamplingPrefilter.use.useOnSink, DEFAULT_PARAMETERS.randomSamplingPrefilter.use.useOnSink);
	parametersHelper.AddParameter<int>("RandomSamplingPrefilter", "StepOnSink", parameters.randomSamplingPrefilter.use.stepOnSink, DEFAULT_PARAMETERS.randomSamplingPrefilter.use.stepOnSink);
	parametersHelper.AddParameter<float>("RandomSamplingPrefilter", "Probability", parameters.randomSamplingPrefilter.probability, DEFAULT_PARAMETERS.randomSamplingPrefilter.probability);

	parametersHelper.AddParameter<bool>("SamplingSurfaceNormalPrefilter", "UseOnSource", 
		parameters.samplingSurfaceNormalPrefilter.use.useOnSource, DEFAULT_PARAMETERS.samplingSurfaceNormalPrefilter.use.useOnSource);
	parametersHelper.AddParameter<int>("SamplingSurfaceNormalPrefilter", "StepOnSource", 
		parameters.samplingSurfaceNormalPrefilter.use.stepOnSource, DEFAULT_PARAMETERS.samplingSurfaceNormalPrefilter.use.stepOnSource);
	parametersHelper.AddParameter<bool>("SamplingSurfaceNormalPrefilter", "UseOnSink", 
		parameters.samplingSurfaceNormalPrefilter.use.useOnSink, DEFAULT_PARAMETERS.samplingSurfaceNormalPrefilter.use.useOnSink);
	parametersHelper.AddParameter<int>("SamplingSurfaceNormalPrefilter", "StepOnSink", 
		parameters.samplingSurfaceNormalPrefilter.use.stepOnSink, DEFAULT_PARAMETERS.samplingSurfaceNormalPrefilter.use.stepOnSink);
	parametersHelper.AddParameter<int>("SamplingSurfaceNormalPrefilter", "NumberOfNeighbours", 
		parameters.samplingSurfaceNormalPrefilter.numberOfNeighbours, DEFAULT_PARAMETERS.samplingSurfaceNormalPrefilter.numberOfNeighbours);
	parametersHelper.AddParameter<bool>("SamplingSurfaceNormalPrefilter", "KeepNormals", 
		parameters.samplingSurfaceNormalPrefilter.keepNormals, DEFAULT_PARAMETERS.samplingSurfaceNormalPrefilter.keepNormals);
	parametersHelper.AddParameter<bool>("SamplingSurfaceNormalPrefilter", "KeepDensities", 
		parameters.samplingSurfaceNormalPrefilter.keepDensities, DEFAULT_PARAMETERS.samplingSurfaceNormalPrefilter.keepDensities);
	parametersHelper.AddParameter<bool>("SamplingSurfaceNormalPrefilter", "KeepEigenValues", 
		parameters.samplingSurfaceNormalPrefilter.keepEigenValues, DEFAULT_PARAMETERS.samplingSurfaceNormalPrefilter.keepEigenValues);
	parametersHelper.AddParameter<bool>("SamplingSurfaceNormalPrefilter", "KeepEigenVectors", 
		parameters.samplingSurfaceNormalPrefilter.keepEigenVectors, DEFAULT_PARAMETERS.samplingSurfaceNormalPrefilter.keepEigenVectors);

	parametersHelper.AddParameter<bool>("TrimmedDistancePostfilter", "Use", parameters.trimmedDistancePostfilter.use, DEFAULT_PARAMETERS.trimmedDistancePostfilter.use);
	parametersHelper.AddParameter<int>("TrimmedDistancePostfilter", "Step", parameters.trimmedDistancePostfilter.step, DEFAULT_PARAMETERS.trimmedDistancePostfilter.step);
	parametersHelper.AddParameter<float>("TrimmedDistancePostfilter", "Ratio", parameters.trimmedDistancePostfilter.ratio, DEFAULT_PARAMETERS.trimmedDistancePostfilter.ratio);

	parametersHelper.AddParameter<int>("GeneralParameters", "KdTreeNumberOfNearestNeighbours", parameters.kdTreeNumberOfNearestNeighbours, DEFAULT_PARAMETERS.kdTreeNumberOfNearestNeighbours);
	parametersHelper.AddParameter<float>("GeneralParameters", "KdTreeMatchingEpsilon", parameters.kdTreeMatchingEpsilon, DEFAULT_PARAMETERS.kdTreeMatchingEpsilon);

	ADD_PARAMETER_WITH_HELPER(MinimizerType, MinimizerTypeHelper, "GeneralParameters", "MinimizerType", minimizerType);

	parametersHelper.AddParameter<int>("GeneralParameters", "MaximumIterations", parameters.maximumIterations, DEFAULT_PARAMETERS.maximumIterations);
	parametersHelper.AddParameter<float>("GeneralParameters", "MaxTranslationDistance", parameters.maxTranslationDistance, DEFAULT_PARAMETERS.maxTranslationDistance);
	parametersHelper.AddParameter<float>("GeneralParameters", "MaxRotationDistance", parameters.maxRotationDistance, DEFAULT_PARAMETERS.maxRotationDistance);
	parametersHelper.AddParameter<int>("GeneralParameters", "SmoothnessLength", parameters.smoothnessLength, DEFAULT_PARAMETERS.smoothnessLength);

	configurationFilePath = "";
	inUseGuess = false;

	Pose3D zeroPose;
	SetPosition(zeroPose, 0, 0, 0);
	SetOrientation(zeroPose, 0, 0, 0, 1);
	lastTransformGuess = transform3DToEigenTransform.Convert(&zeroPose);
}

IcpMatcher::~IcpMatcher()
{
}

void IcpMatcher::configure()
{
	parametersHelper.ReadFile(configurationFilePath);
	ValidateParameters();
	
	if (parameters.useDefault)
		{
		icp.setDefault();
		return;
		}

	std::ifstream file(configurationFilePath.c_str());
	if (!file.good())
		{
		VERIFY(false, "IcpMatcher DFN configuration file not found, using default configuration");
		icp.setDefault();
		}
	else
		{
		SetupIcpMatcher();
		}
	file.close();
}

void IcpMatcher::process()
{
	// Handle empty pointclouds
	if (GetNumberOfPoints(inSourceCloud) == 0 || GetNumberOfPoints(inSinkCloud) == 0)
	{
		outSuccess = false;
		return;
	}

	// Read data from input ports 
	PointMatcher<float>::DataPoints inputSourceCloud = ConvertToDataPoints(inSourceCloud);
	PointMatcher<float>::DataPoints inputSinkCloud = ConvertToDataPoints(inSinkCloud);

	// Process data
	//ValidateInputs(inputSourceCloud, inputSinkCloud);
	Pose3DConstPtr tempTransform;
	if (!inUseGuess)
		{
		tempTransform = ComputeTransform(inputSourceCloud, inputSinkCloud);
		}
	else
		{
		PointMatcher<float>::TransformationParameters transformGuess = transform3DToEigenTransform.Convert(&inTransformGuess);
		tempTransform = ComputeTransform(inputSourceCloud, inputSinkCloud, transformGuess);
		}

	if (tempTransform != NULL)
		{
		Copy(*tempTransform, outTransform);
		delete(tempTransform);
		}
}

IcpMatcher::MinimizerTypeHelper::MinimizerTypeHelper
	(const std::string& parameterName, MinimizerType& boundVariable, const MinimizerType& defaultValue) :
	ParameterHelper(parameterName, boundVariable, defaultValue)
{
}

IcpMatcher::MinimizerType IcpMatcher::MinimizerTypeHelper::Convert(const std::string& minimizerType)
{
	if (minimizerType == "Identity" || minimizerType == "0")
	{
		return MinimizerType::Identity;
	}
	if (minimizerType == "PointToPoint" || minimizerType == "1")
	{
		return MinimizerType::PointToPoint;
	}
	else if (minimizerType == "PointToPlane" || minimizerType == "2")
	{
		return MinimizerType::PointToPlane;
	}
	else
	{
		std::string errorString = "IcpMatcher ConfigurationError: minimizer type has to be one of ";
		errorString += "{Identity, PointToPoint, PointToPlane}";
		ASSERT(false, errorString);
	}
}

IcpMatcher::ThresholdDimensionHelper::ThresholdDimensionHelper
	(const std::string& parameterName, ThresholdDimension& boundVariable, const ThresholdDimension& defaultValue) :
	ParameterHelper(parameterName, boundVariable, defaultValue)
{
}

IcpMatcher::ThresholdDimension IcpMatcher::ThresholdDimensionHelper::Convert(const std::string& thresholdDimension)
{
	if (thresholdDimension == "Radial" || thresholdDimension == "-1")
	{
		return ThresholdDimension::Radial;
	}
	else if (thresholdDimension == "AxisX" || thresholdDimension == "0")
	{
		return ThresholdDimension::AxisX;
	}
	else if (thresholdDimension == "AxisY" || thresholdDimension == "1")
	{
		return ThresholdDimension::AxisY;
	}
	else if (thresholdDimension == "AxisZ" || thresholdDimension == "2")
	{
		return ThresholdDimension::AxisZ;
	}	
	{
		std::string errorString = "IcpMatcher ConfigurationError: distance limit threshold dimension has to be one of ";
		errorString += "{Radial, AxisX, AxisY, AxisZ}";
		ASSERT(false, errorString);
	}
}

const IcpMatcher::IcpOptionsSet IcpMatcher::DEFAULT_PARAMETERS =
{
	/*.useDefault=*/ true,
	/*.fixRotationByNormalization=*/ false,
	/*.distanceLimitPrefilter=*/
	{
	/*.use=*/
		{
		/*.useOnSource =*/ false,
		/*.stepOnSource =*/ 0,
		/*.useOnSink =*/ false,
		/*.stepOnSink =*/ 0
		},
	/*.thresholdDimension =*/ ThresholdDimension::Radial,
	/*.distanceThreshold =*/ 1.0,
	/*.removePointsWithinThreshold =*/ true
	},

	/*.randomSamplingPrefilter=*/
	{
	/*.use=*/
		{
		/*.useOnSource =*/ true,
		/*.stepOnSource =*/ 1,
		/*.useOnSink =*/ false,
		/*.stepOnSink =*/ 0
		},
	/*.probability =*/ 0.75
	},

	/*.samplingSurfaceNormalPrefilter=*/
	{
	/*.use=*/
		{
		/*.useOnSource =*/ false,
		/*.stepOnSource =*/ 0,
		/*.useOnSink =*/ true,
		/*.stepOnSink =*/ 1
		},
	/*.numberOfNeighbours =*/ 5,
	/*.keepNormals =*/ true,
	/*.keepDensities =*/ false,
	/*.keepEigenValues =*/ false,
	/*.keepEigenVectors =*/ false
	},

	/*.trimmedDistancePostfilter=*/
	{
	/*.use=*/ true,
	/*.step=*/ 1,
	/*.ratio =*/ 0.85
	},

	/*.kdTreeNumberOfNearestNeighbours =*/ 1,
	/*.kdTreeMatchingEpsilon =*/ 3.16,

	/*.minimizerType =*/ MinimizerType::PointToPlane, 

	/*.maximumIterations =*/ 40,
	/*.maxTranslationDistance =*/ 0.001,
	/*.maxRotationDistance =*/ 0.01,
	/*.smoothnessLength =*/ 4
};

PointMatcher<float>::DataPoints IcpMatcher::ConvertToDataPoints(const PointCloudWrapper::PointCloud& cloud)
	{
	int numberOfPoints = GetNumberOfPoints(cloud);
	PointMatcher<float>::Matrix cloudMatrix(4, numberOfPoints);
	for(int pointIndex = 0; pointIndex < numberOfPoints; pointIndex++)
		{
		cloudMatrix(0, pointIndex) = GetXCoordinate(cloud, pointIndex);
		cloudMatrix(1, pointIndex) = GetYCoordinate(cloud, pointIndex);
		cloudMatrix(2, pointIndex) = GetZCoordinate(cloud, pointIndex);
		cloudMatrix(3, pointIndex) = 1;
		}

	PointMatcher<float>::DataPoints::Labels labels;
	labels.push_back( PointMatcher<float>::DataPoints::Label("x", 1)); 
	labels.push_back( PointMatcher<float>::DataPoints::Label("y", 1)); 
	labels.push_back( PointMatcher<float>::DataPoints::Label("z", 1)); 
	labels.push_back( PointMatcher<float>::DataPoints::Label("w", 1)); 

	PointMatcher<float>::DataPoints conversion(cloudMatrix, labels);

	return conversion;
	}

PoseWrapper::Pose3D IcpMatcher::ConvertToPose3D(PointMatcher<float>::TransformationParameters transform)
	{
	PoseWrapper::Pose3D conversion;

	return conversion;
	}

Pose3DConstPtr IcpMatcher::ComputeTransform(PointMatcher<float>::DataPoints sourceCloud, PointMatcher<float>::DataPoints sinkCloud)
	{
	PointMatcher<float>::TransformationParameters transform = icp(sourceCloud, sinkCloud);

	FixTransformationMatrix(transform);
	return eigenTransformToTransform3D.Convert(transform);
	}

Pose3DConstPtr IcpMatcher::ComputeTransform(PointMatcher<float>::DataPoints sourceCloud, PointMatcher<float>::DataPoints sinkCloud, PointMatcher<float>::TransformationParameters transformGuess)
	{
	PointMatcher<float>::TransformationParameters transform;
	Pose3DConstPtr returnPose = NULL;
	try 
		{
		if (!FixTransformationMatrix(transformGuess))
			{
			throw std::exception();
			}
		transform = icp(sourceCloud, sinkCloud, transformGuess);

		if (!FixTransformationMatrix(transform))
			{
			throw std::exception();
			}
		returnPose = eigenTransformToTransform3D.Convert(transform);
		lastTransformGuess = transformGuess;
		} 
	catch( ... )
		{
		VERIFY(false, "Warning: ICP failure with current transform guess, trying to use the previous guess");
		transform = icp(sourceCloud, sinkCloud, lastTransformGuess);
		if (returnPose != NULL)
			{
			delete returnPose;
			}

		if (!FixTransformationMatrix(transform))
			{
			outSuccess = false;
			return NULL;
			}
		returnPose = eigenTransformToTransform3D.Convert(transform);
		}

	outSuccess = true;
	return returnPose;
	}

void IcpMatcher::SetupIcpMatcher()
	{
	//Defining convinient alias for data types
	typedef std::shared_ptr<PointMatcher<float>::DataPointsFilter> SharedDataPointsFilter;
	typedef std::shared_ptr<PointMatcher<float>::Matcher> SharedMatcher;
	typedef std::shared_ptr<PointMatcher<float>::ErrorMinimizer> SharedErrorMinimizer;
	typedef std::shared_ptr<PointMatcher<float>::OutlierFilter> SharedOutlierFilter;
	typedef std::shared_ptr<PointMatcher<float>::Inspector> SharedInspector;
	typedef std::shared_ptr<PointMatcher<float>::TransformationChecker> SharedTransformationChecker;
	typedef std::shared_ptr<PointMatcher<float>::Transformation> SharedTransformation;

	//Cleaning previous configuration, if any.
	icp.readingDataPointsFilters.clear();
	icp.referenceDataPointsFilters.clear();
	icp.outlierFilters.clear();
	icp.transformationCheckers.clear();
	icp.transformations.clear();

	//Initializing counters
	const int NumberOfPrefilters = 3;
	int sourcePrefilterCount = 0;
	int sinkPrefilterCount = 0;
	sourcePrefilterCount += (parameters.distanceLimitPrefilter.use.useOnSource ? 0 : 1);
	sinkPrefilterCount += (parameters.distanceLimitPrefilter.use.useOnSink ? 0 : 1);
	sourcePrefilterCount += (parameters.randomSamplingPrefilter.use.useOnSource ? 0 : 1);
	sinkPrefilterCount += (parameters.randomSamplingPrefilter.use.useOnSink ? 0 : 1);
	sourcePrefilterCount += (parameters.samplingSurfaceNormalPrefilter.use.useOnSource ? 0 : 1);
	sinkPrefilterCount += (parameters.samplingSurfaceNormalPrefilter.use.useOnSink ? 0 : 1);

	//Setting source prefilters
	int nextSourceStep = 0;
	while(sourcePrefilterCount < NumberOfPrefilters && nextSourceStep < 100)
		{
		if (parameters.distanceLimitPrefilter.use.useOnSource && parameters.distanceLimitPrefilter.use.stepOnSource == nextSourceStep)
			{
			PointMatcherSupport::Parametrizable::Parameters configuration;
			configuration["dim"] = (parameters.distanceLimitPrefilter.thresholdDimension == ThresholdDimension::Radial ? "-1" : 
				(parameters.distanceLimitPrefilter.thresholdDimension == ThresholdDimension::AxisX ? "0" :
					(parameters.distanceLimitPrefilter.thresholdDimension == ThresholdDimension::AxisY ? "1" : "2")
				)
			);
			configuration["dist"] = std::to_string(parameters.distanceLimitPrefilter.distanceThreshold);
			configuration["removeInside"] = (parameters.distanceLimitPrefilter.removePointsWithinThreshold ? "1" : "0");
			SharedDataPointsFilter minimumDistanceSourceFilter = PointMatcher<float>::get().DataPointsFilterRegistrar.create("DistLimitFilter", configuration);
			icp.readingDataPointsFilters.push_back(minimumDistanceSourceFilter);

			sourcePrefilterCount++;
			}
		else if (parameters.randomSamplingPrefilter.use.useOnSource && parameters.randomSamplingPrefilter.use.stepOnSource == nextSourceStep)
			{
			PRINT_TO_LOG("setting random filter on source", parameters.randomSamplingPrefilter.probability);
			PointMatcherSupport::Parametrizable::Parameters configuration;
			configuration["prob"] = std::to_string(parameters.randomSamplingPrefilter.probability);
			SharedDataPointsFilter randomSamplingSourcePrefilter = PointMatcher<float>::get().DataPointsFilterRegistrar.create("RandomSamplingDataPointsFilter", configuration);
			icp.readingDataPointsFilters.push_back(randomSamplingSourcePrefilter);

			sourcePrefilterCount++;
			}
		else if (parameters.samplingSurfaceNormalPrefilter.use.useOnSource && parameters.samplingSurfaceNormalPrefilter.use.stepOnSource == nextSourceStep)
			{
			PointMatcherSupport::Parametrizable::Parameters configuration;
			configuration["knn"] = std::to_string(parameters.samplingSurfaceNormalPrefilter.numberOfNeighbours);
			configuration["keepNormals"] = (parameters.samplingSurfaceNormalPrefilter.keepNormals ? "1" : "0");
			configuration["keepDensities"] = (parameters.samplingSurfaceNormalPrefilter.keepDensities ? "1" : "0");
			configuration["keepEigenValues"] = (parameters.samplingSurfaceNormalPrefilter.keepEigenValues ? "1" : "0");
			configuration["keepEigenVectors"] = (parameters.samplingSurfaceNormalPrefilter.keepEigenVectors ? "1" : "0");
			SharedDataPointsFilter randomSamplingSourcePrefilter = PointMatcher<float>::get().DataPointsFilterRegistrar.create("SamplingSurfaceNormalDataPointsFilter", configuration);
			icp.readingDataPointsFilters.push_back(randomSamplingSourcePrefilter);

			sourcePrefilterCount++;
			}
		nextSourceStep++;
		}

	//Setting sink prefilters
	int nextSinkStep = 0;
	while(sinkPrefilterCount < NumberOfPrefilters && nextSinkStep < 100)
		{
		if (parameters.distanceLimitPrefilter.use.useOnSink && parameters.distanceLimitPrefilter.use.stepOnSink == nextSinkStep)
			{
			PointMatcherSupport::Parametrizable::Parameters configuration;
			configuration["dim"] = (parameters.distanceLimitPrefilter.thresholdDimension == ThresholdDimension::Radial ? "-1" : 
				(parameters.distanceLimitPrefilter.thresholdDimension == ThresholdDimension::AxisX ? "0" :
					(parameters.distanceLimitPrefilter.thresholdDimension == ThresholdDimension::AxisY ? "1" : "2")
				)
			);
			configuration["dist"] = std::to_string(parameters.distanceLimitPrefilter.distanceThreshold);
			configuration["removeInside"] = (parameters.distanceLimitPrefilter.removePointsWithinThreshold ? "1" : "0");
			SharedDataPointsFilter minimumDistanceSourceFilter = PointMatcher<float>::get().DataPointsFilterRegistrar.create("DistLimitFilter", configuration);
			icp.referenceDataPointsFilters.push_back(minimumDistanceSourceFilter);

			sinkPrefilterCount++;
			}
		else if (parameters.randomSamplingPrefilter.use.useOnSink && parameters.randomSamplingPrefilter.use.stepOnSink == nextSinkStep)
			{
			PointMatcherSupport::Parametrizable::Parameters configuration;
			configuration["prob"] = std::to_string(parameters.randomSamplingPrefilter.probability);
			SharedDataPointsFilter randomSamplingSourcePrefilter = PointMatcher<float>::get().DataPointsFilterRegistrar.create("RandomSamplingDataPointsFilter", configuration);
			icp.referenceDataPointsFilters.push_back(randomSamplingSourcePrefilter);

			sinkPrefilterCount++;
			}
		else if (parameters.samplingSurfaceNormalPrefilter.use.useOnSink && parameters.samplingSurfaceNormalPrefilter.use.stepOnSink == nextSinkStep)
			{
			PRINT_TO_LOG("setting surface normal on sink", parameters.samplingSurfaceNormalPrefilter.keepNormals);
			PointMatcherSupport::Parametrizable::Parameters configuration;
			configuration["knn"] = std::to_string(parameters.samplingSurfaceNormalPrefilter.numberOfNeighbours);
			configuration["keepNormals"] = (parameters.samplingSurfaceNormalPrefilter.keepNormals ? "1" : "0");
			configuration["keepDensities"] = (parameters.samplingSurfaceNormalPrefilter.keepDensities ? "1" : "0");
			configuration["keepEigenValues"] = (parameters.samplingSurfaceNormalPrefilter.keepEigenValues ? "1" : "0");
			configuration["keepEigenVectors"] = (parameters.samplingSurfaceNormalPrefilter.keepEigenVectors ? "1" : "0");
			SharedDataPointsFilter randomSamplingSourcePrefilter = PointMatcher<float>::get().DataPointsFilterRegistrar.create("SamplingSurfaceNormalDataPointsFilter", configuration);
			icp.referenceDataPointsFilters.push_back(randomSamplingSourcePrefilter);

			sinkPrefilterCount++;
			}
		nextSinkStep++;
		}

	//Setting matcher
		{
		PointMatcherSupport::Parametrizable::Parameters configuration;
		configuration["knn"] = std::to_string(parameters.kdTreeNumberOfNearestNeighbours);
		configuration["epsilon"] = std::to_string(parameters.kdTreeMatchingEpsilon);
		SharedMatcher kdtree = PointMatcher<float>::get().MatcherRegistrar.create("KDTreeMatcher", configuration);
		icp.matcher = kdtree;
		}

	//Setting postfilters
	if (parameters.trimmedDistancePostfilter.use)
		{
		PointMatcherSupport::Parametrizable::Parameters configuration;
		configuration["ratio"] = std::to_string(parameters.trimmedDistancePostfilter.ratio);
		SharedOutlierFilter trimmedDistanceFilter = PointMatcher<float>::get().OutlierFilterRegistrar.create("TrimmedDistOutlierFilter", configuration);
		icp.outlierFilters.push_back(trimmedDistanceFilter);
		}

	//Setting error minimizer
	if (parameters.minimizerType == MinimizerType::Identity)
		{
		SharedErrorMinimizer errorMinimizer = PointMatcher<float>::get().ErrorMinimizerRegistrar.create("IdentityErrorMinimizer");
		icp.errorMinimizer = errorMinimizer;
		}
	if (parameters.minimizerType == MinimizerType::PointToPoint)
		{
		SharedErrorMinimizer errorMinimizer = PointMatcher<float>::get().ErrorMinimizerRegistrar.create("PointToPointErrorMinimizer");
		icp.errorMinimizer = errorMinimizer;
		}
	else if (parameters.minimizerType == MinimizerType::PointToPlane)
		{
		SharedErrorMinimizer errorMinimizer = PointMatcher<float>::get().ErrorMinimizerRegistrar.create("PointToPlaneErrorMinimizer");
		icp.errorMinimizer = errorMinimizer;
		}

	//Setting transforms checkers
		{
		PointMatcherSupport::Parametrizable::Parameters configuration;
		configuration["maxIterationCount"] = std::to_string(parameters.maximumIterations);
		SharedTransformationChecker maxIterationChecker = PointMatcher<float>::get().TransformationCheckerRegistrar.create("CounterTransformationChecker", configuration);
		icp.transformationCheckers.push_back(maxIterationChecker);
		}

		{
		PointMatcherSupport::Parametrizable::Parameters configuration;
		configuration["minDiffRotErr"] = std::to_string(parameters.maxRotationDistance);
		configuration["minDiffTransErr"] = std::to_string(parameters.maxTranslationDistance);
		configuration["smoothLength"] = std::to_string(parameters.smoothnessLength);
		SharedTransformationChecker maxDistanceChecker = PointMatcher<float>::get().TransformationCheckerRegistrar.create("DifferentialTransformationChecker", configuration);
		icp.transformationCheckers.push_back(maxDistanceChecker);
		}

	//Setting inspector
		{
		SharedInspector emptyIspector = PointMatcher<float>::get().InspectorRegistrar.create("NullInspector");
		icp.inspector = emptyIspector;
		}

	//Setting transformations
		{
		SharedTransformation rigidTransformation = PointMatcher<float>::get().TransformationRegistrar.create("RigidTransformation");
		icp.transformations.push_back(rigidTransformation);		
		}
	}

bool IcpMatcher::FixTransformationMatrix(PointMatcher<float>::TransformationParameters& transform)
	{
	Eigen::Matrix3f eigenRotationMatrix = transform.block(0,0,3,3);
	float determinant = eigenRotationMatrix.determinant();
	if (std::abs(determinant) < 1.00001 && std::abs(determinant) > 0.99999)
		{
		return true;
		}
	else if (std::abs(determinant) < 0.00001)
		{
		return false;
		}
	PRINT_TO_LOG("Bad Transform", transform);

	if (parameters.fixRotationByNormalization)
		{
		return NormalizeRotationMatrix(transform, determinant);
		}
	else
		{
		return FixSingleRotationElement(transform, determinant);
		}
	}

/*
* Changes a value of the rotation matrix if the determinant is not sufficiently close to 1, so that the determinant becomes 1.
*
*
*/
bool IcpMatcher::FixSingleRotationElement(PointMatcher<float>::TransformationParameters& transform, float determinant)
	{
	struct Term
		{
		float element;
		float coefficient;
		float rest;
		};

	//This list contains an entry for each element of the matrix: element is the element itself, coefficient is the derivative of the determinant with respect to the element
	// rest is the rest of the determinant when written as: determinant = element * coefficient + rest.
	Term termList[9];
	for(int rowIndex = 0; rowIndex<3; rowIndex++)
		{
		for (int columnIndex=0; columnIndex<3; columnIndex++)
			{
			termList[3*rowIndex+columnIndex].element = transform(rowIndex,columnIndex);
			//The derivative is the determinant of the submatrix (up to a sign) that does not contain the element.
			float submatrix[4];
			int fillingIndex = 0;
			for(int subRowIndex = 0; subRowIndex < 3; subRowIndex++)
				{
				for(int subColumnIndex = 0; subColumnIndex < 3; subColumnIndex++)
					{
					if (subRowIndex != rowIndex && subColumnIndex != columnIndex)
						{
						submatrix[fillingIndex] = transform(subRowIndex, subColumnIndex);
						fillingIndex++;
						}
					}
				}
			termList[3*rowIndex+columnIndex].coefficient = submatrix[0]*submatrix[3] - submatrix[1]*submatrix[2];
			//The coefficient is the opposite of the determinant of the submatrix, if the sum of the element indices is odd.
			if ( (rowIndex+columnIndex) % 2 == 1)
				{
				termList[3*rowIndex+columnIndex].coefficient = - termList[3*rowIndex+columnIndex].coefficient;
				}
			}
		}
	//we Compute the rest of the determinant for each element.
	for(int termIndex = 0; termIndex<9; termIndex++)
		{
		int leftoverList[2];
		int fillingIndex=0;
		for(int leftoverIndex = 3*(termIndex/3); leftoverIndex < 3*(termIndex/3+1); leftoverIndex++)
			{
			if ( leftoverIndex != termIndex)
				{
				leftoverList[fillingIndex] = leftoverIndex;
				fillingIndex++;
				}
			}
		termList[termIndex].rest = termList[ leftoverList[0] ].element * termList[ leftoverList[0] ].coefficient + termList[ leftoverList[1] ].element * termList[ leftoverList[1] ].coefficient;
		}

	//For each element we compute the substitue as the one that would allow the determinant to be 1: sub = (1 - rest) / coefficient;
	//We take the substitute that would cause the smallest change.
	bool substituteSet = false;	
	float substituteElement;
	int substituteIndex;
	float minimumDifference;
	for(int termIndex = 0; termIndex < 9; termIndex++)
		{
		if ( std::abs(termList[termIndex].coefficient) > 0)
			{
			float candidate = (1 - termList[termIndex].rest) / termList[termIndex].coefficient;
			float difference = std::abs(termList[termIndex].element - candidate);
			if (!substituteSet || difference < minimumDifference)
				{
				substituteSet = true;
				minimumDifference = difference;
				substituteElement = candidate;
				substituteIndex = termIndex;
				}
			}
		}

	ASSERT(substituteSet, "FixTransformationMatrix error, matrix should have zero determinant, but this escaped the check at the start of the method. Look for bugs!");
	transform(substituteIndex/3, substituteIndex%3) = substituteElement;

	PRINT_TO_LOG("Transform fixed", transform);
	return true;
	}

bool IcpMatcher::NormalizeRotationMatrix(PointMatcher<float>::TransformationParameters& transform, float determinant)
	{
	for(int rowIndex = 0; rowIndex < 3; rowIndex++)
		{
		for(int columnIndex = 0; columnIndex < 3; columnIndex++)
			{
			transform(rowIndex, columnIndex) = transform(rowIndex, columnIndex) / determinant;
			}
		}
	return true;
	}

void IcpMatcher::ValidateParameters()
{
	ASSERT(parameters.distanceLimitPrefilter.distanceThreshold >= 0, "IcpMatcher Configuration error, distanceLimitPrefilter.distanceThreshold is negative");

	ASSERT(parameters.randomSamplingPrefilter.probability >= 0, "IcpMatcher Configuration error, randomSamplingPrefilter.probability is negative");
	ASSERT(parameters.randomSamplingPrefilter.probability <= 1, "IcpMatcher Configuration error, randomSamplingPrefilter.probability is greater than 1");

	ASSERT(parameters.samplingSurfaceNormalPrefilter.numberOfNeighbours >= 1, "IcpMatcher Configuration error, samplingSurfaceNormalPrefilter.numberOfNeighbours is not positive");

	ASSERT(parameters.trimmedDistancePostfilter.ratio >= 0, "IcpMatcher Configuration error, trimmedDistanceRatio is negative");
	ASSERT(parameters.trimmedDistancePostfilter.ratio <= 1, "IcpMatcher Configuration error, trimmedDistanceRatio is greater than 1");

	ASSERT(parameters.kdTreeNumberOfNearestNeighbours >= 1, "IcpMatcher Configuration error, kdTreeNumberOfNearestNeighbours is null or negative");

	ASSERT(parameters.maximumIterations >= 1, "IcpMatcher Configuration error, maximumIterations is null or negative");
	ASSERT(parameters.maxTranslationDistance >= 0, "IcpMatcher Configuration error, maxTranslationDistance is null or negative");
	ASSERT(parameters.maxRotationDistance >= 0, "IcpMatcher Configuration error, maxRotationDistance is null or negative");
	ASSERT(parameters.smoothnessLength >= 0, "IcpMatcher Configuration error, smoothnessLength is null or negative");

	//Initializiong validation counters
	const int NumberOfFilters = 3;
	int sourcePrefilterCount = 0;
	int sinkPrefilterCount = 0;
	sourcePrefilterCount += (parameters.distanceLimitPrefilter.use.useOnSource ? 0 : 1);
	sinkPrefilterCount += (parameters.distanceLimitPrefilter.use.useOnSink ? 0 : 1);
	sourcePrefilterCount += (parameters.randomSamplingPrefilter.use.useOnSource ? 0 : 1);
	sinkPrefilterCount += (parameters.randomSamplingPrefilter.use.useOnSink ? 0 : 1);
	sourcePrefilterCount += (parameters.samplingSurfaceNormalPrefilter.use.useOnSource ? 0 : 1);
	sinkPrefilterCount += (parameters.samplingSurfaceNormalPrefilter.use.useOnSink ? 0 : 1);

	//Validating source prefilter steps
	int nextSourceStep = 0;
	while(sourcePrefilterCount < NumberOfFilters && nextSourceStep < 100)
		{
		bool stepFound = false;
		if (parameters.distanceLimitPrefilter.use.useOnSource && parameters.distanceLimitPrefilter.use.stepOnSource == nextSourceStep)
			{
			ASSERT(!stepFound, "IcpMatcher Configuration error: two source prefilters have the same step");
			sourcePrefilterCount++;
			stepFound = true;
			}
		if (parameters.randomSamplingPrefilter.use.useOnSource && parameters.randomSamplingPrefilter.use.stepOnSource == nextSourceStep)
			{
			ASSERT(!stepFound, "IcpMatcher Configuration error: two source prefilters have the same step");
			sourcePrefilterCount++;
			stepFound = true;
			}
		if (parameters.samplingSurfaceNormalPrefilter.use.useOnSource && parameters.samplingSurfaceNormalPrefilter.use.stepOnSource == nextSourceStep)
			{
			ASSERT(!stepFound, "IcpMatcher Configuration error: two source prefilters have the same step");
			sourcePrefilterCount++;
			stepFound = true;
			}
		nextSourceStep++;
		}
	ASSERT(sourcePrefilterCount == NumberOfFilters, "IcpMatcher Configuration error: a source prefilter step in not in range [0, 99]");

	//Validating sink prefilter steps
	int nextSinkStep = 0;
	while(sinkPrefilterCount < NumberOfFilters && nextSinkStep < 100)
		{
		bool stepFound = false;
		if (parameters.distanceLimitPrefilter.use.useOnSink && parameters.distanceLimitPrefilter.use.stepOnSink == nextSinkStep)
			{
			ASSERT(!stepFound, "IcpMatcher Configuration error: two sink prefilters have the same step");
			sinkPrefilterCount++;
			stepFound = true;
			}
		if (parameters.randomSamplingPrefilter.use.useOnSink && parameters.randomSamplingPrefilter.use.stepOnSink == nextSinkStep)
			{
			ASSERT(!stepFound, "IcpMatcher Configuration error: two sink prefilters have the same step");
			sinkPrefilterCount++;
			stepFound = true;
			}
		if (parameters.samplingSurfaceNormalPrefilter.use.useOnSink && parameters.samplingSurfaceNormalPrefilter.use.stepOnSink == nextSinkStep)
			{
			ASSERT(!stepFound, "IcpMatcher Configuration error: two sink prefilters have the same step");
			sinkPrefilterCount++;
			stepFound = true;
			}
		nextSinkStep++;
		}
	ASSERT(sinkPrefilterCount == NumberOfFilters, "IcpMatcher Configuration error: a sink prefilter step in not in range [0, 99]");
}

void IcpMatcher::ValidateInputs(pcl::PointCloud<pcl::PointXYZ>::ConstPtr sourceCloud, pcl::PointCloud<pcl::PointXYZ>::ConstPtr sinkCloud)
{
	ValidateCloud(sourceCloud);
	ValidateCloud(sinkCloud);
}

void IcpMatcher::ValidateCloud(pcl::PointCloud<pcl::PointXYZ>::ConstPtr cloud)
{
	for (unsigned pointIndex = 0; pointIndex < cloud->points.size(); pointIndex++)
	{
		const pcl::PointXYZ& point = cloud->points.at(pointIndex);
		ASSERT_EQUAL(point.x, point.x, "IcpMatcher Error, Cloud contains an NaN point");
		ASSERT_EQUAL(point.y, point.y, "IcpMatcher Error, Cloud contains an NaN point");
		ASSERT_EQUAL(point.z, point.z, "IcpMatcher Error, Cloud contains an NaN point");
	}
}

}
}
}

/** @} */

