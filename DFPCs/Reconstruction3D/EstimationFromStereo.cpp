/* --------------------------------------------------------------------------
*
* (C) Copyright …
*
* ---------------------------------------------------------------------------
*/

/*!
 * @file EstimationFromStereo.cpp
 * @date 25/07/2018
 * @author Alessandro Bianco
 */

/*!
 * @addtogroup DFNs
 * 
 * Implementation of the EstimationFromStereo class.
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
#include "EstimationFromStereo.hpp"
#include "Errors/Assert.hpp"
#include <Visualizers/OpencvVisualizer.hpp>
#include <Visualizers/PclVisualizer.hpp>
#include <VisualPointFeatureVector3D.hpp>

#define DELETE_PREVIOUS(object) \
	{ \
	if (object != NULL) \
		{ \
		delete(object); \
		object = NULL; \
		} \
	} \

namespace dfpc_ci {

using namespace dfn_ci;
using namespace FrameWrapper;
using namespace PoseWrapper;
using namespace PointCloudWrapper;
using namespace VisualPointFeatureVector2DWrapper;
using namespace VisualPointFeatureVector3DWrapper;
using namespace CorrespondenceMap2DWrapper;
using namespace MatrixWrapper;
using namespace CorrespondenceMap3DWrapper;
using namespace BaseTypesWrapper;

/* --------------------------------------------------------------------------
 *
 * Public Member Functions
 *
 * --------------------------------------------------------------------------
 */
EstimationFromStereo::EstimationFromStereo() :
	EMPTY_FEATURE_VECTOR( NewVisualPointFeatureVector3D() ),
	LEFT_FEATURE_CATEGORY ( "orb_left" ),
	RIGHT_FEATURE_CATEGORY ( "orb_right" ),
	STEREO_CLOUD_CATEGORY( "stereo_cloud" ),
	TRIANGULATION_CLOUD_CATEGORY( "triangulation_cloud" )
	{
	parametersHelper.AddParameter<float>("GeneralParameters", "PointCloudMapResolution", parameters.pointCloudMapResolution, DEFAULT_PARAMETERS.pointCloudMapResolution);
	parametersHelper.AddParameter<float>("GeneralParameters", "SearchRadius", parameters.searchRadius, DEFAULT_PARAMETERS.searchRadius);
	parametersHelper.AddParameter<int>("GeneralParameters", "NumberOfAdjustedStereoPairs", parameters.numberOfAdjustedStereoPairs, DEFAULT_PARAMETERS.numberOfAdjustedStereoPairs);
	parametersHelper.AddParameter<float>("GeneralParameters", "Baseline", parameters.baseline, DEFAULT_PARAMETERS.baseline);

	currentInputNumber = 0;
	oldestCameraIndex = 0;
	firstTimeBundle = true;

	optionalLeftFilter = NULL;
	optionalRightFilter = NULL;
	reconstructor3d = NULL;
	featuresExtractor2d = NULL;
	optionalFeaturesDescriptor2d = NULL;
	featuresMatcher2d = NULL;
	reconstructor3dfrom2dmatches = NULL;
	transformEstimator = NULL;

	bundleHistory = NULL;

	cleanCorrespondenceMap = NewCorrespondenceMap2D();
	leftTimeCorrespondenceMap = NewCorrespondenceMap2D();
	rightTimeCorrespondenceMap = NewCorrespondenceMap2D();

	workingCorrespondenceMapSequence = NULL;
	historyCorrespondenceMapSequence = NULL;

	configurationFilePath = "";
	}

EstimationFromStereo::~EstimationFromStereo()
	{
	DELETE_PREVIOUS(optionalLeftFilter);
	DELETE_PREVIOUS(optionalRightFilter);
	DELETE_PREVIOUS(reconstructor3d);
	DELETE_PREVIOUS(featuresExtractor2d);
	DELETE_PREVIOUS(optionalFeaturesDescriptor2d);
	DELETE_PREVIOUS(featuresMatcher2d);
	DELETE_PREVIOUS(reconstructor3dfrom2dmatches);
	DELETE_PREVIOUS(transformEstimator);

	DELETE_PREVIOUS(bundleHistory);
	DELETE_PREVIOUS(cleanCorrespondenceMap);
	DELETE_PREVIOUS(leftTimeCorrespondenceMap);
	DELETE_PREVIOUS(rightTimeCorrespondenceMap);
	DELETE_PREVIOUS(workingCorrespondenceMapSequence);
	DELETE_PREVIOUS(historyCorrespondenceMapSequence);
	}


void EstimationFromStereo::run() 
	{
	#ifdef TESTING
	logFile.open("/InFuse/myLog.txt", std::ios::app);
	#endif
	DEBUG_PRINT_TO_LOG("Estimation from stereo start", "");
 
	bundleHistory->AddImages(inLeftImage, inRightImage);

	FrameConstPtr filteredLeftImage = NULL;
	FrameConstPtr filteredRightImage = NULL;
	optionalLeftFilter->Execute(inLeftImage, filteredLeftImage);
	optionalRightFilter->Execute(inRightImage, filteredRightImage);

	ComputeStereoPointCloud(filteredLeftImage, filteredRightImage);
	ComputeVisualPointFeatures(filteredLeftImage, filteredRightImage);
	CreateWorkingCorrespondences();

	Poses3DSequenceConstPtr cameraPoses;
	if (currentInputNumber+1 < parameters.numberOfAdjustedStereoPairs)
		{
		outSuccess = false;
		}
	else
		{
		outSuccess = ComputeCameraPoses(cameraPoses);
		}

	if (!outSuccess && !firstTimeBundle)
		{
		bundleHistory->RemoveEntry(0);
		DELETE_PREVIOUS(workingCorrespondenceMapSequence);
		#ifdef TESTING
		logFile << std::endl;
		logFile.close();
		#endif
		return;
		}

	if (outSuccess)
		{
		if(firstTimeBundle)
			{
			AddAllPointCloudsToMap(cameraPoses);
			firstTimeBundle = false;
			}
		else
			{
			AddLastPointCloudToMap(cameraPoses);
			}

		Copy( pointCloudMap.GetLatestPose(), outPose);
		PointCloudWrapper::PointCloudConstPtr outputPointCloud = pointCloudMap.GetScenePointCloudInOrigin(&outPose, parameters.searchRadius);
		Copy(*outputPointCloud, outPointCloud); 

		DEBUG_SHOW_POINT_CLOUD(outputPointCloud);
		DELETE_PREVIOUS(outputPointCloud);
		}

	DELETE_PREVIOUS(historyCorrespondenceMapSequence);
	historyCorrespondenceMapSequence = workingCorrespondenceMapSequence;
	workingCorrespondenceMapSequence = NULL;
	currentInputNumber++;

	#ifdef TESTING
	logFile << std::endl;
	logFile.close();
	#endif
	}

void EstimationFromStereo::setup()
	{
	configurator.configure(configurationFilePath);
	ConfigureExtraParameters(); //Configuration shall happen before alias assignment here.

	InstantiateDFNExecutors();

	bundleHistory = new BundleHistory(parameters.numberOfAdjustedStereoPairs + 1);
	pointCloudMap.SetResolution(parameters.pointCloudMapResolution);

	SetPosition(rightToLeftCameraPose, -parameters.baseline, 0, 0);
	SetOrientation(rightToLeftCameraPose, 0, 0, 0, 1);
	}

/* --------------------------------------------------------------------------
 *
 * Private Member Variables
 *
 * --------------------------------------------------------------------------
 */

const EstimationFromStereo::EstimationFromStereoOptionsSet EstimationFromStereo::DEFAULT_PARAMETERS = 
	{
	.searchRadius = 20,
	.pointCloudMapResolution = 1e-2,
	.numberOfAdjustedStereoPairs = 4,
	.baseline = 1
	};

/* --------------------------------------------------------------------------
 *
 * Private Member Functions
 *
 * --------------------------------------------------------------------------
 */
void EstimationFromStereo::ConfigureExtraParameters()
	{
	parametersHelper.ReadFile( configurator.GetExtraParametersConfigurationFilePath() );

	ASSERT(parameters.pointCloudMapResolution > 0, "EstimationFromStereo Error, Point Cloud Map resolution is not positive");
	}

void EstimationFromStereo::InstantiateDFNExecutors()
	{
	optionalLeftFilter = new ImageFilteringExecutor( static_cast<ImageFilteringInterface*>( configurator.GetDfn("leftFilter", true) ) );
	optionalRightFilter = new ImageFilteringExecutor( static_cast<ImageFilteringInterface*>( configurator.GetDfn("rightFilter", true) ) );
	reconstructor3d = new StereoReconstructionExecutor( static_cast<StereoReconstructionInterface*>( configurator.GetDfn("reconstructor3d") ) );
	featuresExtractor2d = new FeaturesExtraction2DExecutor( static_cast<FeaturesExtraction2DInterface*>( configurator.GetDfn("featuresExtractor2d") ) );
	optionalFeaturesDescriptor2d = new FeaturesDescription2DExecutor( static_cast<FeaturesDescription2DInterface*>( configurator.GetDfn("featuresDescriptor2d", true) ) );
	featuresMatcher2d = new FeaturesMatching2DExecutor( static_cast<FeaturesMatching2DInterface*>( configurator.GetDfn("featuresMatcher2d") ) );
	reconstructor3dfrom2dmatches = new PointCloudReconstruction2DTo3DExecutor( static_cast<PointCloudReconstruction2DTo3DInterface*>( configurator.GetDfn("reconstructor3dfrom2dmatches", true) ) );
	transformEstimator = new Transform3DEstimationExecutor( static_cast<Transform3DEstimationInterface*>( configurator.GetDfn("transformEstimator") ) );
	}

void EstimationFromStereo::ComputeStereoPointCloud(FrameConstPtr filteredLeftImage, FrameConstPtr filteredRightImage)
	{
	PointCloudConstPtr imageCloud = NULL;
	reconstructor3d->Execute(filteredLeftImage, filteredRightImage, imageCloud);

	bundleHistory->AddPointCloud(*imageCloud, STEREO_CLOUD_CATEGORY);

	DEBUG_PRINT_TO_LOG("Stereo points number", GetNumberOfPoints(*imageCloud));
	//DEBUG_SHOW_POINT_CLOUD(imageCloud);
	}

#define MINIMUM(a, b) ( a < b ? a : b )

void EstimationFromStereo::ComputeVisualPointFeatures(FrameConstPtr filteredLeftImage, FrameConstPtr filteredRightImage)
	{
	VisualPointFeatureVector2DConstPtr keypointVector = NULL;
	VisualPointFeatureVector2DConstPtr featureVector = NULL;
	featuresExtractor2d->Execute(filteredLeftImage, keypointVector);
	optionalFeaturesDescriptor2d->Execute(filteredLeftImage, keypointVector, featureVector);
	bundleHistory->AddFeatures(*featureVector, LEFT_FEATURE_CATEGORY);
	PRINT_TO_LOG("Features Number", GetNumberOfPoints(*featureVector) );

	keypointVector = NULL;
	featureVector = NULL;
	featuresExtractor2d->Execute(filteredRightImage, keypointVector);
	optionalFeaturesDescriptor2d->Execute(filteredRightImage, keypointVector, featureVector);
	bundleHistory->AddFeatures(*featureVector, RIGHT_FEATURE_CATEGORY);
	PRINT_TO_LOG("Features Number", GetNumberOfPoints(*featureVector) );

	#ifdef TESTING
	logFile << "current" << " ";
	#endif

	VisualPointFeatureVector2DConstPtr leftFeatureVector = bundleHistory->GetFeatures(0, LEFT_FEATURE_CATEGORY);
	VisualPointFeatureVector2DConstPtr rightFeatureVector = bundleHistory->GetFeatures(0, RIGHT_FEATURE_CATEGORY);
	CorrespondenceMap2DConstPtr leftRightCorrespondenceMap = NULL;
	featuresMatcher2d->Execute(leftFeatureVector, rightFeatureVector,leftRightCorrespondenceMap);
	PRINT_TO_LOG("Correspondences Number", GetNumberOfCorrespondences(*leftRightCorrespondenceMap) );
	CleanLowScoringMatches(leftRightCorrespondenceMap);
	bundleHistory->AddMatches(*cleanCorrespondenceMap);
	PRINT_TO_LOG("Clean Correspondences Number", GetNumberOfCorrespondences(*cleanCorrespondenceMap) );

	PointCloudConstPtr triangulatedKeypointCloud = NULL;
	reconstructor3dfrom2dmatches->Execute(cleanCorrespondenceMap, &rightToLeftCameraPose, triangulatedKeypointCloud);
	//DEBUG_SHOW_2D_CORRESPONDENCES(filteredLeftImage, filteredRightImage, leftRightCorrespondenceMap);
	bundleHistory->AddPointCloud(*triangulatedKeypointCloud, TRIANGULATION_CLOUD_CATEGORY);
	PRINT_TO_LOG("Triangulated points Number", GetNumberOfPoints(*triangulatedKeypointCloud) );
	}

void EstimationFromStereo::CleanLowScoringMatches(CorrespondenceMap2DConstPtr leftRightCorrespondenceMap)
	{
	Copy(*leftRightCorrespondenceMap, *cleanCorrespondenceMap);

	std::vector<BaseTypesWrapper::T_UInt32> removeIndexList;
	for(int correspondenceIndex1=0; correspondenceIndex1< GetNumberOfCorrespondences(*cleanCorrespondenceMap); correspondenceIndex1++)
		{
		BaseTypesWrapper::Point2D source1 = GetSource(*cleanCorrespondenceMap, correspondenceIndex1);
		BaseTypesWrapper::Point2D sink1 = GetSink(*cleanCorrespondenceMap, correspondenceIndex1);
		if (source1.x != source1.x || source1.y != source1.y || sink1.x != sink1.x || sink1.y != sink1.y)
			{
			removeIndexList.push_back(correspondenceIndex1);
			continue;
			}
		bool found = false;
		for (int correspondenceIndex2=0; correspondenceIndex2<correspondenceIndex1 && !found; correspondenceIndex2++)
			{
			BaseTypesWrapper::Point2D source2 = GetSource(*cleanCorrespondenceMap, correspondenceIndex2);
			BaseTypesWrapper::Point2D sink2 = GetSink(*cleanCorrespondenceMap, correspondenceIndex2);
			if ( (source1.x == source2.x && source1.y == source2.y) || (sink1.x == sink2.x && sink1.y == sink2.y) )
				{
				removeIndexList.push_back(correspondenceIndex1);
				found = true;
				}
			}
		}
	RemoveCorrespondences(*cleanCorrespondenceMap, removeIndexList);
	}

bool EstimationFromStereo::ComputeCameraPoses(Poses3DSequenceConstPtr& cameraPoses)
	{
	cameraPoses = NULL;
	bool success;
	float error;
	transformEstimator->Execute(workingCorrespondenceMapSequence, cameraPoses, success, error);
	
	#ifdef TESTING
	logFile << error << " " << success << " ";
	#endif	
	if (!success)
		{
		return false;
		}

	Pose3D firstPose = GetPose(*cameraPoses, 0);
	bool validPose = GetXOrientation(firstPose) > 0 || GetYOrientation(firstPose) > 0 || GetZOrientation(firstPose) > 0 || GetWOrientation(firstPose) > 0;
	if (validPose)
		{
		DEBUG_PRINT_TO_LOG("The first pose is valid", ToString(firstPose) );
		}
	else
		{
		DEBUG_PRINT_TO_LOG("The first pose is NOT valid", ToString(firstPose) );
		}
	DEBUG_PRINT_TO_LOG("Error:", error);
	#ifdef TESTING
	logFile << error << " " << validPose << " ";
	#endif
	return validPose;
	}

void EstimationFromStereo::AddAllPointCloudsToMap(Poses3DSequenceConstPtr& cameraPoses)
	{
	for(int stereoIndex = 0; stereoIndex < parameters.numberOfAdjustedStereoPairs; stereoIndex++)
		{
		if (stereoIndex == 0)
			{
			Pose3D zeroPose;
			SetPosition(zeroPose, 0, 0, 0);
			SetOrientation(zeroPose, 0, 0, 0, 1);
			pointCloudMap.AddPointCloud( bundleHistory->GetPointCloud(stereoIndex, STEREO_CLOUD_CATEGORY), EMPTY_FEATURE_VECTOR, &zeroPose);
			}
		const Pose3D& pose = GetPose(*cameraPoses, parameters.numberOfAdjustedStereoPairs - 1 - stereoIndex);
		pointCloudMap.AddPointCloud(bundleHistory->GetPointCloud(stereoIndex, STEREO_CLOUD_CATEGORY), EMPTY_FEATURE_VECTOR, &pose);
		}
	}

void EstimationFromStereo::AddLastPointCloudToMap(Poses3DSequenceConstPtr& cameraPoses)
	{
	const Pose3D& poseOfPastCameraInCurrentCamera = GetPose(*cameraPoses, 0);

	//Inverting the pose.
	Pose3D poseOfCurrentCameraInPastCamera;
	SetPosition(poseOfCurrentCameraInPastCamera, -GetXPosition(poseOfPastCameraInCurrentCamera), -GetYPosition(poseOfPastCameraInCurrentCamera), -GetZPosition(poseOfPastCameraInCurrentCamera) );

	float qx = GetXOrientation(poseOfPastCameraInCurrentCamera);
	float qy = GetYOrientation(poseOfPastCameraInCurrentCamera);
	float qz = GetZOrientation(poseOfPastCameraInCurrentCamera);
	float qw = GetWOrientation(poseOfPastCameraInCurrentCamera);
	float squaredNorm = qx*qx + qy*qy + qz*qz + qw*qw;
	SetOrientation(poseOfCurrentCameraInPastCamera, -qx/squaredNorm, -qy/squaredNorm, -qz/squaredNorm, qw/squaredNorm);

	pointCloudMap.AttachPointCloud(bundleHistory->GetPointCloud(0, STEREO_CLOUD_CATEGORY), EMPTY_FEATURE_VECTOR, &poseOfCurrentCameraInPastCamera);
	}


#define CONTINUE_ON_INVALID_2D_POINT(point) \
	if (point.x != point.x || point.y != point.y) \
		{ \
		continue; \
		}
#define CONTINUE_ON_INVALID_3D_POINT(point) \
	if (point.x != point.x || point.y != point.y || point.z != point.z) \
		{ \
		continue; \
		}
#define CONTINUE_ON_DISTINCT_POINTS(point1, point2) \
	if (point1.x != point2.x || point1.y != point2.y) \
		{ \
		continue; \
		}
#ifndef TESTING
	#define PRINT_CORRESPONDENCE(message, source, sink)
#else
	#define PRINT_CORRESPONDENCE(message, source, sink) \
		{ \
		std::stringstream stream; \
		stream << "(" << source.x << ", " << source.y <<") (" << sink.x << ", " << sink.y << ")"; \
		std::string string = stream.str(); \
		DEBUG_PRINT_TO_LOG(message, string); \
		}
#endif 
CorrespondenceMap3DPtr EstimationFromStereo::Extract3DCorrespondencesFromTwoImagePairs(std::vector<CorrespondenceMap2DConstPtr> mapList, std::vector<PointCloudConstPtr> pointCloudList)
	{
	ASSERT( mapList.size() == 4 && pointCloudList.size() == 2, "EstimationFromStereo::Extract3DCorrespondencesFromTwoImagePairs, input do not have expected size");
	CorrespondenceMap3DPtr newCorrespondenceMap = NewCorrespondenceMap3D();
	CorrespondenceMap2DConstPtr leftRightCorrespondenceMap = mapList.at(0);
	CorrespondenceMap2DConstPtr leftTimeCorrespondenceMap = mapList.at(1);
	CorrespondenceMap2DConstPtr rightTimeCorrespondenceMap = mapList.at(2);
	CorrespondenceMap2DConstPtr pastCorrespondenceMap = mapList.at(3);
	PointCloudConstPtr triangulatedKeypointCloud = pointCloudList.at(0);
	PointCloudConstPtr pastKeypointCloud = pointCloudList.at(1);
	for(int correspondenceIndex1 = 0; correspondenceIndex1 < GetNumberOfCorrespondences(*leftRightCorrespondenceMap); correspondenceIndex1++)
		{
		Point2D leftRightSource = GetSource(*leftRightCorrespondenceMap, correspondenceIndex1);
		Point2D leftRightSink = GetSink(*leftRightCorrespondenceMap, correspondenceIndex1);
		CONTINUE_ON_INVALID_2D_POINT(leftRightSource);
		CONTINUE_ON_INVALID_2D_POINT(leftRightSink);
		for(int correspondenceIndex2 = 0; correspondenceIndex2 < GetNumberOfCorrespondences(*leftTimeCorrespondenceMap); correspondenceIndex2++)
			{
			Point2D leftTimeSource = GetSource(*leftTimeCorrespondenceMap, correspondenceIndex2);
			Point2D leftTimeSink = GetSink(*leftTimeCorrespondenceMap, correspondenceIndex2);
			CONTINUE_ON_INVALID_2D_POINT(leftTimeSource);
			CONTINUE_ON_INVALID_2D_POINT(leftTimeSink);
			CONTINUE_ON_DISTINCT_POINTS(leftRightSource, leftTimeSource);
			for(int correspondenceIndex3 = 0; correspondenceIndex3 < GetNumberOfCorrespondences(*rightTimeCorrespondenceMap); correspondenceIndex3++)
				{
				Point2D rightTimeSource = GetSource(*rightTimeCorrespondenceMap, correspondenceIndex3);
				Point2D rightTimeSink = GetSink(*rightTimeCorrespondenceMap, correspondenceIndex3);
				CONTINUE_ON_INVALID_2D_POINT(rightTimeSource);
				CONTINUE_ON_INVALID_2D_POINT(rightTimeSink);
				CONTINUE_ON_DISTINCT_POINTS(leftRightSink, rightTimeSource);
				for(int correspondenceIndex4 = 0; correspondenceIndex4 < GetNumberOfCorrespondences(*pastCorrespondenceMap); correspondenceIndex4++)
					{
					Point2D pastSource = GetSource(*pastCorrespondenceMap, correspondenceIndex4);
					Point2D pastSink = GetSink(*pastCorrespondenceMap, correspondenceIndex4);
					CONTINUE_ON_INVALID_2D_POINT(pastSource);
					CONTINUE_ON_INVALID_2D_POINT(pastSink);
					CONTINUE_ON_DISTINCT_POINTS(pastSource, leftTimeSink);
					CONTINUE_ON_DISTINCT_POINTS(pastSink, rightTimeSink);

					Point3D sourcePoint, sinkPoint;
					sourcePoint.x = GetXCoordinate(*triangulatedKeypointCloud, correspondenceIndex1);
					sourcePoint.y = GetYCoordinate(*triangulatedKeypointCloud, correspondenceIndex1);
					sourcePoint.z = GetZCoordinate(*triangulatedKeypointCloud, correspondenceIndex1);
					sinkPoint.x = GetXCoordinate(*pastKeypointCloud, correspondenceIndex4);
					sinkPoint.y = GetYCoordinate(*pastKeypointCloud, correspondenceIndex4);
					sinkPoint.z = GetZCoordinate(*pastKeypointCloud, correspondenceIndex4);

					CONTINUE_ON_INVALID_3D_POINT(sourcePoint);
					CONTINUE_ON_INVALID_3D_POINT(sinkPoint);
					AddCorrespondence(*newCorrespondenceMap, sourcePoint, sinkPoint, 1);
					}
				}
			}
		}
	return newCorrespondenceMap;
	}

void EstimationFromStereo::CreateWorkingCorrespondences()
	{
	ASSERT(workingCorrespondenceMapSequence == NULL, "workingCorrespondenceMapSequence was not deleted");
	workingCorrespondenceMapSequence = NewCorrespondenceMaps3DSequence();

	VisualPointFeatureVector2DConstPtr leftFeatureVector = bundleHistory->GetFeatures(0, LEFT_FEATURE_CATEGORY);
	VisualPointFeatureVector2DConstPtr rightFeatureVector = bundleHistory->GetFeatures(0, RIGHT_FEATURE_CATEGORY);
	CorrespondenceMap2DConstPtr leftRightCorrespondenceMap = bundleHistory->GetMatches(0);
	PointCloudConstPtr triangulatedKeypointCloud = bundleHistory->GetPointCloud(0, TRIANGULATION_CLOUD_CATEGORY);	

	for(int backwardSteps = 1; backwardSteps < parameters.numberOfAdjustedStereoPairs; backwardSteps++)
		{
		VisualPointFeatureVector2DConstPtr pastLeftFeatureVector = bundleHistory->GetFeatures(backwardSteps, LEFT_FEATURE_CATEGORY);
		VisualPointFeatureVector2DConstPtr pastRightFeatureVector = bundleHistory->GetFeatures(backwardSteps, RIGHT_FEATURE_CATEGORY);
		CorrespondenceMap2DConstPtr pastLeftRightCorrespondenceMap = bundleHistory->GetMatches(backwardSteps);
		PointCloudConstPtr pastTriangulatedKeypointCloud = bundleHistory->GetPointCloud(backwardSteps, TRIANGULATION_CLOUD_CATEGORY);

		if (pastLeftFeatureVector == NULL || pastRightFeatureVector == NULL || pastLeftRightCorrespondenceMap == NULL || pastTriangulatedKeypointCloud == NULL)
			{
			DEBUG_PRINT_TO_LOG("Breaking at", backwardSteps);
			break;
			}

		featuresMatcher2d->Execute(leftFeatureVector, pastLeftFeatureVector, leftTimeCorrespondenceMap);
		featuresMatcher2d->Execute(rightFeatureVector, pastRightFeatureVector, rightTimeCorrespondenceMap);
		std::vector<CorrespondenceMap2DConstPtr> correspondenceMapList = {leftRightCorrespondenceMap, leftTimeCorrespondenceMap, rightTimeCorrespondenceMap, pastLeftRightCorrespondenceMap};
		std::vector<PointCloudConstPtr> pointCloudList = {triangulatedKeypointCloud, pastTriangulatedKeypointCloud};
		#ifdef TESTING
		std::vector<FrameConstPtr> imageList = { 
			bundleHistory->GetLeftImage(0), bundleHistory->GetRightImage(0), bundleHistory->GetLeftImage(backwardSteps), bundleHistory->GetRightImage(backwardSteps) 
			};
		DEBUG_SHOW_QUADRUPLE_2D_CORRESPONDENCES( imageList, correspondenceMapList );
		#endif

		CorrespondenceMap3DPtr newCorrespondenceMap = Extract3DCorrespondencesFromTwoImagePairs(correspondenceMapList, pointCloudList);
		AddCorrespondenceMap(*workingCorrespondenceMapSequence, *newCorrespondenceMap);
		DEBUG_PRINT_TO_LOG("New correspondence map size", GetNumberOfCorrespondences(*newCorrespondenceMap) );
		delete(newCorrespondenceMap);
		} 

	DEBUG_PRINT_TO_LOG("history correspondenceMapSequence size", historyCorrespondenceMapSequence != NULL ? GetNumberOfCorrespondenceMaps(*historyCorrespondenceMapSequence) : 0);
	DEBUG_PRINT_TO_LOG("currentInputNumber", currentInputNumber);

	//Adding the old correspondences to the current ones. All correspondences are taken except those with the oldest left and right images
	if (0 < currentInputNumber && currentInputNumber < parameters.numberOfAdjustedStereoPairs)
		{
		for(int correspondenceIndex = 0; correspondenceIndex < GetNumberOfCorrespondenceMaps(*historyCorrespondenceMapSequence); correspondenceIndex++)
			{
			AddCorrespondenceMap(*workingCorrespondenceMapSequence, GetCorrespondenceMap(*historyCorrespondenceMapSequence, correspondenceIndex));
			}
		}
	else if (currentInputNumber > 0)
		{
		int correspondenceIndex = 0;
		for(int sourceIndex = 0; sourceIndex < (parameters.numberOfAdjustedStereoPairs - 1); sourceIndex++)
			{
			for(int sinkIndex = sourceIndex + 1; sinkIndex < (parameters.numberOfAdjustedStereoPairs - 1); sinkIndex++)
				{
				AddCorrespondenceMap(*workingCorrespondenceMapSequence, GetCorrespondenceMap(*historyCorrespondenceMapSequence, correspondenceIndex));	
				correspondenceIndex++;
				}
			correspondenceIndex += 2;
			}
		}
	#ifdef TESTING
	logFile << "corr" << " ";
	logFile << GetNumberOfCorrespondenceMaps(*workingCorrespondenceMapSequence) << " ";
	for(int i=0; i<GetNumberOfCorrespondenceMaps(*workingCorrespondenceMapSequence); i++)
		{
		const CorrespondenceMap3D& map = GetCorrespondenceMap(*workingCorrespondenceMapSequence, i);
		logFile << GetNumberOfCorrespondences(map) << " ";
		}
	#endif
	}

}



/** @} */
