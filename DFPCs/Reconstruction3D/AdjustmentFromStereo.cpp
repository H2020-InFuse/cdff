/* --------------------------------------------------------------------------
*
* (C) Copyright …
*
* ---------------------------------------------------------------------------
*/

/*!
 * @file AdjustmentFromStereo.cpp
 * @date 15/06/2018
 * @author Alessandro Bianco
 */

/*!
 * @addtogroup DFNs
 * 
 * Implementation of the AdjustmentFromStereo class.
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
#include "AdjustmentFromStereo.hpp"
#include "Errors/Assert.hpp"
#include <Visualizers/OpencvVisualizer.hpp>
#include <Visualizers/PclVisualizer.hpp>
#include <VisualPointFeatureVector3D.hpp>

namespace CDFF
{
namespace DFPC
{
namespace Reconstruction3D
{

using namespace CDFF::DFN;
using namespace FrameWrapper;
using namespace PoseWrapper;
using namespace PointCloudWrapper;
using namespace VisualPointFeatureVector2DWrapper;
using namespace VisualPointFeatureVector3DWrapper;
using namespace CorrespondenceMap2DWrapper;
using namespace MatrixWrapper;

/* --------------------------------------------------------------------------
 *
 * Public Member Functions
 *
 * --------------------------------------------------------------------------
 */
AdjustmentFromStereo::AdjustmentFromStereo() :
	EMPTY_FEATURE_VECTOR( NewVisualPointFeatureVector3D() ),
	LEFT_FEATURE_CATEGORY ( "orb_left" ),
	RIGHT_FEATURE_CATEGORY ( "orb_right" ),
	STEREO_CLOUD_CATEGORY( "stereo_cloud" ),
	TRIANGULATION_CLOUD_CATEGORY( "triangulation_cloud" )
	{
	parametersHelper.AddParameter<float>("GeneralParameters", "PointCloudMapResolution", parameters.pointCloudMapResolution, DEFAULT_PARAMETERS.pointCloudMapResolution);
	parametersHelper.AddParameter<float>("GeneralParameters", "SearchRadius", parameters.searchRadius, DEFAULT_PARAMETERS.searchRadius);
	parametersHelper.AddParameter<int>("GeneralParameters", "NumberOfAdjustedStereoPairs", parameters.numberOfAdjustedStereoPairs, DEFAULT_PARAMETERS.numberOfAdjustedStereoPairs);
	parametersHelper.AddParameter<bool>("GeneralParameters", "UseBundleInitialEstimation", parameters.useBundleInitialEstimation, DEFAULT_PARAMETERS.useBundleInitialEstimation);
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
	bundleAdjuster = NULL;
	fundamentalMatrixComputer = NULL;
	perspectiveNPointSolver = NULL;
	reconstructor3dfrom2dmatches = NULL;

	bundleHistory = NULL;
	correspondencesRecorder = NULL;

	cleanCorrespondenceMap = NewCorrespondenceMap2D();
	triangulatedKeypointCloud = NewPointCloud();

	estimatedCameraPoses = NULL;
	presentKeypointVector = NULL;
	keypointCloud = NULL;

	configurationFilePath = "";

	#ifdef TESTING
	logFile.open("/InFuse/myLog.txt");
	logFile << "denseCloud leftKeypoints leftFeatures rightKeypoints rightFeatures correspondences FMFiltersuccess inlierCorrespondences sparseCloud ";
	logFile << "Correspondences error success validPose X Y Z QX QY QZ QW output X Y Z QX QY QZ QW cloud SpaceX SpaceY SpaceZ" << std::endl;
	logFile.close();
	#endif
	}

AdjustmentFromStereo::~AdjustmentFromStereo()
	{
	delete(optionalLeftFilter);
	delete(optionalRightFilter);
	delete(reconstructor3d);
	delete(featuresExtractor2d);
	delete(optionalFeaturesDescriptor2d);
	delete(featuresMatcher2d);
	delete(bundleAdjuster);
	delete(fundamentalMatrixComputer);
	delete(perspectiveNPointSolver);
	delete(reconstructor3dfrom2dmatches);

	DeleteIfNotNull(bundleHistory);
	DeleteIfNotNull(correspondencesRecorder);
	DeleteIfNotNull(cleanCorrespondenceMap);
	DeleteIfNotNull(triangulatedKeypointCloud);

	DeleteIfNotNull(estimatedCameraPoses);
	DeleteIfNotNull(presentKeypointVector);
	DeleteIfNotNull(keypointCloud);

	delete(EMPTY_FEATURE_VECTOR);
	}


void AdjustmentFromStereo::run() 
	{
	#ifdef TESTING
	logFile.open("/InFuse/myLog.txt", std::ios::app);
	#endif
	DEBUG_PRINT_TO_LOG("Adjustment from stereo start", "");
 
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
		correspondencesRecorder->DiscardLatestCorrespondences();
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

		DEBUG_PRINT_TO_LOG("pose", ToString(outPose));
		DEBUG_PRINT_TO_LOG("points", GetNumberOfPoints(*outputPointCloud));

		#ifdef TESTING
		logFile << "output ";
		logFile << GetXPosition(outPose) << " " << GetYPosition(outPose) << " " << GetZPosition(outPose) << " ";
		logFile << GetXOrientation(outPose) << " " << GetYOrientation(outPose) << " " << GetZOrientation(outPose) << " " << GetWOrientation(outPose) << " ";
		logFile << GetNumberOfPoints(outPointCloud) << " ";

		if (GetNumberOfPoints(outPointCloud) > 0)
			{
			double minMax[6] = {
			        GetXCoordinate(outPointCloud, 0),
			        GetXCoordinate(outPointCloud, 0),
			        GetYCoordinate(outPointCloud, 0),
                    GetYCoordinate(outPointCloud, 0),
                    GetZCoordinate(outPointCloud, 0),
                    GetZCoordinate(outPointCloud, 0)
			};
			int numberOfPoints = GetNumberOfPoints(outPointCloud);
			for(int pointIndex = 0; pointIndex < numberOfPoints; pointIndex++)
				{
				bool change[6];
				change[0] = minMax[0] < GetXCoordinate(outPointCloud, pointIndex);
				change[1] = minMax[1] > GetXCoordinate(outPointCloud, pointIndex);
				change[2] = minMax[2] < GetXCoordinate(outPointCloud, pointIndex);
				change[3] = minMax[3] > GetXCoordinate(outPointCloud, pointIndex);
				change[4] = minMax[4] < GetXCoordinate(outPointCloud, pointIndex);
				change[5] = minMax[5] > GetXCoordinate(outPointCloud, pointIndex);
				minMax[0] = change[0] ? GetXCoordinate(outPointCloud, pointIndex) : minMax[0];
				minMax[1] = change[1] ? GetXCoordinate(outPointCloud, pointIndex) : minMax[1];
				minMax[2] = change[2] ? GetXCoordinate(outPointCloud, pointIndex) : minMax[2];
				minMax[3] = change[3] ? GetXCoordinate(outPointCloud, pointIndex) : minMax[3];
				minMax[4] = change[4] ? GetXCoordinate(outPointCloud, pointIndex) : minMax[4];
				minMax[5] = change[5] ? GetXCoordinate(outPointCloud, pointIndex) : minMax[5];
				}
			logFile << (minMax[0] - minMax[1]) << " " << (minMax[2] - minMax[3]) << " " << (minMax[4] - minMax[5]) << " ";
			}
		else
			{
			logFile << "0 0 0 0 0 0 ";
			}
		#endif

		DEBUG_SHOW_POINT_CLOUD(outputPointCloud);
		DeleteIfNotNull(outputPointCloud);
		}

	currentInputNumber++;

	#ifdef TESTING
	logFile << std::endl;
	logFile.close();
	#endif
	}

void AdjustmentFromStereo::setup()
	{
	configurator.configure(configurationFilePath);
	ConfigureExtraParameters(); //Configuration shall happen before alias assignment here.

	InstantiateDFNExecutors();

	DeleteIfNotNull(bundleHistory);
	bundleHistory = new BundleHistory(parameters.numberOfAdjustedStereoPairs + 1);

	DeleteIfNotNull(correspondencesRecorder);
	correspondencesRecorder = new MultipleCorrespondences2DRecorder(parameters.numberOfAdjustedStereoPairs, true);	

	pointCloudMap.SetResolution(parameters.pointCloudMapResolution);

	SetPosition(rightToLeftCameraPose, -parameters.baseline, 0, 0);
	SetOrientation(rightToLeftCameraPose, 0, 0, 0, 1);

	if (parameters.useBundleInitialEstimation && estimatedCameraPoses == NULL)
		{
		estimatedCameraPoses = NewPoses3DSequence();
		presentKeypointVector = NewVisualPointFeatureVector2D();
		keypointCloud = NewPointCloud();
		}
	}

/* --------------------------------------------------------------------------
 *
 * Private Member Variables
 *
 * --------------------------------------------------------------------------
 */

const AdjustmentFromStereo::AdjustmentFromStereoOptionsSet AdjustmentFromStereo::DEFAULT_PARAMETERS = 
	{
	/*.searchRadius =*/ 20,
	/*.pointCloudMapResolution =*/ 1e-2,
	/*.numberOfAdjustedStereoPairs =*/ 4,
	/*.useBundleInitialEstimation =*/ true,
	/*.baseline =*/ 1
	};

/* --------------------------------------------------------------------------
 *
 * Private Member Functions
 *
 * --------------------------------------------------------------------------
 */
void AdjustmentFromStereo::ConfigureExtraParameters()
	{
	parametersHelper.ReadFile( configurator.GetExtraParametersConfigurationFilePath() );

	ASSERT(parameters.pointCloudMapResolution > 0, "AdjustmentFromStereo Error, Point Cloud Map resolution is not positive");
	}

void AdjustmentFromStereo::InstantiateDFNExecutors()
	{
	optionalLeftFilter = new ImageFilteringExecutor( static_cast<ImageFilteringInterface*>( configurator.GetDfn("leftFilter", true) ) );
	optionalRightFilter = new ImageFilteringExecutor( static_cast<ImageFilteringInterface*>( configurator.GetDfn("rightFilter", true) ) );
	reconstructor3d = new StereoReconstructionExecutor( static_cast<StereoReconstructionInterface*>( configurator.GetDfn("reconstructor3d") ) );
	featuresExtractor2d = new FeaturesExtraction2DExecutor( static_cast<FeaturesExtraction2DInterface*>( configurator.GetDfn("featuresExtractor2d") ) );
	optionalFeaturesDescriptor2d = new FeaturesDescription2DExecutor( static_cast<FeaturesDescription2DInterface*>( configurator.GetDfn("featuresDescriptor2d", true) ) );
	featuresMatcher2d = new FeaturesMatching2DExecutor( static_cast<FeaturesMatching2DInterface*>( configurator.GetDfn("featuresMatcher2d") ) );
	bundleAdjuster = new BundleAdjustmentExecutor( static_cast<BundleAdjustmentInterface*>( configurator.GetDfn("bundleAdjuster") ) );
	fundamentalMatrixComputer = new FundamentalMatrixComputationExecutor( static_cast<FundamentalMatrixComputationInterface*>( configurator.GetDfn("fundamentalMatrixComputer") ) );

	if (parameters.useBundleInitialEstimation)
		{		
		perspectiveNPointSolver = new PerspectiveNPointSolvingExecutor( static_cast<PerspectiveNPointSolvingInterface*>( configurator.GetDfn("perspectiveNPointSolver") ) );
		reconstructor3dfrom2dmatches = new PointCloudReconstruction2DTo3DExecutor( static_cast<PointCloudReconstruction2DTo3DInterface*>( configurator.GetDfn("reconstructor3dfrom2dmatches") ) );
		}
	}

/**
* The method filters the left and right images, and uses them for the computation of a point cloud.
*
**/
void AdjustmentFromStereo::ComputeStereoPointCloud(FrameWrapper::FrameConstPtr filteredLeftImage, FrameWrapper::FrameConstPtr filteredRightImage)
	{
	PointCloudConstPtr imageCloud = NULL;
	reconstructor3d->Execute(filteredLeftImage, filteredRightImage, imageCloud);

	bundleHistory->AddPointCloud(*imageCloud, STEREO_CLOUD_CATEGORY);
	#ifdef TESTING
	logFile << GetNumberOfPoints(*imageCloud) << " ";
	#endif

	DEBUG_PRINT_TO_LOG("Stereo points number", GetNumberOfPoints(*imageCloud));
	//DEBUG_SHOW_POINT_CLOUD(imageCloud);
	}

#define MINIMUM(a, b) ( a < b ? a : b )

void AdjustmentFromStereo::ComputeVisualPointFeatures(FrameWrapper::FrameConstPtr filteredLeftImage, FrameWrapper::FrameConstPtr filteredRightImage)
	{
	VisualPointFeatureVector2DConstPtr keypointVector = NULL;
	VisualPointFeatureVector2DConstPtr featureVector = NULL;
	featuresExtractor2d->Execute(filteredLeftImage, keypointVector);
	optionalFeaturesDescriptor2d->Execute(filteredLeftImage, keypointVector, featureVector);
	bundleHistory->AddFeatures(*featureVector, LEFT_FEATURE_CATEGORY);
	DEBUG_PRINT_TO_LOG("Features Number", GetNumberOfPoints(*featureVector) );

	#ifdef TESTING
	logFile << GetNumberOfPoints(*keypointVector) << " " << GetNumberOfPoints(*featureVector) << " ";
	#endif

	keypointVector = NULL;
	featureVector = NULL;
	featuresExtractor2d->Execute(filteredRightImage, keypointVector);
	optionalFeaturesDescriptor2d->Execute(filteredRightImage, keypointVector, featureVector);
	bundleHistory->AddFeatures(*featureVector, RIGHT_FEATURE_CATEGORY);
	DEBUG_PRINT_TO_LOG("Features Number", GetNumberOfPoints(*featureVector) );

	#ifdef TESTING
	logFile << GetNumberOfPoints(*keypointVector) << " " << GetNumberOfPoints(*featureVector) << " ";
	#endif

	VisualPointFeatureVector2DConstPtr leftFeatureVector = bundleHistory->GetFeatures(0, LEFT_FEATURE_CATEGORY);
	VisualPointFeatureVector2DConstPtr rightFeatureVector = bundleHistory->GetFeatures(0, RIGHT_FEATURE_CATEGORY);
	CorrespondenceMap2DConstPtr leftRightCorrespondenceMap = NULL;
	featuresMatcher2d->Execute(leftFeatureVector, rightFeatureVector,leftRightCorrespondenceMap);
	DEBUG_PRINT_TO_LOG("Correspondences Number", GetNumberOfCorrespondences(*leftRightCorrespondenceMap) );

	#ifdef TESTING
	logFile << GetNumberOfCorrespondences(*leftRightCorrespondenceMap) << " ";
	#endif

	MatrixWrapper::Matrix3dConstPtr fundamentalMatrix = NULL;
	bool success = false;
	CorrespondenceMap2DConstPtr inlierCorrespondenceMap = NULL;
	fundamentalMatrixComputer->Execute(leftRightCorrespondenceMap, fundamentalMatrix, success, inlierCorrespondenceMap);
	DEBUG_PRINT_TO_LOG("Inlier Correspondences Number", GetNumberOfCorrespondences(*inlierCorrespondenceMap) );

	#ifdef TESTING
	logFile << success << " " << GetNumberOfCorrespondences(*inlierCorrespondenceMap) << " ";
	#endif

	if (parameters.useBundleInitialEstimation)
		{
		if (success)
			{
			reconstructor3dfrom2dmatches->Execute(inlierCorrespondenceMap, &rightToLeftCameraPose, triangulatedKeypointCloud);
			CleanUnmatchedFeatures(inlierCorrespondenceMap, triangulatedKeypointCloud);
			}
		else
			{
			reconstructor3dfrom2dmatches->Execute(leftRightCorrespondenceMap, &rightToLeftCameraPose, triangulatedKeypointCloud);
			CleanUnmatchedFeatures(leftRightCorrespondenceMap, triangulatedKeypointCloud);
			}
		//DEBUG_SHOW_2D_CORRESPONDENCES(filteredLeftImage, filteredRightImage, leftRightCorrespondenceMap);
		DEBUG_PRINT_TO_LOG("Triangulated points Number", GetNumberOfPoints(*triangulatedKeypointCloud) );
		bundleHistory->AddPointCloud(*triangulatedKeypointCloud, TRIANGULATION_CLOUD_CATEGORY);
		bundleHistory->AddMatches(*cleanCorrespondenceMap);
		#ifdef TESTING
		logFile << GetNumberOfPoints(*triangulatedKeypointCloud) << " " << GetNumberOfCorrespondences(*cleanCorrespondenceMap) << " ";
		#endif
		}
	else
		{
		bundleHistory->AddMatches(*leftRightCorrespondenceMap);
		}
	}

void AdjustmentFromStereo::CleanUnmatchedFeatures(CorrespondenceMap2DWrapper::CorrespondenceMap2DConstPtr map, PointCloudWrapper::PointCloudPtr cloud)
	{
	ASSERT( GetNumberOfCorrespondences(*map) == GetNumberOfPoints(*cloud), "CleanUmatchedFeatures error: expected same number of points in map and cloud");
	Copy(*map, *cleanCorrespondenceMap);	

	std::vector<BaseTypesWrapper::T_UInt32> removeIndexList;	
	for(int pointIndex = 0; pointIndex < GetNumberOfPoints(*cloud); pointIndex++)
		{
		float x = GetXCoordinate(*cloud, pointIndex);
		float y = GetYCoordinate(*cloud, pointIndex);
		float z = GetZCoordinate(*cloud, pointIndex);
		if ( x != x || y != y || z != z)
			{
			removeIndexList.push_back(pointIndex);
			}
		}
	RemoveCorrespondences(*cleanCorrespondenceMap, removeIndexList);
	RemovePoints(*cloud, removeIndexList);
	}

void AdjustmentFromStereo::CreateWorkingCorrespondences()
	{
	VisualPointFeatureVector2DConstPtr leftFeatureVector = bundleHistory->GetFeatures(0, LEFT_FEATURE_CATEGORY);
	VisualPointFeatureVector2DConstPtr rightFeatureVector = bundleHistory->GetFeatures(0, RIGHT_FEATURE_CATEGORY);
	CorrespondenceMap2DConstPtr leftRightCorrespondenceMap = bundleHistory->GetMatches(0);

	correspondencesRecorder->InitializeNewSequence();
	correspondencesRecorder->AddCorrespondences(leftRightCorrespondenceMap);
	CreateWorkingCorrespondences(leftFeatureVector);
	CreateWorkingCorrespondences(rightFeatureVector);
	correspondencesRecorder->CompleteNewSequence();

	#ifdef TESTING
	logFile << "Correspondences ";
	CorrespondenceMaps2DSequenceConstPtr workingCorrespondenceMapSequence = correspondencesRecorder->GetLatestCorrespondences();
	logFile << GetNumberOfCorrespondenceMaps(*workingCorrespondenceMapSequence) << " ";
	for(int i=0; i<GetNumberOfCorrespondenceMaps(*workingCorrespondenceMapSequence); i++)
		{
		const CorrespondenceMap2D& map = GetCorrespondenceMap(*workingCorrespondenceMapSequence, i);
		logFile << GetNumberOfCorrespondences(map) << " ";
		}
	#endif

	/*#ifdef TESTING
	std::vector<CorrespondenceMap2DConstPtr> correspondenceMapList = {leftRightCorrespondenceMap, leftTimeCorrespondenceMap, rightTimeCorrespondenceMap, pastLeftRightCorrespondenceMap};
	std::vector<FrameConstPtr> imageList = { 
		bundleHistory->GetLeftImage(0), bundleHistory->GetRightImage(0), bundleHistory->GetLeftImage(backwardSteps), bundleHistory->GetRightImage(backwardSteps) 
		};
	DEBUG_SHOW_QUADRUPLE_2D_CORRESPONDENCES( imageList, correspondenceMapList );
	#endif*/
	}

void AdjustmentFromStereo::CreateWorkingCorrespondences(VisualPointFeatureVector2DWrapper::VisualPointFeatureVector2DConstPtr featureVector)
	{
	for(int backwardSteps = 1; backwardSteps < parameters.numberOfAdjustedStereoPairs; backwardSteps++)
		{
		VisualPointFeatureVector2DConstPtr pastLeftFeatureVector = bundleHistory->GetFeatures(backwardSteps, LEFT_FEATURE_CATEGORY);
		VisualPointFeatureVector2DConstPtr pastRightFeatureVector = bundleHistory->GetFeatures(backwardSteps, RIGHT_FEATURE_CATEGORY);

		if (pastLeftFeatureVector == NULL || pastRightFeatureVector == NULL)
			{
			DEBUG_PRINT_TO_LOG("Breaking at", backwardSteps);
			break;
			}

		CorrespondenceMap2DConstPtr leftTimeCorrespondenceMap = NULL;
		featuresMatcher2d->Execute(featureVector, pastLeftFeatureVector, leftTimeCorrespondenceMap);
		correspondencesRecorder->AddCorrespondences(leftTimeCorrespondenceMap);

		CorrespondenceMap2DConstPtr rightTimeCorrespondenceMap = NULL;
		featuresMatcher2d->Execute(featureVector, pastRightFeatureVector, rightTimeCorrespondenceMap);
		correspondencesRecorder->AddCorrespondences(rightTimeCorrespondenceMap);
		} 
	}

bool AdjustmentFromStereo::ComputeCameraPoses(PoseWrapper::Poses3DSequenceConstPtr& cameraPoses)
	{
	DEBUG_PRINT_TO_LOG("About to execute bundle adjustment", "");
	DEBUG_PRINT_TO_LOG("currentInputNumber", currentInputNumber );
	cameraPoses = NULL;
	bool success;
	float error;

	CorrespondenceMaps2DSequenceConstPtr workingCorrespondenceMapSequence = correspondencesRecorder->GetLatestCorrespondences();
	if (parameters.useBundleInitialEstimation)
		{
		PointCloudConstPtr triangulatedKeypointCloud = bundleHistory->GetPointCloud(0, TRIANGULATION_CLOUD_CATEGORY);
		PRINT_TO_LOG("Triangulated points Number", GetNumberOfPoints(*triangulatedKeypointCloud) );
		
		EstimateCameraPoses(); 

		bundleAdjuster->Execute(workingCorrespondenceMapSequence, estimatedCameraPoses, triangulatedKeypointCloud, cameraPoses, success, error);
		}
	else
		{
		bundleAdjuster->Execute(workingCorrespondenceMapSequence, cameraPoses, success, error);
		}
	
	if (success)
		{
		DEBUG_PRINT_TO_LOG("Bundle adjustement success", "");
		for(int poseIndex = 0; poseIndex< GetNumberOfPoses(*cameraPoses); poseIndex++)
			{
			DEBUG_PRINT_TO_LOG("Pose", ToString( GetPose(*cameraPoses, poseIndex) ) );
			}
		}
	else
		{
		DEBUG_PRINT_TO_LOG("Bundle adjustement failure", "");
		}

	DEBUG_PRINT_TO_LOG("Error:", error);

	#ifdef TESTING
	logFile << "error " << error << " " << success << " ";
	#endif

	return success;
	}

void AdjustmentFromStereo::AddAllPointCloudsToMap(Poses3DSequenceConstPtr& cameraPoses)
	{
	for(int stereoIndex = 0; stereoIndex < parameters.numberOfAdjustedStereoPairs; stereoIndex++)
		{
		const Pose3D& pose = GetPose(*cameraPoses, 2*stereoIndex);
		pointCloudMap.AddPointCloud( bundleHistory->GetPointCloud(stereoIndex, STEREO_CLOUD_CATEGORY), EMPTY_FEATURE_VECTOR, &pose);
		}
	}

void AdjustmentFromStereo::AddLastPointCloudToMap(Poses3DSequenceConstPtr& cameraPoses)
	{
	const Pose3D& poseOfPastCameraInCurrentCamera = GetPose(*cameraPoses, 2);

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

void AdjustmentFromStereo::EstimateCameraPoses()
	{
	Pose3D zeroPose;
	SetPosition(zeroPose, 0, 0, 0);
	SetOrientation(zeroPose, 0, 0, 0, 1);
	Clear(*estimatedCameraPoses);
	#ifdef TESTING
	logFile << "Estimated Poses ";
	#endif
	
	VisualPointFeatureVector2DConstPtr leftFeatureVector = bundleHistory->GetFeatures(0, LEFT_FEATURE_CATEGORY);
	for(int backwardSteps = 1; backwardSteps < parameters.numberOfAdjustedStereoPairs; backwardSteps++)
		{
		VisualPointFeatureVector2DConstPtr pastLeftFeatureVector = bundleHistory->GetFeatures(backwardSteps, LEFT_FEATURE_CATEGORY);
		CorrespondenceMap2DConstPtr pastCorrespondenceMap = bundleHistory->GetMatches(backwardSteps);
		PointCloudConstPtr pastTriangulatedCloud = bundleHistory->GetPointCloud(backwardSteps, TRIANGULATION_CLOUD_CATEGORY);
		if (pastLeftFeatureVector == NULL)
			{
			break;
			}
		CorrespondenceMap2DConstPtr leftCorrespondenceMap = NULL;
		featuresMatcher2d->Execute(leftFeatureVector, pastLeftFeatureVector, leftCorrespondenceMap);

		Matrix3dConstPtr fundamentalMatrix = NULL;
		Pose3DConstPtr pose = NULL;
		bool success;
		CorrespondenceMap2DConstPtr inlierCorrespondenceMap = NULL;
		fundamentalMatrixComputer->Execute(leftCorrespondenceMap, fundamentalMatrix, success, inlierCorrespondenceMap);
		DEBUG_PRINT_TO_LOG("Number of inlier correspondences", GetNumberOfCorrespondences(*inlierCorrespondenceMap) );
		#ifdef TESTING
		logFile << GetNumberOfCorrespondences(*inlierCorrespondenceMap) << " ";
		#endif
		if (success)
			{
			//cameraTransformEstimator->Execute(fundamentalMatrix, inlierCorrespondenceMap, pose, success);
			EstimatePose(pastCorrespondenceMap, inlierCorrespondenceMap, pastTriangulatedCloud, pose, success);
			}
		if (success)
			{
			DEBUG_PRINT_TO_LOG("estimated pose", ToString(*pose));
			AddPose(*estimatedCameraPoses, *pose);
			#ifdef TESTING
			logFile << GetXPosition(*pose) << " " << GetYPosition(*pose) << " " << GetZPosition(*pose) << " ";
			logFile << GetXOrientation(*pose) << " " << GetYOrientation(*pose) << " " << GetZOrientation(*pose) << " " << GetWOrientation(*pose) << " ";
			#endif
			}
		else
			{
			DEBUG_PRINT_TO_LOG("zero pose", ToString(zeroPose));
			AddPose(*estimatedCameraPoses, zeroPose);
			#ifdef TESTING
			logFile << "0 0 0 0 0 0 1 ";
			#endif
			}
		}
	}

void AdjustmentFromStereo::EstimatePose(CorrespondenceMap2DConstPtr pastLeftRightCorrespondenceMap, CorrespondenceMap2DConstPtr leftPresentPastCorrespondenceMap, 
	PointCloudConstPtr pastCloud, Pose3DConstPtr& pose, bool& success)
	{
	int leftPresentPastCorrespondenceNumber = GetNumberOfCorrespondences(*leftPresentPastCorrespondenceMap);
	int pastLeftRightCorrespondenceNumber = GetNumberOfCorrespondences(*pastLeftRightCorrespondenceMap);
	ASSERT( GetNumberOfPoints(*pastCloud) == pastLeftRightCorrespondenceNumber, "Error: Cloud and correspondences do not have the same cardinality");

	ClearPoints(*presentKeypointVector);
	ClearPoints(*keypointCloud);
	for(int leftPresentPastIndex=0; leftPresentPastIndex<leftPresentPastCorrespondenceNumber; leftPresentPastIndex++)
		{
		BaseTypesWrapper::Point2D leftPoint = GetSource(*leftPresentPastCorrespondenceMap, leftPresentPastIndex);
		BaseTypesWrapper::Point2D pastLeftPoint = GetSink(*leftPresentPastCorrespondenceMap, leftPresentPastIndex);
		for(int pastLeftRightIndex=0; pastLeftRightIndex<pastLeftRightCorrespondenceNumber; pastLeftRightIndex++)
			{
			BaseTypesWrapper::Point2D correspondingPastLeftPoint = GetSource(*pastLeftRightCorrespondenceMap, pastLeftRightIndex);
			if ( correspondingPastLeftPoint.x == pastLeftPoint.x && correspondingPastLeftPoint.y == pastLeftPoint.y )
				{
				AddPoint(*presentKeypointVector, leftPoint.x, leftPoint.y);
				AddPoint(*keypointCloud, GetXCoordinate(*pastCloud, pastLeftRightIndex), GetYCoordinate(*pastCloud, pastLeftRightIndex), GetZCoordinate(*pastCloud, pastLeftRightIndex) );
				break;
				}
			}
		}

	pose = NULL;
	perspectiveNPointSolver->Execute(keypointCloud, presentKeypointVector, pose, success);
	}

}
}
}



/** @} */
