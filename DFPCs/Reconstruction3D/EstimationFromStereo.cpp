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
#include <Visualizers/OpenCVVisualizer.hpp>
#include <Visualizers/PCLVisualizer.hpp>
#include <Types/CPP/VisualPointFeatureVector3D.hpp>

#include <Executors/ImageFiltering/ImageFilteringExecutor.hpp>
#include <Executors/StereoReconstruction/StereoReconstructionExecutor.hpp>
#include <Executors/FeaturesExtraction2D/FeaturesExtraction2DExecutor.hpp>
#include <Executors/FeaturesDescription2D/FeaturesDescription2DExecutor.hpp>
#include <Executors/FeaturesMatching2D/FeaturesMatching2DExecutor.hpp>
#include <Executors/FundamentalMatrixComputation/FundamentalMatrixComputationExecutor.hpp>
#include <Executors/CamerasTransformEstimation/CamerasTransformEstimationExecutor.hpp>
#include <Executors/PointCloudReconstruction2DTo3D/PointCloudReconstruction2DTo3DExecutor.hpp>
#include <Executors/Transform3DEstimation/Transform3DEstimationExecutor.hpp>

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
	parameters = DEFAULT_PARAMETERS;

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
	fundamentalMatrixComputer = NULL;

	bundleHistory = NULL;
	correspondencesRecorder = NULL;

	leftTimeCorrespondenceMap = NewCorrespondenceMap2D();
	rightTimeCorrespondenceMap = NewCorrespondenceMap2D();

	configurationFilePath = "";
	#ifdef TESTING
	logFile.open("/InFuse/myLog.txt");
	logFile << "denseCloud leftKeypoints leftFeatures rightKeypoints rightFeatures correspondences FMFiltersuccess inlierCorrespondences sparseCloud ";
	logFile << "Correspondences error success validPose X Y Z QX QY QZ QW output X Y Z QX QY QZ QW cloud SpaceX SpaceY SpaceZ" << std::endl;
	logFile.close();
	#endif
	}

EstimationFromStereo::~EstimationFromStereo()
	{
	DeleteIfNotNull(bundleHistory);
	DeleteIfNotNull(correspondencesRecorder);
	DeleteIfNotNull(leftTimeCorrespondenceMap);
	DeleteIfNotNull(rightTimeCorrespondenceMap);

	delete(EMPTY_FEATURE_VECTOR);
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
	Executors::Execute(optionalLeftFilter, inLeftImage, filteredLeftImage);
	Executors::Execute(optionalRightFilter, inRightImage, filteredRightImage);

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

void EstimationFromStereo::setup()
	{
	configurator.configure(configurationFilePath);
	ConfigureExtraParameters(); //Configuration shall happen before alias assignment here.

	InstantiateDFNs();

	DeleteIfNotNull(bundleHistory);
	bundleHistory = new BundleHistory(parameters.numberOfAdjustedStereoPairs + 1);

	DeleteIfNotNull(correspondencesRecorder);
	correspondencesRecorder = new MultipleCorrespondencesRecorder(parameters.numberOfAdjustedStereoPairs);	

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
	/*.searchRadius =*/ 20,
	/*.pointCloudMapResolution =*/ 1e-2,
	/*.numberOfAdjustedStereoPairs =*/ 4,
	/*.baseline =*/ 1
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

void EstimationFromStereo::InstantiateDFNs()
	{
	optionalLeftFilter = static_cast<ImageFilteringInterface*>( configurator.GetDfn("leftFilter", true) );
	optionalRightFilter = static_cast<ImageFilteringInterface*>( configurator.GetDfn("rightFilter", true) );
	reconstructor3d = static_cast<StereoReconstructionInterface*>( configurator.GetDfn("reconstructor3d") );
	featuresExtractor2d = static_cast<FeaturesExtraction2DInterface*>( configurator.GetDfn("featuresExtractor2d") );
	optionalFeaturesDescriptor2d = static_cast<FeaturesDescription2DInterface*>( configurator.GetDfn("featuresDescriptor2d", true) );
	featuresMatcher2d = static_cast<FeaturesMatching2DInterface*>( configurator.GetDfn("featuresMatcher2d") );
	reconstructor3dfrom2dmatches = static_cast<PointCloudReconstruction2DTo3DInterface*>( configurator.GetDfn("reconstructor3dfrom2dmatches", true) );
	transformEstimator = static_cast<Transform3DEstimationInterface*>( configurator.GetDfn("transformEstimator") );
	fundamentalMatrixComputer = static_cast<FundamentalMatrixComputationInterface*>( configurator.GetDfn("fundamentalMatrixComputer") );
	}

void EstimationFromStereo::ComputeStereoPointCloud(FrameConstPtr filteredLeftImage, FrameConstPtr filteredRightImage)
	{
	PointCloudConstPtr imageCloud = NULL;
	Executors::Execute(reconstructor3d, filteredLeftImage, filteredRightImage, imageCloud);

	bundleHistory->AddPointCloud(*imageCloud, STEREO_CLOUD_CATEGORY);
	#ifdef TESTING
	logFile << GetNumberOfPoints(*imageCloud) << " ";
	#endif

	DEBUG_PRINT_TO_LOG("Stereo points number", GetNumberOfPoints(*imageCloud));
	//DEBUG_SHOW_POINT_CLOUD(imageCloud);
	}

#define MINIMUM(a, b) ( a < b ? a : b )

void EstimationFromStereo::ComputeVisualPointFeatures(FrameConstPtr filteredLeftImage, FrameConstPtr filteredRightImage)
	{
	VisualPointFeatureVector2DConstPtr keypointVector = NULL;
	VisualPointFeatureVector2DConstPtr featureVector = NULL;
	Executors::Execute(featuresExtractor2d, filteredLeftImage, keypointVector);
	Executors::Execute(optionalFeaturesDescriptor2d, filteredLeftImage, keypointVector, featureVector);
	bundleHistory->AddFeatures(*featureVector, LEFT_FEATURE_CATEGORY);
	DEBUG_PRINT_TO_LOG("Features Number", GetNumberOfPoints(*featureVector) );

	#ifdef TESTING
	logFile << GetNumberOfPoints(*keypointVector) << " " << GetNumberOfPoints(*featureVector) << " ";
	#endif

	keypointVector = NULL;
	featureVector = NULL;
	Executors::Execute(featuresExtractor2d, filteredRightImage, keypointVector);
	Executors::Execute(optionalFeaturesDescriptor2d, filteredRightImage, keypointVector, featureVector);
	bundleHistory->AddFeatures(*featureVector, RIGHT_FEATURE_CATEGORY);
	DEBUG_PRINT_TO_LOG("Features Number", GetNumberOfPoints(*featureVector) );

	#ifdef TESTING
	logFile << GetNumberOfPoints(*keypointVector) << " " << GetNumberOfPoints(*featureVector) << " ";
	#endif

	VisualPointFeatureVector2DConstPtr leftFeatureVector = bundleHistory->GetFeatures(0, LEFT_FEATURE_CATEGORY);
	VisualPointFeatureVector2DConstPtr rightFeatureVector = bundleHistory->GetFeatures(0, RIGHT_FEATURE_CATEGORY);
	CorrespondenceMap2DConstPtr leftRightCorrespondenceMap = NULL;
	Executors::Execute(featuresMatcher2d, leftFeatureVector, rightFeatureVector,leftRightCorrespondenceMap);
	DEBUG_PRINT_TO_LOG("Correspondences Number", GetNumberOfCorrespondences(*leftRightCorrespondenceMap) );

	#ifdef TESTING
	logFile << GetNumberOfCorrespondences(*leftRightCorrespondenceMap) << " ";
	#endif

	MatrixWrapper::Matrix3dConstPtr fundamentalMatrix = NULL;
	bool success = false;
	CorrespondenceMap2DConstPtr inlierCorrespondenceMap = NULL;
	Executors::Execute(fundamentalMatrixComputer, leftRightCorrespondenceMap, fundamentalMatrix, success, inlierCorrespondenceMap);

	#ifdef TESTING
	logFile << success << " " << GetNumberOfCorrespondences(*inlierCorrespondenceMap) << " ";
	#endif

	PointCloudConstPtr triangulatedKeypointCloud = NULL;
	if (success)
		{
		bundleHistory->AddMatches(*inlierCorrespondenceMap);
		DEBUG_PRINT_TO_LOG("Clean Inlier Correspondences Number", GetNumberOfCorrespondences(*inlierCorrespondenceMap) );
		Executors::Execute(reconstructor3dfrom2dmatches, inlierCorrespondenceMap, &rightToLeftCameraPose, triangulatedKeypointCloud);
		}
	else
		{
		bundleHistory->AddMatches(*leftRightCorrespondenceMap);
		DEBUG_PRINT_TO_LOG("Clean Correspondences Number", GetNumberOfCorrespondences(*leftRightCorrespondenceMap) );
		Executors::Execute(reconstructor3dfrom2dmatches, leftRightCorrespondenceMap, &rightToLeftCameraPose, triangulatedKeypointCloud);
		}
	//DEBUG_SHOW_2D_CORRESPONDENCES(filteredLeftImage, filteredRightImage, leftRightCorrespondenceMap);
	bundleHistory->AddPointCloud(*triangulatedKeypointCloud, TRIANGULATION_CLOUD_CATEGORY);
	DEBUG_PRINT_TO_LOG("Triangulated points Number", GetNumberOfPoints(*triangulatedKeypointCloud) );
	#ifdef TESTING
	int validPointCounter = 0;
	int numberOfPoints = GetNumberOfPoints(*triangulatedKeypointCloud);
	for(int pointIndex = 0; pointIndex < numberOfPoints; pointIndex++)
		{
		if ( GetXCoordinate(*triangulatedKeypointCloud, pointIndex) == GetXCoordinate(*triangulatedKeypointCloud, pointIndex) )
			{
			validPointCounter++;
			}
		}
	logFile << validPointCounter << " ";
	#endif
	}

bool EstimationFromStereo::ComputeCameraPoses(Poses3DSequenceConstPtr& cameraPoses)
	{
	cameraPoses = NULL;
	bool success;
	float error;
	CorrespondenceMaps3DSequencePtr workingCorrespondenceMapSequence = correspondencesRecorder->GetLatestCorrespondences();

	Executors::Execute(transformEstimator, workingCorrespondenceMapSequence, cameraPoses, success, error);
	
	#ifdef TESTING
	logFile << "error " << error << " " << success << " ";
	#endif	
	if (!success)
		{
		#ifdef TESTING
		logFile << "0 0 0 0 0 0 0 0 ";
		#endif	
		DEBUG_PRINT_TO_LOG("Transform estimation failed:", error);
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
	logFile << validPose << " " << GetXPosition(firstPose) << " " << GetYPosition(firstPose) << " " << GetZPosition(firstPose) << " ";
	logFile << GetXOrientation(firstPose) << " " << GetYOrientation(firstPose) << " " << GetZOrientation(firstPose) << " " << GetWOrientation(firstPose) << " ";
	#endif
	return validPose;
	}

void EstimationFromStereo::AddAllPointCloudsToMap(Poses3DSequenceConstPtr& cameraPoses)
	{
	for(int stereoIndex = parameters.numberOfAdjustedStereoPairs - 1; stereoIndex >= 0; stereoIndex--)
		{
		if (stereoIndex == 0)
			{
			Pose3D zeroPose;
			SetPosition(zeroPose, 0, 0, 0);
			SetOrientation(zeroPose, 0, 0, 0, 1);
			pointCloudMap.AddPointCloud( bundleHistory->GetPointCloud(stereoIndex, STEREO_CLOUD_CATEGORY), EMPTY_FEATURE_VECTOR, &zeroPose);
			}
		else
			{
			const Pose3D& pose = GetPose(*cameraPoses, stereoIndex);
			pointCloudMap.AddPointCloud(bundleHistory->GetPointCloud(stereoIndex, STEREO_CLOUD_CATEGORY), EMPTY_FEATURE_VECTOR, &pose);
			}
		}
	}

void EstimationFromStereo::AddLastPointCloudToMap(Poses3DSequenceConstPtr& cameraPoses)
	{
	const Pose3D& poseOfCurrentCameraInPastCamera = GetPose(*cameraPoses, 0);

	pointCloudMap.AttachPointCloud(bundleHistory->GetPointCloud(0, STEREO_CLOUD_CATEGORY), EMPTY_FEATURE_VECTOR, &poseOfCurrentCameraInPastCamera);
	}

void EstimationFromStereo::CreateWorkingCorrespondences()
	{
	VisualPointFeatureVector2DConstPtr leftFeatureVector = bundleHistory->GetFeatures(0, LEFT_FEATURE_CATEGORY);
	VisualPointFeatureVector2DConstPtr rightFeatureVector = bundleHistory->GetFeatures(0, RIGHT_FEATURE_CATEGORY);
	CorrespondenceMap2DConstPtr leftRightCorrespondenceMap = bundleHistory->GetMatches(0);
	PointCloudConstPtr triangulatedKeypointCloud = bundleHistory->GetPointCloud(0, TRIANGULATION_CLOUD_CATEGORY);	

	correspondencesRecorder->InitializeNewSequence();
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

		Executors::Execute(featuresMatcher2d, leftFeatureVector, pastLeftFeatureVector, leftTimeCorrespondenceMap);
		Executors::Execute(featuresMatcher2d, rightFeatureVector, pastRightFeatureVector, rightTimeCorrespondenceMap);
		std::vector<CorrespondenceMap2DConstPtr> correspondenceMapList = {leftRightCorrespondenceMap, leftTimeCorrespondenceMap, rightTimeCorrespondenceMap, pastLeftRightCorrespondenceMap};
		std::vector<PointCloudConstPtr> pointCloudList = {triangulatedKeypointCloud, pastTriangulatedKeypointCloud};
		#ifdef TESTING
		std::vector<FrameConstPtr> imageList = { 
			bundleHistory->GetLeftImage(0), bundleHistory->GetRightImage(0), bundleHistory->GetLeftImage(backwardSteps), bundleHistory->GetRightImage(backwardSteps) 
			};
		DEBUG_SHOW_QUADRUPLE_2D_CORRESPONDENCES( imageList, correspondenceMapList );
		#endif
		
		correspondencesRecorder->AddCorrespondencesFromTwoImagePairs(correspondenceMapList, pointCloudList);
		} 
	correspondencesRecorder->CompleteNewSequence();

	#ifdef TESTING
	CorrespondenceMaps3DSequencePtr workingCorrespondenceMapSequence = correspondencesRecorder->GetLatestCorrespondences();
	logFile << "correspondences " << " ";
	logFile << GetNumberOfCorrespondenceMaps(*workingCorrespondenceMapSequence) << " ";
	for(int i=0; i<GetNumberOfCorrespondenceMaps(*workingCorrespondenceMapSequence); i++)
		{
		const CorrespondenceMap3D& map = GetCorrespondenceMap(*workingCorrespondenceMapSequence, i);
		logFile << GetNumberOfCorrespondences(map) << " ";
		}
	#endif
	}

}
}
}



/** @} */
