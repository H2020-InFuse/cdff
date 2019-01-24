/* --------------------------------------------------------------------------
*
* (C) Copyright …
*
* ---------------------------------------------------------------------------
*/

/*!
 * @file VisualOdometry.cpp
 * @date 27/04/2018
 * @author Alessandro Bianco
 */

/*!
 * @addtogroup DFNsTest
 * 
 * Implementation of the class VisualOdometryTestInterface
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
#include "VisualOdometry.hpp"

#include <Executors/FeaturesExtraction2D/FeaturesExtraction2DExecutor.hpp>
#include <Executors/FeaturesDescription2D/FeaturesDescription2DExecutor.hpp>
#include <Executors/FeaturesMatching2D/FeaturesMatching2DExecutor.hpp>
#include <Executors/FundamentalMatrixComputation/FundamentalMatrixComputationExecutor.hpp>
#include <Executors/CamerasTransformEstimation/CamerasTransformEstimationExecutor.hpp>

using namespace CDFF::DFN;
using namespace Converters;
using namespace FrameWrapper;
using namespace VisualPointFeatureVector2DWrapper;
using namespace CorrespondenceMap2DWrapper;
using namespace PoseWrapper;
using namespace MatrixWrapper;
using namespace CDFF::DFN::Executors;

/* --------------------------------------------------------------------------
 *
 * Public Member Functions
 *
 * --------------------------------------------------------------------------
 */
VisualOdometry::VisualOdometry(const std::string& folderPath, const std::vector<std::string>& baseConfigurationFileNamesList, const std::string& performanceMeasuresFileName) :
	PerformanceTestInterface(folderPath, baseConfigurationFileNamesList, performanceMeasuresFileName),
	groundPositionDistanceAggregator( Aggregator::AVERAGE ),
	groundOrientationDistanceAggregator( Aggregator::AVERAGE ),
	leftKeypointsAggregator(Aggregator::AVERAGE ),
	rightKeypointsAggregator(Aggregator::AVERAGE ),
	correspondencesAggregator(Aggregator::AVERAGE )
	{
	AddAggregator("PositionDistance", &groundPositionDistanceAggregator, FIXED_PARAMETERS_VARIABLE_INPUTS);
	AddAggregator("AngleDistace", &groundOrientationDistanceAggregator, FIXED_PARAMETERS_VARIABLE_INPUTS);

	AddAggregator("NumberOfLeftKeypoints", &leftKeypointsAggregator, FIXED_PARAMETERS_VARIABLE_INPUTS);
	AddAggregator("NumberOfRightKeypoints", &rightKeypointsAggregator, FIXED_PARAMETERS_VARIABLE_INPUTS);
	AddAggregator("NumberOfCorrespondences", &correspondencesAggregator, FIXED_PARAMETERS_VARIABLE_INPUTS);

	leftFrame = NULL;
	rightFrame = NULL;
	leftFeaturesVectorHolder = NewVisualPointFeatureVector2D();
	rightFeaturesVectorHolder = NewVisualPointFeatureVector2D();
	correspondenceMapHolder = NewCorrespondenceMap2D();
	decompositionSuccess = false;
	ransacSuccess = false;
	inputId = -1;

	initialX = 0;
	initialY = 0;
	initialZ = 0;
	initialRoll = 0;
	initialPitch = 0;
	initialYaw = 0;
	imageLimit = 0;
	}

VisualOdometry::~VisualOdometry()
	{
	if (leftFrame != NULL)
		{
		delete(leftFrame);
		}
	if (rightFrame != NULL)
		{
		delete(rightFrame);
		}
	delete(leftFeaturesVectorHolder);
	delete(rightFeaturesVectorHolder);
	delete(correspondenceMapHolder);
	}

void VisualOdometry::SetDfns(FeaturesExtraction2DInterface* extractor, FeaturesDescription2DInterface* descriptor, FeaturesMatching2DInterface* matcher, 
		FundamentalMatrixComputationInterface* matrixComputer, CamerasTransformEstimationInterface* poseEstimator)
	{
	this->extractor = extractor;
	this->descriptor = descriptor;
	this->matcher = matcher;
	this->matrixComputer = matrixComputer;
	this->poseEstimator = poseEstimator;

	AddDfn(extractor);
	if (descriptor != NULL)
		{
		AddDfn(descriptor);
		}
	AddDfn(matcher);
	AddDfn(matrixComputer);
	AddDfn(poseEstimator);
	}

void VisualOdometry::SetInputFiles(const std::string& imageFileNamesFolder, const std::string& imagesFileContainer, const std::string& posesFileContainer)
	{
	this->imageFileNamesFolder = imageFileNamesFolder;
	this->imagesFileContainer = imagesFileContainer;
	this->posesFileContainer = posesFileContainer;
	}

void VisualOdometry::LoadInputFiles()
	{
	LoadImageFileNames();
	LoadPoses();
	ComputeImagePoses();
	ASSERT(imageFileNamesList.size() == imagePosesList.size(), "Number of displacement different from number of images");
	}

void VisualOdometry::SetImageLimit(unsigned imageLimit)
	{
	if (imageLimit < imageFileNamesList.size())
		{
		this->imageLimit = imageLimit;
		}
	else
		{
		this->imageLimit = imageFileNamesList.size();
		}
	}


/* --------------------------------------------------------------------------
 *
 * Private Member Functions
 *
 * --------------------------------------------------------------------------
 */
void VisualOdometry::LoadImageFileNames()
	{
	std::ifstream containerFile(imagesFileContainer.c_str());
	ASSERT(containerFile.good(), "Error image container file not found");

	std::string line;

	std::getline(containerFile, line);
	std::getline(containerFile, line);
	std::getline(containerFile, line);

	while (std::getline(containerFile, line))
		{
		std::vector<std::string> stringsList;
		boost::split(stringsList, line, boost::is_any_of(" "));
		imageFileNamesList.push_back( stringsList.at(1) );

		imageTimesList.push_back( std::stod( stringsList.at(0) ) );
		}	

	containerFile.close();
	}

void VisualOdometry::LoadPoses()
	{
	std::ifstream containerFile(posesFileContainer.c_str());
	ASSERT(containerFile.good(), "Error poses container file not found");
	
	std::string line;

	std::getline(containerFile, line);
	std::getline(containerFile, line);
	std::getline(containerFile, line);

	while (std::getline(containerFile, line))
		{
		std::vector<std::string> stringsList;
		boost::split(stringsList, line, boost::is_any_of(" "));
		ASSERT(stringsList.size() == 8, "Error pose does not contain 8 elements");
	
		Pose3D pose;
		SetPosition(pose, std::stof( stringsList.at(1) ), std::stof( stringsList.at(2) ), std::stof( stringsList.at(3) ) );
		SetOrientation(pose, std::stof( stringsList.at(4) ), std::stof( stringsList.at(5) ), std::stof( stringsList.at(6) ), std::stof( stringsList.at(7) ) );
		posesList.push_back(pose);

		poseTimesList.push_back( std::stod( stringsList.at(0) ) );
		}

	containerFile.close();
	}

void VisualOdometry::ComputeImagePoses()
	{
	for(unsigned imageIndex = 0; imageIndex < imageFileNamesList.size(); imageIndex++)
		{
		double imageTime = imageTimesList.at(imageIndex);
		unsigned afterPoseIndex;
		bool afterPoseFound = false;
		for(unsigned index = 0; index < posesList.size() && !afterPoseFound; index++)
			{
			afterPoseFound = (poseTimesList.at(index) > imageTime);
			if (afterPoseFound)
				{
				afterPoseIndex = index;
				}
			}				
		ASSERT(afterPoseFound, "Error: No after pose found");
		ASSERT(afterPoseIndex > 0, "Error: No before pose found");
		unsigned beforePoseIndex = afterPoseIndex-1;

		Pose3D estimatedPose = InterpolatePose(beforePoseIndex, afterPoseIndex, imageTime);
		imagePosesList.push_back(estimatedPose);

		if (imageIndex == 0)
			{
			initialX = GetXPosition(estimatedPose);
			initialY = GetYPosition(estimatedPose);
			initialZ = GetZPosition(estimatedPose);
			Eigen::Quaternionf poseQuaternion(GetWOrientation(estimatedPose), GetXOrientation(estimatedPose), GetYOrientation(estimatedPose), GetZOrientation(estimatedPose));
			initialRoll = poseQuaternion.toRotationMatrix().eulerAngles(0, 1, 2)[0];
			initialPitch = poseQuaternion.toRotationMatrix().eulerAngles(0, 1, 2)[1];
			initialYaw = poseQuaternion.toRotationMatrix().eulerAngles(0, 1, 2)[2];
			}
		}
	}

Pose3D VisualOdometry::InterpolatePose(unsigned beforePoseIndex, unsigned afterPoseIndex, double imageTime)
	{
	Pose3D interpolation;

	const Pose3D& beforePose = posesList.at(beforePoseIndex);
	const Pose3D& afterPose = posesList.at(afterPoseIndex);
	const double beforeTime = poseTimesList.at(beforePoseIndex);
	const double afterTime = poseTimesList.at(afterPoseIndex);

	double deltaTime = afterTime - beforeTime;
	float deltaX = GetXPosition(afterPose) - GetXPosition(beforePose);
	float deltaY = GetYPosition(afterPose) - GetYPosition(beforePose);
	float deltaZ = GetZPosition(afterPose) - GetZPosition(beforePose);

	float runTime = imageTime - beforeTime;
	float linearInterpolationX = GetXPosition(beforePose) + (deltaX / deltaTime) * runTime;
	float linearInterpolationY = GetYPosition(beforePose) + (deltaY / deltaTime) * runTime;
	float linearInterpolationZ = GetZPosition(beforePose) + (deltaZ / deltaTime) * runTime;

	SetPosition(interpolation, linearInterpolationX, linearInterpolationY, linearInterpolationZ);

	Eigen::Quaternionf beforeQuaternion(GetWOrientation(beforePose), GetXOrientation(beforePose), GetYOrientation(beforePose), GetZOrientation(beforePose));
	float beforeRoll = beforeQuaternion.toRotationMatrix().eulerAngles(0, 1, 2)[0];
	float beforePitch = beforeQuaternion.toRotationMatrix().eulerAngles(0, 1, 2)[1];
	float beforeYaw = beforeQuaternion.toRotationMatrix().eulerAngles(0, 1, 2)[2];

	Eigen::Quaternionf afterQuaternion(GetWOrientation(afterPose), GetXOrientation(afterPose), GetYOrientation(afterPose), GetZOrientation(afterPose));
	float afterRoll = afterQuaternion.toRotationMatrix().eulerAngles(0, 1, 2)[0];
	float afterPitch = afterQuaternion.toRotationMatrix().eulerAngles(0, 1, 2)[1];
	float afterYaw = afterQuaternion.toRotationMatrix().eulerAngles(0, 1, 2)[2];

	float linearInterpolationRoll = beforeRoll + ( (afterRoll-beforeRoll) / deltaTime) * runTime;
	float linearInterpolationPitch = beforePitch + ( (afterPitch-beforePitch) / deltaTime) * runTime;
	float linearInterpolationYaw = beforeYaw + ( (afterYaw-beforeYaw) / deltaTime) * runTime;

	Eigen::Quaternionf linearInterpolationQ = 
		Eigen::AngleAxisf(linearInterpolationRoll, Eigen::Vector3f::UnitX()) * 
		Eigen::AngleAxisf(linearInterpolationPitch, Eigen::Vector3f::UnitY()) * 
		Eigen::AngleAxisf(linearInterpolationYaw, Eigen::Vector3f::UnitZ());

	SetOrientation(interpolation, linearInterpolationQ.x(), linearInterpolationQ.y(), linearInterpolationQ.z(), linearInterpolationQ.w());

	return interpolation;
	}

bool VisualOdometry::SetNextInputs()
	{
	inputId++;
	if (inputId+1 >= imageLimit)
		{
		return false;
		}

	std::stringstream nextFileStream, successiveFileStream;
	nextFileStream << imageFileNamesFolder << "/" << imageFileNamesList.at(inputId);
	successiveFileStream << imageFileNamesFolder << "/" << imageFileNamesList.at(inputId+1);

	cvLeftImage = cv::imread(nextFileStream.str(), cv::IMREAD_COLOR);
	cvRightImage = cv::imread(successiveFileStream.str(), cv::IMREAD_COLOR);
	ASSERT( cvLeftImage.size() == cvRightImage.size(), "Performance Test Error: input images do not have same size");

	if (leftFrame != NULL)
		{
		delete(leftFrame);
		}
	if (rightFrame != NULL)
		{
		delete(rightFrame);
		}
	MatToFrameConverter converter;
	leftFrame = converter.Convert(cvLeftImage);
	rightFrame = converter.Convert(cvRightImage);

	return true;	
	}

void VisualOdometry::ExecuteDfns()
	{
	VisualPointFeatureVector2DConstPtr leftKeypointsVector = NULL;
	VisualPointFeatureVector2DConstPtr rightKeypointsVector = NULL;
	VisualPointFeatureVector2DConstPtr leftFeaturesVector = NULL;
	VisualPointFeatureVector2DConstPtr rightFeaturesVector = NULL;
	CorrespondenceMap2DConstPtr correspondenceMap = NULL;
	Matrix3dConstPtr fundamentalMatrix = NULL;
	Pose3DConstPtr newPose = NULL;

	Execute(extractor, leftFrame, leftKeypointsVector);
	Execute(descriptor, leftFrame, leftKeypointsVector, leftFeaturesVector);
	Copy(*leftFeaturesVector, *leftFeaturesVectorHolder);

	Execute(extractor, rightFrame, rightKeypointsVector);
	Execute(descriptor, rightFrame, rightKeypointsVector, rightFeaturesVector);
	Copy(*rightFeaturesVector, *rightFeaturesVectorHolder);

	Execute(matcher, leftFeaturesVectorHolder, rightFeaturesVector, correspondenceMap);
	Copy(*correspondenceMap, *correspondenceMapHolder);

	Execute(matrixComputer, correspondenceMap, fundamentalMatrix, ransacSuccess);
	if(ransacSuccess)
		{
		Execute(poseEstimator, fundamentalMatrix, correspondenceMap, newPose, decompositionSuccess);
		}
	else
		{
		decompositionSuccess = false;
		}

	if (decompositionSuccess)
		{
		Copy(*newPose, pose);
		}
	}

VisualOdometry::MeasuresMap VisualOdometry::ExtractMeasures()
	{
	MeasuresMap measuresMap;

	measuresMap["RansacSuccess"] = ransacSuccess;
	measuresMap["DecompositionSuccess"] = decompositionSuccess;
	measuresMap["NumberOfLeftKeypoints"] = GetNumberOfPoints(*leftFeaturesVectorHolder);
	measuresMap["NumberOfRightKeypoints"] = GetNumberOfPoints(*rightFeaturesVectorHolder);
	measuresMap["NumberOfCorrespondences"] = GetNumberOfCorrespondences(*correspondenceMapHolder);
	measuresMap["MixedOrderPixelIndex"] = ComputeMixedOrderPixelIndex();

	if (decompositionSuccess)
		{
		Pose3D& leftPose = imagePosesList.at(inputId);

		/**** The output pose roll pitch and yaw seem to be opposite of the ground truth *****/
		float absolutePoseX = GetXPosition(pose) + GetXPosition(leftPose);
		float absolutePoseY = GetYPosition(pose) + GetYPosition(leftPose);
		float absolutePoseZ = GetZPosition(pose) + GetZPosition(leftPose);
		Eigen::Quaternionf poseQuaternion(GetWOrientation(pose), GetXOrientation(pose), GetYOrientation(pose), GetZOrientation(pose));
		Eigen::Quaternionf leftPoseQuaternion(GetWOrientation(leftPose), GetXOrientation(leftPose), GetYOrientation(leftPose), GetZOrientation(leftPose));
		float absolutePoseRoll = poseQuaternion.toRotationMatrix().eulerAngles(0, 1, 2)[0] + leftPoseQuaternion.toRotationMatrix().eulerAngles(0, 1, 2)[0];
		float absolutePosePitch = poseQuaternion.toRotationMatrix().eulerAngles(0, 1, 2)[1] + leftPoseQuaternion.toRotationMatrix().eulerAngles(0, 1, 2)[0];
		float absolutePoseYaw = poseQuaternion.toRotationMatrix().eulerAngles(0, 1, 2)[2] + leftPoseQuaternion.toRotationMatrix().eulerAngles(0, 1, 2)[0];

		Pose3D& rightPose = imagePosesList.at(inputId+1);
		float differenceX = GetXPosition(rightPose) - absolutePoseX;
		float differenceY = GetYPosition(rightPose) - absolutePoseY;
		float differenceZ = GetZPosition(rightPose) - absolutePoseZ;
		float squaredDistance = differenceX*differenceX + differenceY*differenceY + differenceZ*differenceZ;
		measuresMap["PositionDistance"] = std::sqrt(squaredDistance);

		Eigen::Quaternionf absolutePoseQuaternion =
			Eigen::AngleAxisf(absolutePoseRoll, Eigen::Vector3f::UnitX()) * 
			Eigen::AngleAxisf(absolutePosePitch, Eigen::Vector3f::UnitY()) * 
			Eigen::AngleAxisf(absolutePoseYaw, Eigen::Vector3f::UnitZ());
		float scalarProduct =
			GetXOrientation(rightPose) *	absolutePoseQuaternion.x() +
			GetYOrientation(rightPose) *	absolutePoseQuaternion.y() +
			GetZOrientation(rightPose) *	absolutePoseQuaternion.z() +
			GetWOrientation(rightPose) *	absolutePoseQuaternion.w();
		measuresMap["AngleDistace"] = 1 - scalarProduct*scalarProduct;
		}
	else
		{
		measuresMap["PositionDistance"] = 1;
		measuresMap["AngleDistance"] = 1;
		}

	return measuresMap;
	}

float VisualOdometry::ComputeMixedOrderPixelIndex()
	{
	SingleCorrespondence correspondencesList[MAX_CORRESPONDENCES_2D];

	for(unsigned correspondenceIndex = 0; correspondenceIndex < GetNumberOfCorrespondences(*correspondenceMapHolder); correspondenceIndex++)
		{
		BaseTypesWrapper::Point2D source = GetSource(*correspondenceMapHolder, correspondenceIndex);
		BaseTypesWrapper::Point2D sink = GetSink(*correspondenceMapHolder, correspondenceIndex);
		correspondencesList[correspondenceIndex].sourceX = source.x;
		correspondencesList[correspondenceIndex].sourceY = source.y;
		correspondencesList[correspondenceIndex].sinkX = sink.x;
		correspondencesList[correspondenceIndex].sinkX = sink.y;
		}
	
	std::qsort(correspondencesList, GetNumberOfCorrespondences(*correspondenceMapHolder), sizeof(SingleCorrespondence), VisualOdometry::SourceCompare);

	unsigned swapCounter = 0;
	for(unsigned loopIndex = 0; loopIndex < GetNumberOfCorrespondences(*correspondenceMapHolder); loopIndex++)
		{
		for(unsigned correspondenceIndex = loopIndex; correspondenceIndex < GetNumberOfCorrespondences(*correspondenceMapHolder)-1; correspondenceIndex++)
			{
			int compareScore = SinkCompare(correspondencesList[correspondenceIndex], correspondencesList[correspondenceIndex+1]);
			if (compareScore > 0)
				{
				SingleCorrespondence temporary = correspondencesList[correspondenceIndex];
				correspondencesList[correspondenceIndex] = correspondencesList[correspondenceIndex+1];
				correspondencesList[correspondenceIndex+1] = temporary;
				swapCounter++;
				}
			}
		}

	return swapCounter;
	}

int VisualOdometry::SourceCompare(const void* c1, const void* c2)
	{
	const SingleCorrespondence* correspondence1 = static_cast<const SingleCorrespondence*>(c1);
	const SingleCorrespondence* correspondence2 = static_cast<const SingleCorrespondence*>(c2);

	return VisualOdometry::SourceCompare( *correspondence1, *correspondence2);
	}

int VisualOdometry::SourceCompare(const SingleCorrespondence& correspondence1, const SingleCorrespondence& correspondence2)
	{
	if (correspondence1.sourceX < correspondence2.sourceX)
		{
		return -1;
		}
	else if (correspondence1.sourceX == correspondence2.sourceX)
		{
		if (correspondence1.sourceY < correspondence2.sourceY)
			{
			return -1;
			}
		else if (correspondence1.sourceY == correspondence2.sourceY)
			{
			return 0;
			}
		else
			{
			return 1;
			}
		}
	else
		{
		return 1;
		}
	}

int VisualOdometry::SinkCompare(const SingleCorrespondence& correspondence1, const SingleCorrespondence& correspondence2)
	{
	if (correspondence1.sinkX < correspondence2.sinkX)
		{
		return -1;
		}
	else if (correspondence1.sinkX == correspondence2.sinkX)
		{
		if (correspondence1.sinkY < correspondence2.sinkY)
			{
			return -1;
			}
		else if (correspondence1.sinkY == correspondence2.sinkY)
			{
			return 0;
			}
		else
			{
			return 1;
			}
		}
	else
		{
		return 1;
		}
	}

/** @} */
