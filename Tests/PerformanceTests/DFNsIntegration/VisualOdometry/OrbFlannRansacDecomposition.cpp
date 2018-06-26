/* --------------------------------------------------------------------------
*
* (C) Copyright …
*
* ---------------------------------------------------------------------------
*/

/*!
 * @file OrbFlannRansacDecomposition.cpp
 * @date 26/03/2018
 * @author Alessandro Bianco
 */

/*!
 * @addtogroup DFNsTest
 * 
 * Performance Test for the integration of Orb extractor descriptor, Flann matcher, fundamental matrix RANSAC, and essential matrix decomposition
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
#include <opencv2/highgui/highgui.hpp>
#include <opencv2/imgproc/imgproc.hpp>
#include <opencv2/core/core.hpp>
#include <boost/algorithm/string.hpp>
#include <fstream>
#include <string>

#include <FeaturesExtraction2D/OrbDetectorDescriptor.hpp>
#include <FeaturesMatching2D/FlannMatcher.hpp>
#include <FundamentalMatrixComputation/FundamentalMatrixRansac.hpp>
#include <CamerasTransformEstimation/EssentialMatrixDecomposition.hpp>

#include <Stubs/Common/ConversionCache/CacheHandler.hpp>
#include <ConversionCache/ConversionCache.hpp>
#include <Mocks/Common/Converters/FrameToMatConverter.hpp>
#include <Mocks/Common/Converters/VisualPointFeatureVector2DToMatConverter.hpp>
#include <Mocks/Common/Converters/MatToVisualPointFeatureVector2DConverter.hpp>
#include <Mocks/Common/Converters/MatToTransform3DConverter.hpp>
#include <MatToFrameConverter.hpp>

#include <Errors/Assert.hpp>
#include <PerformanceTests/DFNsIntegration/PerformanceTestInterface.hpp>
#include <PerformanceTests/Aggregator.hpp>

#include <Eigen/Geometry>


using namespace dfn_ci;
using namespace Common;
using namespace Converters;
using namespace FrameWrapper;
using namespace VisualPointFeatureVector2DWrapper;
using namespace CorrespondenceMap2DWrapper;
using namespace PoseWrapper;
using namespace MatrixWrapper;


class OrbFlannRansacDecomposition : public PerformanceTestInterface
	{
	public:
		OrbFlannRansacDecomposition(std::string folderPath, std::vector<std::string> baseConfigurationFileNamesList, std::string performanceMeasuresFileName);
		~OrbFlannRansacDecomposition();

		void SetInputFiles(std::string imageFileNamesFolder, std::string imagesFileContainer, std::string posesFileContainer);
		void LoadInputFiles();
		void SetImageLimit(unsigned imageLimit);
	protected:

	private:
		struct SingleCorrespondence
			{
			unsigned sourceX;
			unsigned sinkX;
			unsigned sourceY;
			unsigned sinkY;
			};

		cv::Mat cvLeftImage;
		cv::Mat cvRightImage;

		FrameConstPtr leftFrame;
		FrameConstPtr rightFrame;
		VisualPointFeatureVector2DConstPtr leftFeaturesVector;
		VisualPointFeatureVector2DConstPtr rightFeaturesVector;
		CorrespondenceMap2DConstPtr correspondenceMap;
		bool ransacSuccess;
		Matrix3dConstPtr fundamentalMatrix;
		bool decompositionSuccess;
		Pose3DConstPtr pose;

		OrbDetectorDescriptor* orb;
		FlannMatcher* flann;
		FundamentalMatrixRansac* ransac;
		EssentialMatrixDecomposition* decomposition;

		Aggregator* groundPositionDistanceAggregator;
		Aggregator* groundOrientationDistanceAggregator;
		Aggregator* leftKeypointsAggregator;
		Aggregator* rightKeypointsAggregator;
		Aggregator* correspondencesAggregator;
		std::vector<std::string> imageFileNamesList;
		std::vector<Pose3D> posesList;
		std::vector<double> imageTimesList;
		std::vector<double> poseTimesList;
		std::vector<Pose3D> imagePosesList;

		void LoadImageFileNames();
		void LoadPoses();
		void ComputeImagePoses();

		float initialX, initialY, initialZ;
		float initialRoll, initialPitch, initialYaw;
		
		Pose3D InterpolatePose(unsigned beforePoseIndex, unsigned afterPoseIndex, double imageTime);
		void SetupMocksAndStubs();

		bool SetNextInputs();
		void ExecuteDfns();
		MeasuresMap ExtractMeasures();

		float ComputeMixedOrderPixelIndex();

		int long inputId;
		std::string imageFileNamesFolder;
		std::string imagesFileContainer;
		std::string posesFileContainer;
		unsigned imageLimit;

		static int SourceCompare(const void* c1, const void* c2);
		static int SourceCompare(const SingleCorrespondence& correspondence1, const SingleCorrespondence& correspondence2);
		static int SinkCompare(const SingleCorrespondence& correspondence1, const SingleCorrespondence& correspondence2);
	};

OrbFlannRansacDecomposition::OrbFlannRansacDecomposition(std::string folderPath, std::vector<std::string> baseConfigurationFileNamesList, std::string performanceMeasuresFileName)
	: PerformanceTestInterface(folderPath, baseConfigurationFileNamesList, performanceMeasuresFileName)
	{
	orb = new OrbDetectorDescriptor();
	flann = new FlannMatcher();
	ransac = new FundamentalMatrixRansac();
	decomposition = new EssentialMatrixDecomposition();
	AddDfn(orb);
	AddDfn(flann);
	AddDfn(ransac);
	AddDfn(decomposition);

	groundPositionDistanceAggregator = new Aggregator( Aggregator::AVERAGE );
	AddAggregator("PositionDistance", groundPositionDistanceAggregator, FIXED_PARAMETERS_VARIABLE_INPUTS);
	groundOrientationDistanceAggregator = new Aggregator( Aggregator::AVERAGE );
	AddAggregator("AngleDistace", groundOrientationDistanceAggregator, FIXED_PARAMETERS_VARIABLE_INPUTS);

	leftKeypointsAggregator = new Aggregator( Aggregator::AVERAGE );
	AddAggregator("NumberOfLeftKeypoints", leftKeypointsAggregator, FIXED_PARAMETERS_VARIABLE_INPUTS);
	rightKeypointsAggregator = new Aggregator( Aggregator::AVERAGE );
	AddAggregator("NumberOfRightKeypoints", rightKeypointsAggregator, FIXED_PARAMETERS_VARIABLE_INPUTS);
	correspondencesAggregator = new Aggregator( Aggregator::AVERAGE );
	AddAggregator("NumberOfCorrespondences", correspondencesAggregator, FIXED_PARAMETERS_VARIABLE_INPUTS);

	leftFrame = NULL;
	rightFrame = NULL;
	leftFeaturesVector = NULL;
	rightFeaturesVector = NULL;
	correspondenceMap = NULL;
	fundamentalMatrix = NULL;
	pose = NULL;

	ransacSuccess = false;
	decompositionSuccess = false;
	inputId = -1;

	imageFileNamesFolder = "../tests/Data/Images";
	imagesFileContainer = "../tests/Data/Images/imagesList.txt";
	posesFileContainer = "../tests/Data/Images/posesList.txt";
	}

OrbFlannRansacDecomposition::~OrbFlannRansacDecomposition()
	{
	if (leftFrame != NULL)
		{
		delete(leftFrame);
		}
	if (rightFrame != NULL)
		{
		delete(rightFrame);
		}
	if (leftFeaturesVector != NULL)
		{
		delete(leftFeaturesVector);
		}
	if (rightFeaturesVector != NULL)
		{
		delete(rightFeaturesVector);
		}
	if (correspondenceMap != NULL)
		{
		delete(correspondenceMap);
		}
	if (fundamentalMatrix != NULL)
		{
		delete(fundamentalMatrix);
		}
	if (pose != NULL)
		{
		delete(pose);
		}
	}

void OrbFlannRansacDecomposition::SetInputFiles(std::string imageFileNamesFolder, std::string imagesFileContainer, std::string posesFileContainer)
	{
	this->imageFileNamesFolder = imageFileNamesFolder;
	this->imagesFileContainer = imagesFileContainer;
	this->posesFileContainer = posesFileContainer;
	}

void OrbFlannRansacDecomposition::LoadInputFiles()
	{
	LoadImageFileNames();
	LoadPoses();
	ComputeImagePoses();
	ASSERT(imageFileNamesList.size() == imagePosesList.size(), "Number of displacement different from number of images");
	}

void OrbFlannRansacDecomposition::SetImageLimit(unsigned imageLimit)
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

void OrbFlannRansacDecomposition::LoadImageFileNames()
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

void OrbFlannRansacDecomposition::LoadPoses()
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
	
		Pose3D pose;
		SetPosition(pose, std::stof( stringsList.at(1) ), std::stof( stringsList.at(2) ), std::stof( stringsList.at(3) ) );
		SetOrientation(pose, std::stof( stringsList.at(4) ), std::stof( stringsList.at(5) ), std::stof( stringsList.at(6) ), std::stof( stringsList.at(7) ) );
		posesList.push_back(pose);

		poseTimesList.push_back( std::stod( stringsList.at(0) ) );
		}

	containerFile.close();
	}

void OrbFlannRansacDecomposition::ComputeImagePoses()
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

Pose3D OrbFlannRansacDecomposition::InterpolatePose(unsigned beforePoseIndex, unsigned afterPoseIndex, double imageTime)
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

bool OrbFlannRansacDecomposition::SetNextInputs()
	{
	inputId++;
	if (inputId+1 >= imageLimit)
		{
		return false;
		}

	std::stringstream nextFileStream, successiveFileStream;
	nextFileStream << imageFileNamesFolder << "/" << imageFileNamesList.at(inputId);
	successiveFileStream << imageFileNamesFolder << "/" << imageFileNamesList.at(inputId+1);

	std::string string = nextFileStream.str();
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

void OrbFlannRansacDecomposition::ExecuteDfns()
	{
	if (leftFeaturesVector != NULL)
		{
		delete(leftFeaturesVector);
		}
	orb->frameInput(*leftFrame);
	orb->process();
	VisualPointFeatureVector2DPtr newLeftFeaturesVector = NewVisualPointFeatureVector2D();
	Copy( orb->featuresOutput(), *newLeftFeaturesVector);
	leftFeaturesVector = newLeftFeaturesVector;

	if (rightFeaturesVector != NULL)
		{
		delete(rightFeaturesVector);
		}
	orb->frameInput(*rightFrame);
	orb->process();
	VisualPointFeatureVector2DPtr newRightFeaturesVector = NewVisualPointFeatureVector2D();
	Copy( orb->featuresOutput(), *newRightFeaturesVector);
	rightFeaturesVector = newRightFeaturesVector;

	if (correspondenceMap != NULL)
		{
		delete(correspondenceMap);
		}
	flann->sinkFeaturesInput( *leftFeaturesVector );
	flann->sourceFeaturesInput( *rightFeaturesVector );
	flann->process();
	CorrespondenceMap2DPtr newCorrespondenceMap = NewCorrespondenceMap2D();
	Copy( flann->matchesOutput(), *newCorrespondenceMap);
	correspondenceMap = newCorrespondenceMap;

	if (fundamentalMatrix != NULL)
		{
		delete(fundamentalMatrix);
		}
	ransac->matchesInput(*correspondenceMap);
	ransac->process();
	Matrix3dPtr newFundamentalMatrix = NewMatrix3d();
	ransacSuccess = ransac->successOutput();
	Copy( ransac->fundamentalMatrixOutput(), *newFundamentalMatrix);
	fundamentalMatrix = newFundamentalMatrix;

	if (pose != NULL)
		{
		delete(pose);
		}
	if (ransacSuccess)
		{
		decomposition->fundamentalMatrixInput(*fundamentalMatrix);
		decomposition->matchesInput(*correspondenceMap);
		decomposition->process();
		decompositionSuccess = decomposition->successOutput();
		Pose3DPtr newPose = NewPose3D();
		Copy( decomposition->transformOutput(), *newPose);
		pose = newPose;
		}
	else
		{
		decompositionSuccess = false;
		pose = NULL;
		}
	}

OrbFlannRansacDecomposition::MeasuresMap OrbFlannRansacDecomposition::ExtractMeasures()
	{
	MeasuresMap measuresMap;

	measuresMap["RansacSuccess"] = ransacSuccess;
	measuresMap["DecompositionSuccess"] = decompositionSuccess;
	measuresMap["NumberOfLeftKeypoints"] = GetNumberOfPoints(*leftFeaturesVector);
	measuresMap["NumberOfRightKeypoints"] = GetNumberOfPoints(*rightFeaturesVector);
	measuresMap["NumberOfCorrespondences"] = GetNumberOfCorrespondences(*correspondenceMap);
	measuresMap["MixedOrderPixelIndex"] = ComputeMixedOrderPixelIndex();

	if (decompositionSuccess)
		{
		Pose3D& leftPose = imagePosesList.at(inputId);

		/**** The output pose roll pitch and yaw seem to be opposite of the ground truth *****/
		float absolutePoseX = GetXPosition(*pose) + GetXPosition(leftPose);
		float absolutePoseY = GetYPosition(*pose) + GetYPosition(leftPose);
		float absolutePoseZ = GetZPosition(*pose) + GetZPosition(leftPose);
		Eigen::Quaternionf poseQuaternion(GetWOrientation(*pose), GetXOrientation(*pose), GetYOrientation(*pose), GetZOrientation(*pose));
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

float OrbFlannRansacDecomposition::ComputeMixedOrderPixelIndex()
	{
	SingleCorrespondence correspondencesList[MAX_CORRESPONDENCES_2D];

	for(unsigned correspondenceIndex = 0; correspondenceIndex < GetNumberOfCorrespondences(*correspondenceMap); correspondenceIndex++)
		{
		BaseTypesWrapper::Point2D source = GetSource(*correspondenceMap, correspondenceIndex);
		BaseTypesWrapper::Point2D sink = GetSink(*correspondenceMap, correspondenceIndex);
		correspondencesList[correspondenceIndex].sourceX = source.x;
		correspondencesList[correspondenceIndex].sourceY = source.y;
		correspondencesList[correspondenceIndex].sinkX = sink.x;
		correspondencesList[correspondenceIndex].sinkX = sink.y;
		}
	
	std::qsort(correspondencesList, GetNumberOfCorrespondences(*correspondenceMap), sizeof(SingleCorrespondence), OrbFlannRansacDecomposition::SourceCompare);

	unsigned swapCounter = 0;
	for(unsigned loopIndex = 0; loopIndex < GetNumberOfCorrespondences(*correspondenceMap); loopIndex++)
		{
		for(unsigned correspondenceIndex = loopIndex; correspondenceIndex < GetNumberOfCorrespondences(*correspondenceMap)-1; correspondenceIndex++)
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

int OrbFlannRansacDecomposition::SourceCompare(const void* c1, const void* c2)
	{
	SingleCorrespondence* correspondence1 = (SingleCorrespondence*)c1;
	SingleCorrespondence* correspondence2 = (SingleCorrespondence*)c2;

	return OrbFlannRansacDecomposition::SourceCompare( *correspondence1, *correspondence2);
	}

int OrbFlannRansacDecomposition::SourceCompare(const SingleCorrespondence& correspondence1, const SingleCorrespondence& correspondence2)
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

int OrbFlannRansacDecomposition::SinkCompare(const SingleCorrespondence& correspondence1, const SingleCorrespondence& correspondence2)
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


int main(int argc, char** argv)
	{
	std::string essentialMatrixConfigurationFile = "EssentialMatrixDecomposition_PerformanceTest_2.yaml";
	if (argc >= 2)
		{
		essentialMatrixConfigurationFile = argv[1];
		}

	std::vector<std::string> baseConfigurationFiles =
		{
		"OrbExtractorDescriptor_PerformanceTest_1.yaml",
		"FlannMatcher_PerformanceTest_1.yaml",
		"FundamentalMatrixRansac_PerformanceTest_1.yaml",
		essentialMatrixConfigurationFile
		};
	OrbFlannRansacDecomposition interface("../tests/ConfigurationFiles/DFNsIntegration/VisualOdometry", baseConfigurationFiles, "Orb_Flann_Ransac_Decomposition.txt");

	if (argc >= 5)
		{
		std::string imageFileNamesFolder = argv[2];
		std::string imagesFileName = argv[3];
		std::string posesFileName = argv[4];
		std::stringstream imagesFileContainer, posesFileContainer;
		imagesFileContainer << imageFileNamesFolder << "/" << imagesFileName;
		posesFileContainer << imageFileNamesFolder << "/" << posesFileName;		
		interface.SetInputFiles(imageFileNamesFolder, imagesFileContainer.str(), posesFileContainer.str());
		}
	
	interface.LoadInputFiles();

	if (argc >= 6)
		{
		unsigned imageLimit = std::stoi(argv[5]);
		interface.SetImageLimit(imageLimit);
		}

	interface.Run();
	};

/** @} */
