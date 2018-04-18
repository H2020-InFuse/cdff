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
	protected:

	private:
		Stubs::CacheHandler<VisualPointFeatureVector2DConstPtr, cv::Mat >* stubVisualFeaturesCache;
		Mocks::VisualPointFeatureVector2DToMatConverter* mockVisualFeaturesConverter;

		Stubs::CacheHandler<cv::Mat, VisualPointFeatureVector2DConstPtr>* stubVisualFeaturesCache2;
		Mocks::MatToVisualPointFeatureVector2DConverter* mockVisualFeaturesConverter2;

		Stubs::CacheHandler<FrameConstPtr, cv::Mat>* stubInputCache;
		Mocks::FrameToMatConverter* mockInputConverter;

		Stubs::CacheHandler<cv::Mat, Pose3DConstPtr>* stubPoseCache;
		Mocks::MatToTransform3DConverter* mockPoseConverter;

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
		Aggregator* groundOrientationDistanceAggregator1;
		Aggregator* groundOrientationDistanceAggregator2;
		std::vector<std::string> imageFileNamesList;
		std::vector<Pose3D> posesList;
		std::vector<double> imageTimesList;
		std::vector<double> poseTimesList;
		std::vector<Pose3D> imagePosesList;

		static const std::string imageFileNamesFolder;
		static const std::string imagesFileContainer;
		static const std::string posesFileContainer;
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

		int long inputId;
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
	SetupMocksAndStubs();

	groundPositionDistanceAggregator = new Aggregator( Aggregator::AVERAGE );
	AddAggregator("PositionDistance", groundPositionDistanceAggregator, FIXED_PARAMETERS_VARIABLE_INPUTS);
	groundOrientationDistanceAggregator1 = new Aggregator( Aggregator::AVERAGE );
	AddAggregator("AngleDistace1", groundOrientationDistanceAggregator1, FIXED_PARAMETERS_VARIABLE_INPUTS);
	groundOrientationDistanceAggregator2 = new Aggregator( Aggregator::AVERAGE );
	AddAggregator("AngleDistace2", groundOrientationDistanceAggregator2, FIXED_PARAMETERS_VARIABLE_INPUTS);

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

	LoadImageFileNames();
	LoadPoses();
	ComputeImagePoses();
	ASSERT(imageFileNamesList.size() == imagePosesList.size(), "Number of displacement different from number of images");
	}

OrbFlannRansacDecomposition::~OrbFlannRansacDecomposition()
	{
	delete(stubInputCache);
	delete(mockInputConverter);
	delete(stubVisualFeaturesCache);
	delete(mockVisualFeaturesConverter);
	delete(stubVisualFeaturesCache2);
	delete(mockVisualFeaturesConverter2);
	delete(stubPoseCache);
	delete(mockPoseConverter);

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

const std::string OrbFlannRansacDecomposition::imageFileNamesFolder = "/media/alexander/Agridrive/rgbd_dataset_freiburg1_plant"; //"../tests/Data/Images";
const std::string OrbFlannRansacDecomposition::imagesFileContainer = "/media/alexander/Agridrive/rgbd_dataset_freiburg1_plant/rgb.txt"; //"../tests/Data/Images/imagesList.txt";
const std::string OrbFlannRansacDecomposition::posesFileContainer = "/media/alexander/Agridrive/rgbd_dataset_freiburg1_plant/groundtruth.txt"; //"../tests/Data/Images/posesList.txt";

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

void OrbFlannRansacDecomposition::SetupMocksAndStubs()
	{
	stubInputCache = new Stubs::CacheHandler<FrameConstPtr, cv::Mat>();
	mockInputConverter = new Mocks::FrameToMatConverter();
	ConversionCache<FrameConstPtr, cv::Mat, FrameToMatConverter>::Instance(stubInputCache, mockInputConverter);

	stubVisualFeaturesCache = new Stubs::CacheHandler<VisualPointFeatureVector2DConstPtr, cv::Mat>();
	mockVisualFeaturesConverter = new Mocks::VisualPointFeatureVector2DToMatConverter();
	ConversionCache<VisualPointFeatureVector2DConstPtr, cv::Mat, VisualPointFeatureVector2DToMatConverter>::Instance(stubVisualFeaturesCache, mockVisualFeaturesConverter);

	stubVisualFeaturesCache2 = new Stubs::CacheHandler<cv::Mat, VisualPointFeatureVector2DConstPtr>();
	mockVisualFeaturesConverter2 = new Mocks::MatToVisualPointFeatureVector2DConverter();
	ConversionCache<cv::Mat, VisualPointFeatureVector2DConstPtr, MatToVisualPointFeatureVector2DConverter>::Instance(stubVisualFeaturesCache2, mockVisualFeaturesConverter2);

	stubPoseCache = new Stubs::CacheHandler<cv::Mat, Pose3DConstPtr>();
	mockPoseConverter = new Mocks::MatToTransform3DConverter;
	ConversionCache<cv::Mat, Pose3DConstPtr, MatToTransform3DConverter>::Instance(stubPoseCache, mockPoseConverter);
	}

bool OrbFlannRansacDecomposition::SetNextInputs()
	{
	inputId++;
	if (inputId+1 >= imageFileNamesList.size())
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
	orb->imageInput(leftFrame);
	orb->process();
	leftFeaturesVector = orb->featuresSetOutput();

	if (rightFeaturesVector != NULL)
		{
		delete(rightFeaturesVector);
		}
	orb->imageInput(rightFrame);
	orb->process();
	rightFeaturesVector = orb->featuresSetOutput();

	if (correspondenceMap != NULL)
		{
		delete(correspondenceMap);
		}
	flann->sinkFeaturesVectorInput( leftFeaturesVector );
	flann->sourceFeaturesVectorInput( rightFeaturesVector );
	flann->process();
	correspondenceMap = flann->correspondenceMapOutput();

	if (fundamentalMatrix != NULL)
		{
		delete(fundamentalMatrix);
		}
	ransac->correspondenceMapInput(correspondenceMap);
	ransac->process();
	ransacSuccess = ransac->successOutput();
	fundamentalMatrix = ransac->fundamentalMatrixOutput();

	if (pose != NULL)
		{
		delete(pose);
		}
	if (ransacSuccess)
		{
		decomposition->fundamentalMatrixInput(fundamentalMatrix);
		decomposition->correspondenceMapInput(correspondenceMap);
		decomposition->process();
		decompositionSuccess = decomposition->successOutput();
		pose = decomposition->transformOutput();
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
		measuresMap["AngleDistace1"] = 1 - scalarProduct*scalarProduct;				

		Eigen::Quaternionf rightPoseQuaternion(GetWOrientation(rightPose), GetXOrientation(rightPose), GetYOrientation(rightPose), GetZOrientation(rightPose));
		float differenceRoll = 1 - std::cos(rightPoseQuaternion.toRotationMatrix().eulerAngles(0, 1, 2)[0] - absolutePoseRoll);
		float differencePitch = 1 - std::cos(rightPoseQuaternion.toRotationMatrix().eulerAngles(0, 1, 2)[1] - absolutePosePitch);
		float differenceYaw = 1 - std::cos(rightPoseQuaternion.toRotationMatrix().eulerAngles(0, 1, 2)[2] - absolutePoseYaw);
		measuresMap["AngleDistace2"] = differenceRoll + differencePitch + differenceYaw;
		}

	return measuresMap;
	}


int main(int argc, char** argv)
	{
	std::vector<std::string> baseConfigurationFiles =
		{
		"OrbExtractorDescriptor_PerformanceTest_1.yaml",
		"FlannMatcher_PerformanceTest_1.yaml",
		"FundamentalMatrixRansac_PerformanceTest_1.yaml",
		"EssentialMatrixDecomposition_PerformanceTest_2.yaml"
		};
	OrbFlannRansacDecomposition interface("../tests/ConfigurationFiles/DFNsIntegration/VisualOdometry", baseConfigurationFiles, "Orb_Flann_Ransac_Decomposition.txt");
	interface.Run();
	};

/** @} */
