/* --------------------------------------------------------------------------
*
* (C) Copyright …
*
* ---------------------------------------------------------------------------
*/

/*!
 * @file StereoReconstruction.cpp
 * @date 02/02/2018
 * @author Alessandro Bianco
 */

/*!
 * @addtogroup DFNsTest
 * 
 * Testing application for a Stereo Reconstruction Integration Test.
 * This test uses: DFN ImageUndistortion, DFN Orb2D, DFN Flann Matcher, DFN Fundamental Matrix, DFN Essential Matrix and DFN Triangulation
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
#include<pcl/io/ply_io.h>
#include <pcl/visualization/pcl_visualizer.h>
#include <ConversionCache/ConversionCache.hpp>

#include <Errors/Assert.hpp>
#include <GuiTests/ParametersInterface.hpp>
#include <GuiTests/MainInterface.hpp>
#include <GuiTests/DFNsIntegration/DFNsIntegrationTestInterface.hpp>

#include <ImageFiltering/ImageUndistortion.hpp>
#include <PointCloudReconstruction2DTo3D/Triangulation.hpp>
#include <FeaturesMatching2D/FlannMatcher.hpp>
#include <FeaturesExtraction2D/OrbDetectorDescriptor.hpp>
#include <FundamentalMatrixComputation/FundamentalMatrixRansac.hpp>
#include <CamerasTransformEstimation/EssentialMatrixDecomposition.hpp>

#include <FrameToMatConverter.hpp>
#include <MatToFrameConverter.hpp>
#include <PointCloudToPclPointCloudConverter.hpp>
#include <PclPointCloudToPointCloudConverter.hpp>

#include <Stubs/Common/ConversionCache/CacheHandler.hpp>
#include <Mocks/Common/Converters/FrameToMatConverter.hpp>
#include <Mocks/Common/Converters/MatToFrameConverter.hpp>
#include <Mocks/Common/Converters/MatToVisualPointFeatureVector2DConverter.hpp>
#include <Mocks/Common/Converters/VisualPointFeatureVector2DToMatConverter.hpp>
#include <Mocks/Common/Converters/PointCloudToPclPointCloudConverter.hpp>
#include <Mocks/Common/Converters/MatToTransform3DConverter.hpp>
#include <Mocks/Common/Converters/Transform3DToMatConverter.hpp>


using namespace dfn_ci;
using namespace Converters;
using namespace Common;
using namespace FrameWrapper;
using namespace VisualPointFeatureVector2DWrapper;
using namespace CorrespondenceMap2DWrapper;
using namespace PoseWrapper;
using namespace PointCloudWrapper;
using namespace MatrixWrapper;
using namespace BaseTypesWrapper;

class StereoReconstructionTestInterface : public DFNsIntegrationTestInterface
	{
	public:
		StereoReconstructionTestInterface(std::string dfnName, int buttonWidth, int buttonHeight);
		~StereoReconstructionTestInterface();
	protected:

	private:
		enum State
			{
			START,
			UNDISTORT_LEFT_IMAGE_DONE,
			UNDISTORT_RIGHT_IMAGE_DONE,
			ORB_LEFT_IMAGE_DONE,
			ORB_RIGHT_IMAGE_DONE,
			MATCHING_DONE,
			FUNDAMENTAL_DONE,
			POSE_DONE,
			END
			};
		State state;

		Stubs::CacheHandler<FrameConstPtr, cv::Mat>* stubFrameCache;
		Mocks::FrameToMatConverter* mockFrameConverter;
		Stubs::CacheHandler<cv::Mat, FrameConstPtr>* stubInverseFrameCache;
		Mocks::MatToFrameConverter* mockInverseFrameConverter;
		Stubs::CacheHandler<cv::Mat, VisualPointFeatureVector2DConstPtr>* stubMatToVectorCache;
		Mocks::MatToVisualPointFeatureVector2DConverter* mockMatToVectorConverter;
		Stubs::CacheHandler<VisualPointFeatureVector2DConstPtr, cv::Mat>* stubVectorToMatCache;
		Mocks::VisualPointFeatureVector2DToMatConverter* mockVectorToMatConverter;
		Stubs::CacheHandler<cv::Mat, Pose3DConstPtr>* stubEssentialPoseCache;
		Mocks::MatToPose3DConverter* mockEssentialPoseConverter;
		Stubs::CacheHandler<Pose3DConstPtr, cv::Mat>* stubTriangulationPoseCache;
		Mocks::Pose3DToMatConverter* mockTriangulationPoseConverter;
		

		ImageUndistortion* leftUndistortion;
		ImageUndistortion* rightUndistortion;
		OrbDetectorDescriptor* orb;
		FlannMatcher* flann;	
		FundamentalMatrixRansac* ransac;	
		EssentialMatrixDecomposition* essential;
		Triangulation* triangulation;

		std::string outputWindowName;
		cv::Mat leftCvImage;
		cv::Mat rightCvImage;

		FrameConstPtr leftImage;
		FrameConstPtr rightImage;
		FrameConstPtr undistortedLeftImage;
		FrameConstPtr undistortedRightImage;
		VisualPointFeatureVector2DConstPtr leftFeaturesVector;
		VisualPointFeatureVector2DConstPtr rightFeaturesVector;
		CorrespondenceMap2DConstPtr correspondenceMap;
		Matrix3dConstPtr fundamentalMatrix;
		Pose3DConstPtr secondCameraPose;

		void SetupMocksAndStubs();
		void SetupParameters();
		void DisplayResult();

		void ResetProcess();
		bool IsProcessCompleted();
		void UpdateState();
		DFNCommonInterface* PrepareNextDfn();

		void PrepareUndistortionLeft();
		void PrepareUndistortionRight();
		void PrepareOrbLeft();
		void PrepareOrbRight();
		void PrepareFlann();
		void PrepareRansac();
		void PrepareEssential();
		void PrepareTriangulation();

		void ExtractCalibrationParameters();
		void ExtractCalibrationParametersOneCamera();
		void VisualizeCorrespondences(CorrespondenceMap2DConstPtr correspondenceMap);
		void VisualizeFeatures(VisualPointFeatureVector2DConstPtr leftFeaturesVector, VisualPointFeatureVector2DConstPtr rightFeaturesVector);
	};

StereoReconstructionTestInterface::StereoReconstructionTestInterface(std::string integrationName, int buttonWidth, int buttonHeight)
	: DFNsIntegrationTestInterface(buttonWidth, buttonHeight)
	{
	//ExtractCalibrationParametersOneCamera();
	//ExtractCalibrationParameters();

	leftUndistortion = new ImageUndistortion();
	AddDFN(leftUndistortion, "leftUndistortion");

	rightUndistortion = new ImageUndistortion();
	AddDFN(rightUndistortion, "rightUndistortion");
	
	orb = new OrbDetectorDescriptor();
	AddDFN(orb, "orb");

	flann = new FlannMatcher();
	AddDFN(flann, "flann");
		
	ransac = new FundamentalMatrixRansac();
	AddDFN(ransac, "ransac");

	essential = new EssentialMatrixDecomposition();
	AddDFN(essential, "essential");
	
	triangulation = new Triangulation();
	AddDFN(triangulation, "triangulation");

	cv::Mat doubleImage = cv::imread("../../tests/Data/Images/SmestechLab.jpg", cv::IMREAD_COLOR);
	ASSERT(doubleImage.rows > 0 && doubleImage.cols > 0, "Test Error: failed to load image correctly");

	unsigned singleImageCols = doubleImage.cols/2;
	unsigned singleImageRows = doubleImage.rows;
	unsigned extractCols = singleImageCols;
	unsigned extractRows = singleImageRows;
	unsigned startRow = 0; //singleImageRows/4;
	unsigned startColumnLeft = 0; //singleImageCols/4;
	unsigned startColumnRight = singleImageCols; //singleImageCols/4 + singleImageCols;
	doubleImage(cv::Rect(startColumnLeft, startRow, extractCols, extractRows) ).copyTo(leftCvImage);
	doubleImage(cv::Rect(startColumnRight, startRow, extractCols, extractRows) ).copyTo(rightCvImage);

	outputWindowName = integrationName;
	}

StereoReconstructionTestInterface::~StereoReconstructionTestInterface()
	{
	delete(leftUndistortion);
	delete(rightUndistortion);
	delete(orb);
	delete(flann);
	delete(ransac);
	delete(essential);
	delete(triangulation);
	}

void StereoReconstructionTestInterface::SetupMocksAndStubs()
	{
	stubFrameCache = new Stubs::CacheHandler<FrameConstPtr, cv::Mat>();
	mockFrameConverter = new Mocks::FrameToMatConverter();
	ConversionCache<FrameConstPtr, cv::Mat, FrameToMatConverter>::Instance(stubFrameCache, mockFrameConverter);

	stubInverseFrameCache = new Stubs::CacheHandler<cv::Mat, FrameConstPtr>();
	mockInverseFrameConverter = new Mocks::MatToFrameConverter();
	ConversionCache<cv::Mat, FrameConstPtr, MatToFrameConverter>::Instance(stubInverseFrameCache, mockInverseFrameConverter);

	stubMatToVectorCache = new Stubs::CacheHandler<cv::Mat, VisualPointFeatureVector2DConstPtr>();
	mockMatToVectorConverter = new Mocks::MatToVisualPointFeatureVector2DConverter();
	ConversionCache<cv::Mat, VisualPointFeatureVector2DConstPtr, MatToVisualPointFeatureVector2DConverter>::Instance(stubMatToVectorCache, mockMatToVectorConverter);

	stubVectorToMatCache = new Stubs::CacheHandler<VisualPointFeatureVector2DConstPtr, cv::Mat>();
	mockVectorToMatConverter = new Mocks::VisualPointFeatureVector2DToMatConverter();
	ConversionCache<VisualPointFeatureVector2DConstPtr, cv::Mat, VisualPointFeatureVector2DToMatConverter>::Instance(stubVectorToMatCache, mockVectorToMatConverter);

	stubEssentialPoseCache = new Stubs::CacheHandler<cv::Mat, Pose3DConstPtr>();
	mockEssentialPoseConverter = new Mocks::MatToPose3DConverter();
	ConversionCache<cv::Mat, Pose3DConstPtr, MatToPose3DConverter>::Instance(stubEssentialPoseCache, mockEssentialPoseConverter);

	stubTriangulationPoseCache = new Stubs::CacheHandler<Pose3DConstPtr, cv::Mat>();
	mockTriangulationPoseConverter = new Mocks::Pose3DToMatConverter();
	ConversionCache<Pose3DConstPtr, cv::Mat, Pose3DToMatConverter>::Instance(stubTriangulationPoseCache, mockTriangulationPoseConverter);
	}

void StereoReconstructionTestInterface::SetupParameters()
	{
	AddParameter(leftUndistortion, "CameraMatrix", "FocalLengthX", 1408.899186439272, 1500, 1e-5);
	AddParameter(leftUndistortion, "CameraMatrix", "FocalLengthY", 1403.116708010621, 1500, 1e-5);
	AddParameter(leftUndistortion, "CameraMatrix", "PrinciplePointX", 1053.351342078365, 1500, 1e-5);
	AddParameter(leftUndistortion, "CameraMatrix", "PrinciplePointY", 588.8342842821718, 1500, 1e-5);

	AddSignedParameter(leftUndistortion, "Distortion", "K1", 0.8010323519594021, 10, 1e-8);
	AddSignedParameter(leftUndistortion, "Distortion", "K2", 104.7894482598434, 110, 1e-6);
	AddSignedParameter(leftUndistortion, "Distortion", "K3", -70.73535010082334, 110, 1e-6);
	AddSignedParameter(leftUndistortion, "Distortion", "K4", 0.8556124926892613, 10, 1e-8);
	AddSignedParameter(leftUndistortion, "Distortion", "K5", 105.9684960970509, 110, 1e-6);
	AddSignedParameter(leftUndistortion, "Distortion", "K6", -60.88515255428263, 110, 1e-6);
	AddSignedParameter(leftUndistortion, "Distortion", "P1", -6.223427196342877e-05, 10, 1e-8);
	AddSignedParameter(leftUndistortion, "Distortion", "P2", -0.002209798328517273, 10, 1e-8);
	AddParameter(leftUndistortion, "Distortion", "UseK3", 1, 1);
	AddParameter(leftUndistortion, "Distortion", "UseK4ToK6", 1, 1);

	AddParameter(rightUndistortion, "CameraMatrix", "FocalLengthX", 1415.631284126374, 1500, 1e-5);
	AddParameter(rightUndistortion, "CameraMatrix", "FocalLengthY", 1408.026118461406, 1500, 1e-5);
	AddParameter(rightUndistortion, "CameraMatrix", "PrinciplePointX", 1013.347852589407, 1500, 1e-5);
	AddParameter(rightUndistortion, "CameraMatrix", "PrinciplePointY", 592.5031927882591, 1500, 1e-5);

	AddSignedParameter(rightUndistortion, "Distortion", "K1", -5.700997352957169, 10, 1e-8);
	AddSignedParameter(rightUndistortion, "Distortion", "K2", 9.016454056014156, 10, 1e-8);
	AddSignedParameter(rightUndistortion, "Distortion", "K3", -2.465929585147688, 10, 1e-8);
	AddSignedParameter(rightUndistortion, "Distortion", "K4", -5.560701053165053, 10, 1e-8);
	AddSignedParameter(rightUndistortion, "Distortion", "K5", 8.264481221246962, 10, 1e-8);
	AddSignedParameter(rightUndistortion, "Distortion", "K6", -1.458304668407831, 10, 1e-8);
	AddSignedParameter(rightUndistortion, "Distortion", "P1", 0.001056515495810514, 10, 1e-8);
	AddSignedParameter(rightUndistortion, "Distortion", "P2", 0.002542555946247054, 10, 1e-8);
	AddParameter(rightUndistortion, "Distortion", "UseK3", 1, 1);
	AddParameter(rightUndistortion, "Distortion", "UseK4ToK6", 1, 1);

	AddParameter(orb, "GeneralParameters", "EdgeThreshold", 31, 100);
	AddParameter(orb, "GeneralParameters", "FastThreshold", 20, 100);
	AddParameter(orb, "GeneralParameters", "FirstLevel", 0, 2);
	AddParameter(orb, "GeneralParameters", "MaxFeaturesNumber", 500, 1000, 10);
	AddParameter(orb, "GeneralParameters", "LevelsNumber", 8, 20);
	AddParameter(orb, "GeneralParameters", "PatchSize", 31, 100);
	AddParameter(orb, "GeneralParameters", "ScaleFactor", 1.2, 10, 0.1);
	AddParameter(orb, "GeneralParameters", "ScoreType", 0, 2);
	AddParameter(orb ,"GeneralParameters", "SizeOfBrightnessTestSet", 2, 4);

	AddParameter(flann, "GeneralParameters", "DistanceThreshold", 0.02, 1.00, 0.01);
	AddParameter(flann, "GeneralParameters", "MatcherMethod", 4, 6);
	AddParameter(flann, "GeneralParameters", "AcceptanceRatio", 0.75, 1, 0.01);
	AddParameter(flann, "LocalitySensitiveHashingParameters", "TableNumber", 6, 20);
	AddParameter(flann, "LocalitySensitiveHashingParameters", "KeySize", 12, 20);
	AddParameter(flann, "LocalitySensitiveHashingParameters", "MultiProbeLevel", 1, 20);

	AddParameter(ransac, "GeneralParameters", "OutlierThreshold", 1, 100);
	AddParameter(ransac, "GeneralParameters", "Confidence", 0.9, 1, 0.01);

	AddParameter(essential, "GeneralParameters", "NumberOfTestPoints", 20, 100);
	AddParameter(essential, "FirstCameraMatrix", "FocalLengthX", 1408.899186439272, 1500, 1e-5);
	AddParameter(essential, "FirstCameraMatrix", "FocalLengthY", 1403.116708010621, 1500, 1e-5);
	AddParameter(essential, "FirstCameraMatrix", "PrinciplePointX", 1053.351342078365, 1500, 1e-5);
	AddParameter(essential, "FirstCameraMatrix", "PrinciplePointY", 588.8342842821718, 1500, 1e-5);
	AddParameter(essential, "SecondCameraMatrix", "FocalLengthX", 1415.631284126374, 1500, 1e-5);
	AddParameter(essential, "SecondCameraMatrix", "FocalLengthY", 1408.026118461406, 1500, 1e-5);
	AddParameter(essential, "SecondCameraMatrix", "PrinciplePointX", 1013.347852589407, 1500, 1e-5);
	AddParameter(essential, "SecondCameraMatrix", "PrinciplePointY", 592.5031927882591, 1500, 1e-5);

	AddParameter(triangulation, "FirstCameraMatrix", "FocalLengthX", 1408.899186439272, 1500, 1e-5);
	AddParameter(triangulation, "FirstCameraMatrix", "FocalLengthY", 1403.116708010621, 1500, 1e-5);
	AddParameter(triangulation, "FirstCameraMatrix", "PrinciplePointX", 1053.351342078365, 1500, 1e-5);
	AddParameter(triangulation, "FirstCameraMatrix", "PrinciplePointY", 588.8342842821718, 1500, 1e-5);
	AddParameter(triangulation, "SecondCameraMatrix", "FocalLengthX", 1415.631284126374, 1500, 1e-5);
	AddParameter(triangulation, "SecondCameraMatrix", "FocalLengthY", 1408.026118461406, 1500, 1e-5);
	AddParameter(triangulation, "SecondCameraMatrix", "PrinciplePointX", 1013.347852589407, 1500, 1e-5);
	AddParameter(triangulation, "SecondCameraMatrix", "PrinciplePointY", 592.5031927882591, 1500, 1e-5);
	}

void StereoReconstructionTestInterface::DisplayResult()
	{
	PointCloudWrapper::PointCloudConstPtr pointCloud = triangulation->pointCloudOutput();

	PRINT_TO_LOG("The processing took (seconds): ", GetTotalProcessingTimeSeconds() );
	PRINT_TO_LOG("Virtual Memory used (Kb): ", GetTotalVirtualMemoryUsedKB() );
	PRINT_TO_LOG("Number of points: ", GetNumberOfPoints(*pointCloud) );

	pcl::PointCloud<pcl::PointXYZ>::Ptr pclPointCloud = boost::make_shared<pcl::PointCloud<pcl::PointXYZ> >();
	for(int pointIndex = 0; pointIndex < GetNumberOfPoints(*pointCloud); pointIndex++)
		{
		pcl::PointXYZ newPoint(GetXCoordinate(*pointCloud, pointIndex), GetYCoordinate(*pointCloud, pointIndex), GetZCoordinate(*pointCloud, pointIndex) );
		std::stringstream stream;
		pclPointCloud->points.push_back(newPoint);

		stream << "Point "<<pointIndex<<": ("<<newPoint.x<<", "<<newPoint.y<<", "<<newPoint.z<<")";
		std::string string = stream.str();
		PRINT_TO_LOG("", string );
		}

	pcl::visualization::PCLVisualizer viewer (outputWindowName);
    	pcl::visualization::PointCloudColorHandlerCustom<pcl::PointXYZ> pclCloudColor(pclPointCloud, 255, 255, 255);
    	viewer.addPointCloud(pclPointCloud,pclCloudColor,"input");

    	while (!viewer.wasStopped ())
    		{
        	viewer.spinOnce();
        	pcl_sleep (0.01);
    		} 

	delete(leftImage);
	delete(rightImage);
	delete(undistortedLeftImage);
	delete(undistortedRightImage);
	delete(leftFeaturesVector);
	delete(rightFeaturesVector);
	delete(correspondenceMap);
	delete(fundamentalMatrix);
	delete(secondCameraPose);
	delete(pointCloud);
	}

void StereoReconstructionTestInterface::ResetProcess()
	{
	state = START;
	}

bool StereoReconstructionTestInterface::IsProcessCompleted()
	{
	if (state == END)
		{
		PRINT_TO_LOG("Step Triangulation processing time (seconds): ", GetLastProcessingTimeSeconds(4) );
		return true;
		}
	return false;
	}

void StereoReconstructionTestInterface::UpdateState()
	{
	if (state == START)
		{
		state = UNDISTORT_LEFT_IMAGE_DONE;
		}
	else if (state == UNDISTORT_LEFT_IMAGE_DONE)
		{
		state = UNDISTORT_RIGHT_IMAGE_DONE;
		}
	else if (state == UNDISTORT_RIGHT_IMAGE_DONE)
		{
		state = ORB_LEFT_IMAGE_DONE;
		}
	else if (state == ORB_LEFT_IMAGE_DONE)
		{
		state = ORB_RIGHT_IMAGE_DONE;
		}
	else if (state == ORB_RIGHT_IMAGE_DONE)
		{
		state = MATCHING_DONE;
		}
	else if (state == MATCHING_DONE)
		{
		state = FUNDAMENTAL_DONE;
		}
	else if (state == FUNDAMENTAL_DONE)
		{
		state = POSE_DONE;
		}
	}

DFNCommonInterface* StereoReconstructionTestInterface::PrepareNextDfn()
	{
	if (state == START)
		{
		PrepareUndistortionLeft();
		return leftUndistortion;
		}
	else if (state == UNDISTORT_LEFT_IMAGE_DONE)
		{
		PrepareUndistortionRight();
		return rightUndistortion;
		}
	else if (state == UNDISTORT_RIGHT_IMAGE_DONE)
		{
		PrepareOrbLeft();
		return orb;
		}
	else if (state == ORB_LEFT_IMAGE_DONE)
		{
		PRINT_TO_LOG("Step Orb Left processing time (seconds): ", GetLastProcessingTimeSeconds(0) );
		PrepareOrbRight();
		return orb;
		}
	else if (state == ORB_RIGHT_IMAGE_DONE)
		{
		PRINT_TO_LOG("Step Orb Right processing time (seconds): ", GetLastProcessingTimeSeconds(1) );
		PrepareFlann();
		return flann;
		}
	else if (state == MATCHING_DONE)
		{
		PRINT_TO_LOG("Step Flann processing time (seconds): ", GetLastProcessingTimeSeconds(2) );
		PrepareRansac();
		return ransac;
		}
	else if (state == FUNDAMENTAL_DONE)
		{
		PRINT_TO_LOG("Step Ransac processing time (seconds): ", GetLastProcessingTimeSeconds(2) );
		PrepareEssential();
		return essential;
		}
	else if (state == POSE_DONE)
		{
		PRINT_TO_LOG("Step Essential processing time (seconds): ", GetLastProcessingTimeSeconds(3) );
		PrepareTriangulation();
		state = END;
		return triangulation;
		}
	ASSERT(false, "Unhandled State!");
	return 0;
	}

void StereoReconstructionTestInterface::PrepareUndistortionLeft()
	{
	MatToFrameConverter converter;
	leftImage = converter.Convert(leftCvImage);
	leftUndistortion->imageInput(leftImage);
	}

void StereoReconstructionTestInterface::PrepareUndistortionRight()
	{
	MatToFrameConverter converter;
	rightImage = converter.Convert(rightCvImage);
	rightUndistortion->imageInput(rightImage);
	}

void StereoReconstructionTestInterface::PrepareOrbLeft()
	{
	undistortedLeftImage = leftUndistortion->filteredImageOutput();

	MatToFrameConverter converter;
	leftImage = converter.Convert(leftCvImage);
	orb->imageInput(undistortedLeftImage);
	}

void StereoReconstructionTestInterface::PrepareOrbRight()
	{
	undistortedRightImage = rightUndistortion->filteredImageOutput();
	leftFeaturesVector = orb->featuresSetOutput();
	PRINT_TO_LOG("Number of features points from left image: ", GetNumberOfPoints(*leftFeaturesVector));

	MatToFrameConverter converter;
	rightImage = converter.Convert(rightCvImage);
	orb->imageInput(undistortedRightImage);
	}

void StereoReconstructionTestInterface::PrepareFlann()
	{
	rightFeaturesVector = orb->featuresSetOutput();
	PRINT_TO_LOG("Number of features points from right image: ", GetNumberOfPoints(*rightFeaturesVector));
	VisualizeFeatures(leftFeaturesVector, rightFeaturesVector);

	flann->sourceFeaturesVectorInput(leftFeaturesVector);
	flann->sinkFeaturesVectorInput(rightFeaturesVector);
	}

void StereoReconstructionTestInterface::PrepareRansac()
	{
	correspondenceMap = flann->correspondenceMapOutput();
	VisualizeCorrespondences(correspondenceMap);
	PRINT_TO_LOG("Number of correspondences from flann matcher: ", GetNumberOfCorrespondences(*correspondenceMap));

	ransac->correspondenceMapInput(correspondenceMap);
	}

void StereoReconstructionTestInterface::PrepareEssential()
	{
	bool success = ransac->successOutput();
	ASSERT(success, "Fundamental Matrix Ransac failed: unable to find a valid transform");

	fundamentalMatrix = ransac->fundamentalMatrixOutput();
	PRINT_TO_LOG("Fundamental matrix Ransac found transform: ", "");
	
	essential->fundamentalMatrixInput(fundamentalMatrix);
	essential->correspondenceMapInput(correspondenceMap);	
	}

void StereoReconstructionTestInterface::PrepareTriangulation()
	{
	secondCameraPose = essential->transformOutput();

	triangulation->poseInput(secondCameraPose);
	triangulation->correspondenceMapInput(correspondenceMap);
	}


/*
This is the output of the following method:
	cv::Mat rotationTranslationMatrix(3, 4, CV_32FC1, cv::Scalar(0));
	rotationTranslationMatrix.at<float>(0,0) = 0.453266151496562; 
	rotationTranslationMatrix.at<float>(0,1) = -0.4189590134265467;
	rotationTranslationMatrix.at<float>(0,2) = 0.7867802367727283;
 	rotationTranslationMatrix.at<float>(1,0) = -0.03139148385142432;
	rotationTranslationMatrix.at<float>(1,1) = 0.8746086689785711; 
	rotationTranslationMatrix.at<float>(1,2) = 0.4838122062217302;
 	rotationTranslationMatrix.at<float>(2,0) = -0.8908223002648179;
	rotationTranslationMatrix.at<float>(2,1) = -0.2439938958584555;
	rotationTranslationMatrix.at<float>(2,2) = 0.3832787603490593;
	rotationTranslationMatrix.at<float>(0,3) = -2.042263939854534;
	rotationTranslationMatrix.at<float>(1,3) = -1.342189200820332;
	rotationTranslationMatrix.at<float>(2,3) = 0.7502406403804528;

	cv::Mat leftCameraMatrixS(3, 3, CV_32FC1, cv::Scalar(0));
	leftCameraMatrix.at<float>(0, 0) = 9245.469388977166;
	leftCameraMatrix.at<float>(1, 1) = 20554.77101321935;
	leftCameraMatrix.at<float>(0, 2) = -1010.674087871778;
	leftCameraMatrix.at<float>(1, 2) = -2562.301106695449;
	leftCameraMatrix.at<float>(2, 2) = 1.0;

	cv::Mat rightCameraMatrixS(3, 3, CV_32FC1, cv::Scalar(0));
	rightCameraMatrix.at<float>(0, 0) = 947.2017949977272;
	rightCameraMatrix.at<float>(1, 1) = 1657.602441223055;
	rightCameraMatrix.at<float>(0, 2) = 3026.416358478908;
	rightCameraMatrix.at<float>(1, 2) = 257.3840612345921;
	rightCameraMatrix.at<float>(2, 2) = 1.0;
*/
void StereoReconstructionTestInterface::ExtractCalibrationParameters()
	{
	static const unsigned ROW_NUMBER = 7;
	static const unsigned COLUMN_NUMBER = 11;
	static const float SQUARE_EDGE_LENGTH = 0.02;

 	std::vector<std::vector<cv::Point2f> > leftCornersList(11);
 	std::vector<std::vector<cv::Point2f> > rightCornersList(11);

	cv::Size imageSize;
	for(unsigned imageId=1; imageId<=11; imageId++)
		{
		std::stringstream filePath;
		filePath<<"../../tests/Data/Images/chessboard"<<imageId<<".jpg";
		cv::Mat stereoImage = cv::imread(filePath.str(), cv::IMREAD_COLOR);
		cv::Mat leftImage = stereoImage( cv::Rect(0, 0, stereoImage.cols/2, stereoImage.rows) );
		cv::Mat rightImage = stereoImage( cv::Rect(stereoImage.cols/2, 0, stereoImage.cols/2, stereoImage.rows) );
		ASSERT(leftImage.rows == rightImage.rows && leftImage.cols == rightImage.cols, "Error, left and right image should have same size");
		
		if (imageId == 1)
			{
			imageSize = leftImage.size();
			}
		else
			{
			ASSERT(imageSize == leftImage.size(), "Images in the set do not have same size");
			}

		bool leftFlag = cv::findChessboardCorners(leftImage, cv::Size(ROW_NUMBER, COLUMN_NUMBER), leftCornersList[imageId-1]);
		bool rightFlag = cv::findChessboardCorners(rightImage, cv::Size(ROW_NUMBER, COLUMN_NUMBER), rightCornersList[imageId-1]);
		ASSERT(leftFlag && rightFlag, "Camera calibration failed");

		//cv::Mat img = leftImage.clone();
		//cv::drawChessboardCorners(img, cv::Size(7, 11), leftCornersList[imageId-1], leftFlag);
		//cv::imshow("img", img);
		//cv::waitKey(0);
		}

	std::vector< std::vector<cv::Point3f> > objectPoints(11);
	for(unsigned column=0; column<COLUMN_NUMBER; column++) 
		{
		for(unsigned row=0; row<ROW_NUMBER; row++)
			{
			for(unsigned imageId = 1; imageId<=11; imageId++)
				{
				cv::Point3f point((float)column*SQUARE_EDGE_LENGTH, (float)row*SQUARE_EDGE_LENGTH, 0);
				objectPoints[imageId-1].push_back( point );
				}
			}
		}

	cv::Mat leftCameraMatrix = cv::Mat::eye(3, 3, CV_64FC1);
	cv::Mat rightCameraMatrix = cv::Mat::eye(3, 3, CV_64FC1);

	cv::Mat leftDistortionCoefficients, rightDistortionCoefficients;
	cv::Mat rotationMatrix, translationMatrix, essentialMatrix, fundamentalMatrix;

	cv::stereoCalibrate
		(
		objectPoints,
		leftCornersList,
		rightCornersList,
		leftCameraMatrix,
		leftDistortionCoefficients,
		rightCameraMatrix,
		rightDistortionCoefficients,
		imageSize,
		rotationMatrix,
		translationMatrix,
		essentialMatrix,
		fundamentalMatrix,
		CV_CALIB_RATIONAL_MODEL,
		cv::TermCriteria(CV_TERMCRIT_ITER+CV_TERMCRIT_EPS, 1000, 1e-5)
		);
	PRINT_TO_LOG("Calibration complete", "");

	PRINT_TO_LOG("Left calibration: ", leftCameraMatrix);
	PRINT_TO_LOG("Right calibration: ", rightCameraMatrix);
	PRINT_TO_LOG("Left distortion: ", leftDistortionCoefficients);
	PRINT_TO_LOG("Right distortion: ", rightDistortionCoefficients);
	PRINT_TO_LOG("Translation: ", translationMatrix);
	PRINT_TO_LOG("Rotation: ", rotationMatrix);

	cv::Mat rototranslationMatrix(3, 4, CV_64FC1, cv::Scalar(0));
	rototranslationMatrix.at<double>(0,0) = rotationMatrix.at<double>(0,0);
	rototranslationMatrix.at<double>(0,1) = rotationMatrix.at<double>(0,1);
	rototranslationMatrix.at<double>(0,2) = rotationMatrix.at<double>(0,2);
	rototranslationMatrix.at<double>(1,0) = rotationMatrix.at<double>(1,0);
	rototranslationMatrix.at<double>(1,1) = rotationMatrix.at<double>(1,1);
	rototranslationMatrix.at<double>(1,2) = rotationMatrix.at<double>(1,2);
	rototranslationMatrix.at<double>(2,0) = rotationMatrix.at<double>(2,0);
	rototranslationMatrix.at<double>(2,1) = rotationMatrix.at<double>(1,1);
	rototranslationMatrix.at<double>(2,2) = rotationMatrix.at<double>(2,1);
	rototranslationMatrix.at<double>(0,3) = translationMatrix.at<double>(0);
	rototranslationMatrix.at<double>(1,3) = translationMatrix.at<double>(1);
	rototranslationMatrix.at<double>(2,3) = translationMatrix.at<double>(2);

	cv::Mat leftProjectionMatrix = leftCameraMatrix*rototranslationMatrix;
	cv::Mat rightProjectionMatrix = rightCameraMatrix*rototranslationMatrix;
	PRINT_TO_LOG("Left Projection Matrix", leftProjectionMatrix);
	PRINT_TO_LOG("Right Projection Matrix", rightProjectionMatrix);	

	cv::Mat leftRectificationMatrix, rightRectificationMatrix, leftRectifiedProjectionMatrix, rightRectifiedProjectionMatrix, disparityToDepthMatrix;
	cv::stereoRectify
		(
		leftCameraMatrix,
		leftDistortionCoefficients,
		rightCameraMatrix,
		rightDistortionCoefficients,
		imageSize,
		rotationMatrix,
		translationMatrix,
		leftRectificationMatrix,
		rightRectificationMatrix,
		leftRectifiedProjectionMatrix,
		rightRectifiedProjectionMatrix,
		disparityToDepthMatrix
		);

	PRINT_TO_LOG("Rectified Left Projection Matrix", leftRectifiedProjectionMatrix);
	PRINT_TO_LOG("Rectified Right Projection Matrix", rightRectifiedProjectionMatrix);	
	}


/* This is the output of the following method:
	cv::Mat leftCameraMatrix(3, 3, CV_32FC1, cv::Scalar(0));
	leftCameraMatrix.at<float>(0, 0) = 1408.899186439272;
	leftCameraMatrix.at<float>(1, 1) = 1403.116708010621;
	leftCameraMatrix.at<float>(0, 2) = 1053.351342078365;
	leftCameraMatrix.at<float>(1, 2) = 588.8342842821718;
	leftCameraMatrix.at<float>(2, 2) = 1.0;

	cv::Mat rightCameraMatrix(3, 3, CV_32FC1, cv::Scalar(0));
	rightCameraMatrix.at<float>(0, 0) = 1415.631284126374;
	rightCameraMatrix.at<float>(1, 1) = 1408.026118461406;
	rightCameraMatrix.at<float>(0, 2) = 1013.347852589407;
	rightCameraMatrix.at<float>(1, 2) = 592.5031927882591;
	rightCameraMatrix.at<float>(2, 2) = 1.0;

Left distortion:  [0.8010323519594021, 104.7894482598434, -6.223427196342877e-05, -0.002209798328517273, -70.73535010082334, 0.8556124926892613, 105.9684960970509, -60.88515255428263, 0, 0, 0, 0, 0, 0]
Right distortion:  [-5.700997352957169, 9.016454056014156, 0.001056515495810514, 0.002542555946247054, -2.465929585147688, -5.560701053165053, 8.264481221246962, -1.458304668407831, 0, 0, 0, 0, 0, 0]

*/
void StereoReconstructionTestInterface::ExtractCalibrationParametersOneCamera()
	{
	static const unsigned ROW_NUMBER = 7;
	static const unsigned COLUMN_NUMBER = 11;
	static const float SQUARE_EDGE_LENGTH = 0.02;

 	std::vector<std::vector<cv::Point2f> > leftCornersList(11);
 	std::vector<std::vector<cv::Point2f> > rightCornersList(11);

	cv::Size imageSize;
	for(unsigned imageId=1; imageId<=11; imageId++)
		{
		std::stringstream filePath;
		filePath<<"../../tests/Data/Images/chessboard"<<imageId<<".jpg";
		cv::Mat stereoImage = cv::imread(filePath.str(), cv::IMREAD_COLOR);
		cv::Mat leftImage = stereoImage( cv::Rect(0, 0, stereoImage.cols/2, stereoImage.rows) );
		cv::Mat rightImage = stereoImage( cv::Rect(stereoImage.cols/2, 0, stereoImage.cols/2, stereoImage.rows) );
		ASSERT(leftImage.rows == rightImage.rows && leftImage.cols == rightImage.cols, "Error, left and right image should have same size");
		
		if (imageId == 1)
			{
			imageSize = leftImage.size();
			}
		else
			{
			ASSERT(imageSize == leftImage.size(), "Images in the set do not have same size");
			}

		bool leftFlag = cv::findChessboardCorners(leftImage, cv::Size(ROW_NUMBER, COLUMN_NUMBER), leftCornersList[imageId-1]);
		bool rightFlag = cv::findChessboardCorners(rightImage, cv::Size(ROW_NUMBER, COLUMN_NUMBER), rightCornersList[imageId-1]);
		ASSERT(leftFlag && rightFlag, "Camera calibration failed");
		}

	std::vector< std::vector<cv::Point3f> > objectPoints(11);
	for(unsigned column=0; column<COLUMN_NUMBER; column++) 
		{
		for(unsigned row=0; row<ROW_NUMBER; row++)
			{
			for(unsigned imageId = 1; imageId<=11; imageId++)
				{
				cv::Point3f point((float)column*SQUARE_EDGE_LENGTH, (float)row*SQUARE_EDGE_LENGTH, 0);
				objectPoints[imageId-1].push_back( point );
				}
			}
		}

	cv::Mat leftCameraMatrix = cv::Mat::eye(3, 3, CV_64FC1);
	cv::Mat rightCameraMatrix = cv::Mat::eye(3, 3, CV_64FC1);

	cv::Mat leftDistortionCoefficients, rightDistortionCoefficients;
	cv::Mat leftRotationMatrix, leftTranslationMatrix, rightRotationMatrix, rightTranslationMatrix;

	cv::calibrateCamera
		(
		objectPoints,
		leftCornersList,
		imageSize,
		leftCameraMatrix,
		leftDistortionCoefficients,
		leftRotationMatrix,
		leftTranslationMatrix,
		CV_CALIB_RATIONAL_MODEL,
		cv::TermCriteria(CV_TERMCRIT_ITER+CV_TERMCRIT_EPS, 1000, 1e-5)
		);
	cv::calibrateCamera
		(
		objectPoints,
		rightCornersList,
		imageSize,
		rightCameraMatrix,
		rightDistortionCoefficients,
		rightRotationMatrix,
		rightTranslationMatrix,
		CV_CALIB_RATIONAL_MODEL,
		cv::TermCriteria(CV_TERMCRIT_ITER+CV_TERMCRIT_EPS, 1000, 1e-5)
		);
	PRINT_TO_LOG("Calibration complete", "");

	PRINT_TO_LOG("Left calibration: ", leftCameraMatrix);
	PRINT_TO_LOG("Right calibration: ", rightCameraMatrix);
	PRINT_TO_LOG("Left distortion: ", leftDistortionCoefficients);
	PRINT_TO_LOG("Right distortion: ", rightDistortionCoefficients);
	PRINT_TO_LOG("Translation: ", leftTranslationMatrix);
	PRINT_TO_LOG("Rotation: ", leftRotationMatrix);
	}

void StereoReconstructionTestInterface::VisualizeCorrespondences(CorrespondenceMap2DConstPtr correspondenceMap)
	{
	std::vector<cv::KeyPoint> sourceVector, sinkVector;
	std::vector<cv::DMatch> matchesVector;
	for(int correspondenceIndex = 0; correspondenceIndex < GetNumberOfCorrespondences(*correspondenceMap); correspondenceIndex++)
		{
		BaseTypesWrapper::Point2D sourcePoint, sinkPoint;
		sourcePoint = GetSource(*correspondenceMap, correspondenceIndex);
		sinkPoint = GetSink(*correspondenceMap, correspondenceIndex);
		cv::KeyPoint sourceKeypoint(sourcePoint.x, sourcePoint.y, 0.02);
		cv::KeyPoint sinkKeypoint(sinkPoint.x, sinkPoint.y, 0.02);
		sourceVector.push_back(sourceKeypoint);
		sinkVector.push_back(sinkKeypoint);
		cv::DMatch match;
		match.queryIdx = correspondenceIndex;
		match.trainIdx = correspondenceIndex;
		matchesVector.push_back(match);
		}
	
	FrameToMatConverter converter;
	cv::Mat leftUndistortedCvImage = converter.Convert(undistortedLeftImage);
	cv::Mat rightUndistortedCvImage = converter.Convert(undistortedRightImage);

 	cv::Mat outputImage;
 	cv::drawMatches
		(
		leftUndistortedCvImage, sourceVector, rightUndistortedCvImage, sinkVector, matchesVector, 
		outputImage, 
		cv::Scalar::all(-1), cv::Scalar::all(-1),
               	std::vector<char>(), 
		cv::DrawMatchesFlags::NOT_DRAW_SINGLE_POINTS
		);

	cv::namedWindow(outputWindowName, CV_WINDOW_NORMAL);
	cv::imshow(outputWindowName, outputImage);
	cv::resizeWindow(outputWindowName, leftCvImage.cols*2, leftCvImage.rows);
	cv::waitKey(500);
	}

void StereoReconstructionTestInterface::VisualizeFeatures(VisualPointFeatureVector2DConstPtr leftFeaturesVector, VisualPointFeatureVector2DConstPtr rightFeaturesVector)
	{
	FrameToMatConverter converter;
	cv::Mat leftUndistortedCvImage = converter.Convert(undistortedLeftImage);
	cv::Mat rightUndistortedCvImage = converter.Convert(undistortedRightImage);

	for(int index = 0; index < GetNumberOfPoints(*leftFeaturesVector); index++)
		{
		cv::circle(leftUndistortedCvImage, cv::Point( GetXCoordinate(*leftFeaturesVector, index), GetYCoordinate(*leftFeaturesVector, index) ), 2, cv::Scalar(0, 0, 200), 2);
		}
	for(int index = 0; index < GetNumberOfPoints(*rightFeaturesVector); index++)
		{
		cv::circle(rightUndistortedCvImage, cv::Point( GetXCoordinate(*rightFeaturesVector, index), GetYCoordinate(*rightFeaturesVector, index) ), 2, cv::Scalar(0, 0, 200), 2);
		}

	cv::Mat outputImage(leftUndistortedCvImage.rows, leftUndistortedCvImage.cols*2, rightUndistortedCvImage.type());
	leftUndistortedCvImage.copyTo( outputImage( cv::Rect(0,0,leftUndistortedCvImage.cols, leftUndistortedCvImage.rows) ));
	rightUndistortedCvImage.copyTo( outputImage( cv::Rect(leftUndistortedCvImage.cols,0,leftUndistortedCvImage.cols, leftUndistortedCvImage.rows) ));

	cv::imshow("Features", outputImage);
	cv::waitKey(500);
	}


int main(int argc, char** argv)
	{
	StereoReconstructionTestInterface interface("Stereo Reconstruction", 100, 40);
	interface.Run();
	};

/** @} */
