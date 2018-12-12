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
 * This test uses: DFN ImageUndistortion, DFN Harris Detector, DFN OrbDescriptor, DFN Flann Matcher, DFN Fundamental Matrix, DFN Essential Matrix and DFN Triangulation
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

#include <Errors/Assert.hpp>
#include <GuiTests/ParametersInterface.hpp>
#include <GuiTests/MainInterface.hpp>
#include <GuiTests/DFNsIntegration/DFNsIntegrationTestInterface.hpp>

#include <ImageFiltering/ImageUndistortion.hpp>
#include <PointCloudReconstruction2DTo3D/Triangulation.hpp>
#include <FeaturesMatching2D/FlannMatcher.hpp>
#include <FeaturesExtraction2D/HarrisDetector2D.hpp>
#include <FeaturesDescription2D/OrbDescriptor.hpp>
#include <FundamentalMatrixComputation/FundamentalMatrixRansac.hpp>
#include <CamerasTransformEstimation/EssentialMatrixDecomposition.hpp>

#include <Converters/FrameToMatConverter.hpp>
#include <Converters/MatToFrameConverter.hpp>
#include <Converters/PointCloudToPclPointCloudConverter.hpp>
#include <Converters/PclPointCloudToPointCloudConverter.hpp>

using namespace CDFF::DFN;
using namespace CDFF::DFN::ImageFiltering;
using namespace CDFF::DFN::PointCloudReconstruction2DTo3D;
using namespace CDFF::DFN::FeaturesMatching2D;
using namespace CDFF::DFN::FeaturesExtraction2D;
using namespace CDFF::DFN::FeaturesDescription2D;
using namespace CDFF::DFN::FundamentalMatrixComputation;
using namespace CDFF::DFN::CamerasTransformEstimation;
using namespace Converters;
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
		StereoReconstructionTestInterface(const std::string& integrationName, int buttonWidth, int buttonHeight);
		~StereoReconstructionTestInterface();
	protected:

	private:
		enum State
			{
			START,
			UNDISTORT_LEFT_IMAGE_DONE,
			UNDISTORT_RIGHT_IMAGE_DONE,
			HARRIS_LEFT_IMAGE_DONE,
			HARRIS_RIGHT_IMAGE_DONE,
			ORB_LEFT_IMAGE_DONE,
			ORB_RIGHT_IMAGE_DONE,
			MATCHING_DONE,
			FUNDAMENTAL_DONE,
			POSE_DONE,
			END
			};
		State state;

		ImageUndistortion* leftUndistortion;
		ImageUndistortion* rightUndistortion;
		HarrisDetector2D* harris;
		OrbDescriptor* orb;
		FlannMatcher* flann;	
		FundamentalMatrixRansac* ransac;
		EssentialMatrixDecomposition* essential;	
		Triangulation* triangulation;

		std::string outputWindowName;
		cv::Mat leftCvImage;
		cv::Mat rightCvImage;

		FrameConstPtr leftImage;
		FrameConstPtr rightImage;
		FramePtr undistortedLeftImage;
		FramePtr undistortedRightImage;
		VisualPointFeatureVector2DPtr leftKeypointsVector;
		VisualPointFeatureVector2DPtr rightKeypointsVector;
		VisualPointFeatureVector2DPtr leftFeaturesVector;
		VisualPointFeatureVector2DPtr rightFeaturesVector;
		CorrespondenceMap2DPtr correspondenceMap;
		Matrix3dPtr fundamentalMatrix;
		Pose3DPtr secondCameraPose;

		void SetupMocksAndStubs();
		void SetupParameters();
		void DisplayResult();

		void ResetProcess();
		bool IsProcessCompleted();
		void UpdateState();
		DFNCommonInterface* PrepareNextDfn();

		void PrepareUndistortionLeft();
		void PrepareUndistortionRight();
		void PrepareHarrisLeft();
		void PrepareHarrisRight();
		void PrepareOrbLeft();
		void PrepareOrbRight();
		void PrepareFlann();
		void PrepareRansac();
		void PrepareEssential();
		void PrepareTriangulation();

		void VisualizeCorrespondences(CorrespondenceMap2DConstPtr correspondenceMap);
		void VisualizeFeatures(VisualPointFeatureVector2DConstPtr leftFeaturesVector, VisualPointFeatureVector2DConstPtr rightFeaturesVector);
	};

StereoReconstructionTestInterface::StereoReconstructionTestInterface(const std::string& integrationName, int buttonWidth, int buttonHeight)
	: DFNsIntegrationTestInterface(buttonWidth, buttonHeight)
	{
	leftUndistortion = new ImageUndistortion();
	AddDFN(leftUndistortion, "leftUndistortion");

	rightUndistortion = new ImageUndistortion();
	AddDFN(rightUndistortion, "rightUndistortion");

	harris = new HarrisDetector2D();
	AddDFN(harris, "harris");
	
	orb = new OrbDescriptor();
	AddDFN(orb, "orb");

	flann = new FlannMatcher();
	AddDFN(flann, "flann");
		
	ransac = new FundamentalMatrixRansac();
	AddDFN(ransac, "ransac");

	essential = new EssentialMatrixDecomposition();
	AddDFN(essential, "essential");
	
	triangulation = new Triangulation();
	AddDFN(triangulation, "triangulation");

	cv::Mat doubleImage = cv::imread("../../tests/Data/Images/stuff.jpg", cv::IMREAD_COLOR);
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
	delete(harris);
	delete(orb);
	delete(flann);
	delete(ransac);
	delete(essential);
	delete(triangulation);
	}

void StereoReconstructionTestInterface::SetupMocksAndStubs()
	{

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

	AddParameter(harris, "GeneralParameters", "ApertureSize", 5, 7);
	AddParameter(harris, "GeneralParameters", "BlockSize", 2, 50);
	AddParameter(harris, "GeneralParameters", "ParameterK", 0.04, 1.00, 0.01);
	AddParameter(harris, "GeneralParameters", "DetectionThreshold", 200, 255);
	AddParameter(harris, "GeneralParameters", "UseGaussianBlur", 1, 1);
	AddParameter(harris, "GaussianBlur", "KernelWidth", 3, 99);
	AddParameter(harris, "GaussianBlur", "KernelHeight", 3, 99);
	AddParameter(harris, "GaussianBlur", "WidthStandardDeviation", 0.00, 1.00, 0.01);
	AddParameter(harris, "GaussianBlur", "HeightStandardDeviation", 0.00, 1.00, 0.01);

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
	PointCloudWrapper::PointCloudPtr pointCloud = NewPointCloud();
	Copy(triangulation->pointcloudOutput(), *pointCloud);

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
	delete(leftKeypointsVector);
	delete(rightKeypointsVector);
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
		state = HARRIS_LEFT_IMAGE_DONE;
		}
	else if (state == HARRIS_LEFT_IMAGE_DONE)
		{
		state = HARRIS_RIGHT_IMAGE_DONE;
		}
	else if (state == HARRIS_RIGHT_IMAGE_DONE)
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
		PRINT_TO_LOG("Step Undistort Left processing time (seconds): ", GetLastProcessingTimeSeconds(0) );
		PrepareUndistortionRight();
		return rightUndistortion;
		}
	else if (state == UNDISTORT_RIGHT_IMAGE_DONE)
		{
		PRINT_TO_LOG("Step Undistort Right processing time (seconds): ", GetLastProcessingTimeSeconds(0) );
		PrepareHarrisLeft();
		return harris;
		}
	else if (state == HARRIS_LEFT_IMAGE_DONE)
		{
		PRINT_TO_LOG("Step Harris Left processing time (seconds): ", GetLastProcessingTimeSeconds(0) );
		PrepareHarrisRight();
		return harris;
		}
	else if (state == HARRIS_RIGHT_IMAGE_DONE)
		{
		PRINT_TO_LOG("Step Harris Right processing time (seconds): ", GetLastProcessingTimeSeconds(0) );
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
	leftUndistortion->imageInput(*leftImage);
	}

void StereoReconstructionTestInterface::PrepareUndistortionRight()
	{
	MatToFrameConverter converter;
	rightImage = converter.Convert(rightCvImage);
	rightUndistortion->imageInput(*rightImage);
	}

void StereoReconstructionTestInterface::PrepareHarrisLeft()
	{
	undistortedLeftImage = NewFrame();
	Copy( leftUndistortion->imageOutput(), *undistortedLeftImage);

	harris->frameInput(*undistortedLeftImage);
	}

void StereoReconstructionTestInterface::PrepareHarrisRight()
	{
	undistortedRightImage = NewFrame();
	Copy( rightUndistortion->imageOutput(), *undistortedRightImage);
	leftKeypointsVector = NewVisualPointFeatureVector2D();
	Copy( harris->featuresOutput(), *leftKeypointsVector);
	PRINT_TO_LOG("Number of features points from left image: ", GetNumberOfPoints(*leftKeypointsVector));

	harris->frameInput(*undistortedRightImage);
	}

void StereoReconstructionTestInterface::PrepareOrbLeft()
	{
	rightKeypointsVector = NewVisualPointFeatureVector2D();
	Copy( harris->featuresOutput(), *rightKeypointsVector);
	PRINT_TO_LOG("Number of features points from left image: ", GetNumberOfPoints(*rightKeypointsVector));

	orb->frameInput(*undistortedLeftImage);
	orb->featuresInput(*leftKeypointsVector);
	}

void StereoReconstructionTestInterface::PrepareOrbRight()
	{
	leftFeaturesVector = NewVisualPointFeatureVector2D();
	Copy(orb->featuresOutput(), *leftFeaturesVector);

	orb->frameInput(*undistortedRightImage);
	orb->featuresInput(*rightKeypointsVector);
	}

void StereoReconstructionTestInterface::PrepareFlann()
	{
	rightFeaturesVector = NewVisualPointFeatureVector2D();
	Copy( orb->featuresOutput(), *rightFeaturesVector);
	VisualizeFeatures(leftFeaturesVector, rightFeaturesVector);

	flann->sourceFeaturesInput(*leftFeaturesVector);
	flann->sinkFeaturesInput(*rightFeaturesVector);
	}

void StereoReconstructionTestInterface::PrepareRansac()
	{
	correspondenceMap = NewCorrespondenceMap2D();
	Copy( flann->matchesOutput(), *correspondenceMap);
	VisualizeCorrespondences(correspondenceMap);
	PRINT_TO_LOG("Number of correspondences from flann matcher: ", GetNumberOfCorrespondences(*correspondenceMap));

	ransac->matchesInput(*correspondenceMap);
	}

void StereoReconstructionTestInterface::PrepareEssential()
	{
	bool success = ransac->successOutput();
	ASSERT(success, "Fundamental Matrix Ransac failed: unable to find a fundamental matrix");

	fundamentalMatrix = NewMatrix3d();
	Copy( ransac->fundamentalMatrixOutput(), *fundamentalMatrix);
	PRINT_TO_LOG("Fundamental matrix Ransac found transform: ", "");
	
	essential->fundamentalMatrixInput(*fundamentalMatrix);
	essential->matchesInput(*correspondenceMap);	
	}

void StereoReconstructionTestInterface::PrepareTriangulation()
	{
	secondCameraPose = NewPose3D();
	Copy( essential->transformOutput(), *secondCameraPose);
	bool success = essential->successOutput();
	ASSERT(success, "Essential Matrix Decomposition failed: unable to find a valid transform");

	std::stringstream transformStream;
	transformStream << "Position: (" << GetXPosition(*secondCameraPose) <<", "<<GetYPosition(*secondCameraPose) << ", "<<GetZPosition(*secondCameraPose) <<") ";
	transformStream << "Orientation: (" <<GetXRotation(*secondCameraPose)<<", "<<GetYRotation(*secondCameraPose)<<", "<<GetZRotation(*secondCameraPose) <<", "<<GetWRotation(*secondCameraPose)<<")";
	std::string transformString = transformStream.str();
	PRINT_TO_LOG("Computed Transform is:", transformString);

	triangulation->poseInput(*secondCameraPose);
	triangulation->matchesInput(*correspondenceMap);
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
