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
 * This test uses: DFN Orb2D, DFN Flann Matcher, DFN Essential Matrix and DFN Triangulation
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

#include <PointCloudReconstruction2DTo3D/Triangulation.hpp>
#include <FeaturesMatching2D/FlannMatcher.hpp>
#include <FeaturesExtraction2D/OrbDetectorDescriptor.hpp>
#include <CamerasTransformEstimation/EssentialMatrixRansac.hpp>

#include <FrameToMatConverter.hpp>
#include <MatToFrameConverter.hpp>
#include <PointCloudToPclPointCloudConverter.hpp>
#include <PclPointCloudToPointCloudConverter.hpp>

#include <Stubs/Common/ConversionCache/CacheHandler.hpp>
#include <Mocks/Common/Converters/FrameToMatConverter.hpp>
#include <Mocks/Common/Converters/MatToVisualPointFeatureVector2DConverter.hpp>
#include <Mocks/Common/Converters/VisualPointFeatureVector2DToMatConverter.hpp>
#include <Mocks/Common/Converters/PointCloudToPclPointCloudConverter.hpp>



using namespace dfn_ci;
using namespace Converters;
using namespace Common;
using namespace FrameWrapper;
using namespace VisualPointFeatureVector2DWrapper;
using namespace CorrespondenceMap2DWrapper;
using namespace PoseWrapper;
using namespace PointCloudWrapper;

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
			ORB_LEFT_IMAGE_DONE,
			ORB_RIGHT_IMAGE_DONE,
			MATCHING_DONE,
			TRANSFORM_DONE,
			END
			};
		State state;

		Stubs::CacheHandler<FrameConstPtr, cv::Mat>* stubFrameCache;
		Mocks::FrameToMatConverter* mockFrameConverter;
		Stubs::CacheHandler<cv::Mat, VisualPointFeatureVector2DConstPtr>* stubMatToVectorCache;
		Mocks::MatToVisualPointFeatureVector2DConverter* mockMatToVectorConverter;
		Stubs::CacheHandler<VisualPointFeatureVector2DConstPtr, cv::Mat>* stubVectorToMatCache;
		Mocks::VisualPointFeatureVector2DToMatConverter* mockVectorToMatConverter;

		OrbDetectorDescriptor* orb;
		FlannMatcher* flann;	
		EssentialMatrixRansac* ransac;	
		Triangulation* triangulation;

		std::string outputWindowName;
		cv::Mat leftCvImage;
		cv::Mat rightCvImage;

		FrameConstPtr leftImage;
		FrameConstPtr rightImage;
		VisualPointFeatureVector2DConstPtr leftFeaturesVector;
		VisualPointFeatureVector2DConstPtr rightFeaturesVector;
		CorrespondenceMap2DConstPtr correspondenceMap;
		Transform3DConstPtr transform;

		void SetupMocksAndStubs();
		void SetupParameters();
		void DisplayResult();

		void ResetProcess();
		bool IsProcessCompleted();
		void UpdateState();
		DFNCommonInterface* PrepareNextDfn();

		void PrepareOrbLeft();
		void PrepareOrbRight();
		void PrepareFlann();
		void PrepareRansac();
		void PrepareTriangulation();

		void ExtractCalibrationParameters();
		void ExtractCalibrationParametersOneCamera();
		void VisualizeCorrespondences(CorrespondenceMap2DConstPtr correspondenceMap);
	};

StereoReconstructionTestInterface::StereoReconstructionTestInterface(std::string integrationName, int buttonWidth, int buttonHeight)
	: DFNsIntegrationTestInterface(buttonWidth, buttonHeight)
	{
	//ExtractCalibrationParametersOneCamera();
	ExtractCalibrationParameters();

	orb = new OrbDetectorDescriptor();
	AddDFN(orb, "orb");

	flann = new FlannMatcher();
	AddDFN(flann, "flann");
		
	ransac = new EssentialMatrixRansac();
	AddDFN(ransac, "ransac");
	
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
	delete(orb);
	delete(flann);
	delete(ransac);
	delete(triangulation);
	}

void StereoReconstructionTestInterface::SetupMocksAndStubs()
	{
	stubFrameCache = new Stubs::CacheHandler<FrameConstPtr, cv::Mat>();
	mockFrameConverter = new Mocks::FrameToMatConverter();
	ConversionCache<FrameConstPtr, cv::Mat, FrameToMatConverter>::Instance(stubFrameCache, mockFrameConverter);

	stubMatToVectorCache = new Stubs::CacheHandler<cv::Mat, VisualPointFeatureVector2DConstPtr>();
	mockMatToVectorConverter = new Mocks::MatToVisualPointFeatureVector2DConverter();
	ConversionCache<cv::Mat, VisualPointFeatureVector2DConstPtr, MatToVisualPointFeatureVector2DConverter>::Instance(stubMatToVectorCache, mockMatToVectorConverter);

	stubVectorToMatCache = new Stubs::CacheHandler<VisualPointFeatureVector2DConstPtr, cv::Mat>();
	mockVectorToMatConverter = new Mocks::VisualPointFeatureVector2DToMatConverter();
	ConversionCache<VisualPointFeatureVector2DConstPtr, cv::Mat, VisualPointFeatureVector2DToMatConverter>::Instance(stubVectorToMatCache, mockVectorToMatConverter);
	}

void StereoReconstructionTestInterface::SetupParameters()
	{
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
	AddParameter(flann, "LocalitySensitiveHashingParameters", "TableNumber", 6, 20);
	AddParameter(flann, "LocalitySensitiveHashingParameters", "KeySize", 12, 20);
	AddParameter(flann, "LocalitySensitiveHashingParameters", "MultiProbeLevel", 1, 20);

	AddParameter(ransac, "GeneralParameters", "OutlierThreshold", 1, 100);
	AddParameter(ransac, "GeneralParameters", "Confidence", 0.9, 1, 0.01);
	AddParameter(ransac, "FirstCameraMatrix", "FocalLengthX", 1408.899186439272, 1500, 0.0001);
	AddParameter(ransac, "FirstCameraMatrix", "FocalLengthY", 1403.116708010621, 1500, 0.0001);
	AddParameter(ransac, "FirstCameraMatrix", "PrinciplePointX", 1053.351342078365, 1500, 0.001);
	AddParameter(ransac, "FirstCameraMatrix", "PrinciplePointY", 588.8342842821718, 1500, 0.001);
	AddParameter(ransac, "SecondCameraMatrix", "FocalLengthX", 1415.631284126374, 1500, 0.0001);
	AddParameter(ransac, "SecondCameraMatrix", "FocalLengthY", 1408.026118461406, 1500, 0.0001);
	AddParameter(ransac, "SecondCameraMatrix", "PrinciplePointX", 1013.347852589407, 1500, 0.001);
	AddParameter(ransac, "SecondCameraMatrix", "PrinciplePointY", 592.5031927882591, 1500, 0.001);
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
	delete(leftFeaturesVector);
	delete(rightFeaturesVector);
	delete(correspondenceMap);
	delete(transform);
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
		state = TRANSFORM_DONE;
		}
	}

DFNCommonInterface* StereoReconstructionTestInterface::PrepareNextDfn()
	{
	if (state == START)
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
	else if (state == TRANSFORM_DONE)
		{
		PRINT_TO_LOG("Step Ransac processing time (seconds): ", GetLastProcessingTimeSeconds(3) );
		PrepareTriangulation();
		state = END;
		return triangulation;
		}
	ASSERT(false, "Unhandled State!");
	return 0;
	}

void StereoReconstructionTestInterface::PrepareOrbLeft()
	{
	MatToFrameConverter converter;
	leftImage = converter.Convert(leftCvImage);
	orb->imageInput(leftImage);
	}

void StereoReconstructionTestInterface::PrepareOrbRight()
	{
	leftFeaturesVector = orb->featuresSetOutput();
	PRINT_TO_LOG("Number of features points from left image: ", GetNumberOfPoints(*leftFeaturesVector));

	MatToFrameConverter converter;
	rightImage = converter.Convert(rightCvImage);
	orb->imageInput(rightImage);
	}

void StereoReconstructionTestInterface::PrepareFlann()
	{
	rightFeaturesVector = orb->featuresSetOutput();
	PRINT_TO_LOG("Number of features points from right image: ", GetNumberOfPoints(*rightFeaturesVector));

	flann->sourceFeaturesVectorInput(leftFeaturesVector);
	flann->sinkFeaturesVectorInput(rightFeaturesVector);
	}

void StereoReconstructionTestInterface::PrepareRansac()
	{
	correspondenceMap = flann->correspondenceMapOutput();
	PRINT_TO_LOG("Number of correspondences from flann matcher: ", GetNumberOfCorrespondences(*correspondenceMap));

	ransac->correspondenceMapInput(correspondenceMap);
	}

void StereoReconstructionTestInterface::PrepareTriangulation()
	{
	bool success = ransac->successOutput();
	ASSERT(success, "Essential Matrix Ransac failed: unable to find a valid transform");

	transform = ransac->transformOutput();
	std::stringstream stream;
	stream << "(" << GetXTranslation(*transform)<<", "<< GetYTranslation(*transform)<<", "<< GetZTranslation(*transform)<<") ";
	stream << "(" << GetXRotation(*transform)<<", "<< GetYOrientation(*transform)<<", "<< GetZRotation(*transform)<<", "<< GetWRotation(*transform) <<") ";
	std::string message = stream.str();
	PRINT_TO_LOG("Essential matrix Ransac found transform: ", message);

	triangulation->transformInput(transform);
	triangulation->correspondenceMapInput(correspondenceMap);
	}

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
	

 	cv::Mat outputImage;
 	cv::drawMatches
		(
		leftCvImage, sourceVector, rightCvImage, sinkVector, matchesVector, 
		outputImage, 
		cv::Scalar::all(-1), cv::Scalar::all(-1),
               	std::vector<char>(), 
		cv::DrawMatchesFlags::NOT_DRAW_SINGLE_POINTS
		);

	cv::namedWindow(outputWindowName, CV_WINDOW_NORMAL);
	cv::imshow(outputWindowName, outputImage);
	}


int main(int argc, char** argv)
	{
	StereoReconstructionTestInterface interface("Stereo Reconstruction", 100, 40);
	interface.Run();
	};

/** @} */
