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
	};

StereoReconstructionTestInterface::StereoReconstructionTestInterface(std::string integrationName, int buttonWidth, int buttonHeight)
	: DFNsIntegrationTestInterface(buttonWidth, buttonHeight)
	{
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
	unsigned extractCols = singleImageCols/2;
	unsigned extractRows = singleImageRows/2;
	unsigned startRow = singleImageRows/4;
	unsigned startColumnLeft = singleImageCols/4;
	unsigned startColumnRight = singleImageCols/4 + singleImageCols;
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
	AddParameter(ransac, "GeneralParameters", "FocalLength", 1, 5, 0.1);
	AddParameter(ransac, "GeneralParameters", "PrinciplePointX", 0, 10);
	AddParameter(ransac, "GeneralParameters", "PrinciplePointY", 0, 10);
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


int main(int argc, char** argv)
	{
	StereoReconstructionTestInterface interface("Stereo Reconstruction", 100, 40);
	interface.Run();
	};

/** @} */
