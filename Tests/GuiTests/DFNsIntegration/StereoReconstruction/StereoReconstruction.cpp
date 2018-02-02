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
		unsigned PrepareNextDfn();

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

	leftCvImage = cv::imread("../../tests/Data/Images/AlgeriaDesert.jpg", cv::IMREAD_COLOR);
	rightCvImage = cv::imread("../../tests/Data/Images/AlgeriaDesert.jpg", cv::IMREAD_COLOR);

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

	}

void StereoReconstructionTestInterface::SetupParameters()
	{

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
	return (state == END);
	}

unsigned StereoReconstructionTestInterface::PrepareNextDfn()
	{
	if (state == START)
		{
		PrepareOrbLeft();
		return GetDfnIndex(orb);
		}
	else if (state == ORB_LEFT_IMAGE_DONE)
		{
		PrepareOrbRight();
		return GetDfnIndex(orb);
		}
	else if (state == ORB_RIGHT_IMAGE_DONE)
		{
		PrepareFlann();
		return GetDfnIndex(flann);
		}
	else if (state == MATCHING_DONE)
		{
		PrepareRansac();
		return GetDfnIndex(ransac);
		}
	else if (state == TRANSFORM_DONE)
		{
		PrepareTriangulation();
		return GetDfnIndex(triangulation);
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

	MatToFrameConverter converter;
	rightImage = converter.Convert(rightCvImage);
	orb->imageInput(rightImage);
	}

void StereoReconstructionTestInterface::PrepareFlann()
	{
	rightFeaturesVector = orb->featuresSetOutput();

	flann->sourceFeaturesVectorInput(leftFeaturesVector);
	flann->sinkFeaturesVectorInput(rightFeaturesVector);
	}

void StereoReconstructionTestInterface::PrepareRansac()
	{
	correspondenceMap = flann->correspondenceMapOutput();
	ransac->correspondenceMapInput(correspondenceMap);
	}

void StereoReconstructionTestInterface::PrepareTriangulation()
	{
	transform = ransac->transformOutput();
	}


int main(int argc, char** argv)
	{
	StereoReconstructionTestInterface interface("Stereo Reconstruction", 100, 40);
	interface.Run();
	};

/** @} */
