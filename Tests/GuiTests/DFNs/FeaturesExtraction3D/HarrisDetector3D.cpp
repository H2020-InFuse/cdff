/*!
 * @file HarrisDetector3D.cpp
 * @date 01/12/2017
 * @author Alessandro Bianco
 */

/*!
 * @addtogroup DFNsTest
 *
 * Testing application for the DFN HarrisDetector3D.
 *
 *
 * @{
 */

#include <opencv2/highgui/highgui.hpp>
#include <opencv2/imgproc/imgproc.hpp>
#include <opencv2/core/core.hpp>
#include <FeaturesExtraction3D/HarrisDetector3D.hpp>
#include <PclPointCloudToPointCloudConverter.hpp>
#include <Errors/Assert.hpp>
#include <GuiTests/ParametersInterface.hpp>
#include <GuiTests/MainInterface.hpp>
#include <GuiTests/DFNs/DFNTestInterface.hpp>
#include <pcl/io/ply_io.h>
#include <pcl/visualization/pcl_visualizer.h>
#include <Visualizers/PclVisualizer.hpp>


using namespace dfn_ci;
using namespace Converters;
using namespace VisualPointFeatureVector3DWrapper;
using namespace PointCloudWrapper;

class HarrisDetector3DTestInterface : public DFNTestInterface
	{
	public:
		HarrisDetector3DTestInterface(std::string dfnName, int buttonWidth, int buttonHeight);
		~HarrisDetector3DTestInterface();
	protected:

	private:
		HarrisDetector3D* harris;

		pcl::PointCloud<pcl::PointXYZ>::Ptr pclCloud;
		PointCloudConstPtr inputCloud;
		std::string outputWindowName;

		void SetupParameters();
		void DisplayResult();
	};

HarrisDetector3DTestInterface::HarrisDetector3DTestInterface(std::string dfnName, int buttonWidth, int buttonHeight)
	: DFNTestInterface(dfnName, buttonWidth, buttonHeight)
	{
	harris = new HarrisDetector3D();
	SetDFN(harris);

	pcl::PointCloud<pcl::PointXYZ>::Ptr basePclCloud = boost::make_shared<pcl::PointCloud<pcl::PointXYZ> >();
	pclCloud = boost::make_shared<pcl::PointCloud<pcl::PointXYZ> >();
	pcl::io::loadPLYFile("../../tests/Data/PointClouds/bunny0.ply", *basePclCloud);
	for (unsigned pointIndex = 0; pointIndex < basePclCloud->points.size(); pointIndex++)
		{
		pcl::PointXYZ point = basePclCloud->points.at(pointIndex);
		if (point.x == point.x && point.y == point.y && point.z == point.z)
			{
			pclCloud->points.push_back(point);
			}
		}
	inputCloud = PclPointCloudToPointCloudConverter().Convert(pclCloud);

	harris->pointcloudInput(*inputCloud);
	outputWindowName = "Harris Detector 3D Result";
	}

HarrisDetector3DTestInterface::~HarrisDetector3DTestInterface()
	{
	delete(harris);
	delete(inputCloud);
	}

void HarrisDetector3DTestInterface::SetupParameters()
	{
	AddParameter("GeneralParameters", "NonMaxSuppression", 1, 1);
	AddParameter("GeneralParameters", "Radius", 0.010, 0.100, 0.001);
	AddParameter("GeneralParameters", "SearchRadius", 0.010, 0.100, 0.001);
	AddParameter("GeneralParameters", "EnableRefinement", 0, 1);
	AddParameter("GeneralParameters", "DetectionThreshold", 0.0010, 0.0100, 0.0001);
	AddParameter("GeneralParameters", "NumberOfThreads", 0, 10);
	AddParameter("GeneralParameters", "HarrisMethod", 0, 4);
	}

void HarrisDetector3DTestInterface::DisplayResult()
	{
	const VisualPointFeatureVector3D& featuresVector = harris->featuresOutput();

	PRINT_TO_LOG("The processing took (seconds): ", GetLastProcessingTimeSeconds() );
	PRINT_TO_LOG("Virtual Memory used (Kb): ", GetTotalVirtualMemoryUsedKB() );
	PRINT_TO_LOG("Number of feature points: ", GetNumberOfPoints(featuresVector) );

	Visualizers::PclVisualizer::Enable();
	Visualizers::PclVisualizer::ShowVisualFeatures(inputCloud, &featuresVector);
	}

int main(int argc, char** argv)
	{
	HarrisDetector3DTestInterface interface("HarrisDetector3D", 100, 40);
	interface.Run();
	};

/** @} */
