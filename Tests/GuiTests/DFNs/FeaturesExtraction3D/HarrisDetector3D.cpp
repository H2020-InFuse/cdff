/**
 * @author Alessandro Bianco
 */

/**
 * Test application for the DFN HarrisDetector3D
 */

/**
 * @addtogroup DFNsTest
 * @{
 */

#include <FeaturesExtraction3D/HarrisDetector3D.hpp>
#include <Converters/PclPointCloudToPointCloudConverter.hpp>
#include <Errors/Assert.hpp>
#include <GuiTests/ParametersInterface.hpp>
#include <GuiTests/MainInterface.hpp>
#include <GuiTests/DFNs/DFNTestInterface.hpp>
#include <pcl/io/ply_io.h>
#include <pcl/visualization/pcl_visualizer.h>
#include <Visualizers/PCLVisualizer.hpp>

using namespace CDFF::DFN::FeaturesExtraction3D;
using namespace Converters;
using namespace VisualPointFeatureVector3DWrapper;
using namespace PointCloudWrapper;

const std::string USAGE =" \n as optional parameter, provide a path to a ply file \n";
const std::string DEFAULT_INPUT_FILE = "../../tests/Data/PointClouds/bunny0.ply";

class HarrisDetector3DTestInterface : public DFNTestInterface
{
	public:

		HarrisDetector3DTestInterface(const std::string& dfnName, int buttonWidth, int buttonHeight, std::string inputFile = DEFAULT_INPUT_FILE);
		~HarrisDetector3DTestInterface();

	private:

		HarrisDetector3D harris;

		pcl::PointCloud<pcl::PointXYZ>::Ptr pclCloud;
		PointCloudConstPtr inputCloud;

		std::string outputWindowName;

		void SetupParameters() override;
		void DisplayResult() override;
};

HarrisDetector3DTestInterface::HarrisDetector3DTestInterface(const std::string& dfnName, int buttonWidth, int buttonHeight, std::string inputFile) :
	DFNTestInterface(dfnName, buttonWidth, buttonHeight),
	harris(harris)
{
	SetDFN(&harris);

	pcl::PointCloud<pcl::PointXYZ>::Ptr basePclCloud = boost::make_shared<pcl::PointCloud<pcl::PointXYZ> >();
	pclCloud = boost::make_shared<pcl::PointCloud<pcl::PointXYZ> >();
	pcl::io::loadPLYFile(inputFile, *basePclCloud);
	for (unsigned pointIndex = 0; pointIndex < basePclCloud->points.size(); pointIndex++)
	{
		pcl::PointXYZ point = basePclCloud->points.at(pointIndex);
		if (point.x == point.x && point.y == point.y && point.z == point.z)
		{
			pclCloud->points.push_back(point);
		}
	}
	inputCloud = PclPointCloudToPointCloudConverter().Convert(pclCloud);
	harris.pointcloudInput(*inputCloud);

	outputWindowName = "Harris Detector 3D Result";
}

HarrisDetector3DTestInterface::~HarrisDetector3DTestInterface()
{
	delete inputCloud;
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
	const VisualPointFeatureVector3D& features = harris.featuresOutput();

	PRINT_TO_LOG("Processing time (seconds): ", GetLastProcessingTimeSeconds());
	PRINT_TO_LOG("Virtual memory used (kb): ", GetTotalVirtualMemoryUsedKB());
	PRINT_TO_LOG("Number of keypoints: ", GetNumberOfPoints(features));

	Visualizers::PclVisualizer::Enable();
	Visualizers::PclVisualizer::ShowVisualFeatures(inputCloud, &features);
}

int main(int argc, char **argv)
{
	ASSERT(argc <= 2, USAGE);
	std::string inputFile = (argc == 1) ? DEFAULT_INPUT_FILE : argv[1];

	HarrisDetector3DTestInterface* interface = new HarrisDetector3DTestInterface("HarrisDetector3D", 100, 40, inputFile);
	interface->Run();
	delete(interface);
};

/** @} */
