/**
 * @author Alessandro Bianco
 */

/**
 * Test application for the DFN CornerDetector3D
 */

/**
 * @addtogroup DFNsTest
 * @{
 */

#include <FeaturesExtraction3D/CornerDetector3D.hpp>
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

class CornerDetector3DTestInterface : public DFNTestInterface
{
	public:

		CornerDetector3DTestInterface(const std::string& dfnName, int buttonWidth, int buttonHeight, std::string inputFile = DEFAULT_INPUT_FILE);
		~CornerDetector3DTestInterface();

	private:

		CornerDetector3D* cornerDetector;

		pcl::PointCloud<pcl::PointXYZ>::Ptr pclCloud;
		PointCloudConstPtr inputCloud;

		std::string outputWindowName;

		void SetupParameters();
		void DisplayResult();
};

CornerDetector3DTestInterface::CornerDetector3DTestInterface(const std::string& dfnName, int buttonWidth, int buttonHeight, std::string inputFile)
	: DFNTestInterface(dfnName, buttonWidth, buttonHeight)
{
	cornerDetector = new CornerDetector3D;
	SetDFN(cornerDetector);

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
	cornerDetector->pointcloudInput(*inputCloud);

	outputWindowName = "Harris Detector 3D Result";
}

CornerDetector3DTestInterface::~CornerDetector3DTestInterface()
{
	delete cornerDetector;
	delete inputCloud;
}

void CornerDetector3DTestInterface::SetupParameters()
{
	AddParameter("GeneralParameters", "NumberOfNeighboursNormalEstimation", 50, 200);
	AddParameter("GeneralParameters", "NumberOfNeighboursRegionGrowing", 40, 200);
	AddParameter("GeneralParameters", "NumberOfNeightbourBorderSelection", 15, 200);
	AddParameter("GeneralParameters", "SmoothnessThreshold", 0.080, 7, 0.001);
	AddParameter("GeneralParameters", "CurvatureThreshold", 0.300, 7, 0.001);
	AddParameter("GeneralParameters", "MinimumClusterSize", 1, 1000001);
	AddParameter("GeneralParameters", "MaximumClusterSize", 1000000, 1000001);
	AddParameter("GeneralParameters", "SearchRadiusBorderSelection", 0.015, 2, 0.001);
	AddParameter("GeneralParameters", "NumberOfHistogramSlots", 10, 100);
	AddParameter("GeneralParameters", "MaximumNumberOfEmptySlots", 3, 100);
}

void CornerDetector3DTestInterface::DisplayResult()
{
	const VisualPointFeatureVector3D& features = cornerDetector->featuresOutput();

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

	CornerDetector3DTestInterface interface("CornerDetector3D", 100, 40, inputFile);
	interface.Run();
};

/** @} */
