/**
 * @author Alessandro Bianco
 */

/**
 * Test application for the DFN IssDetector3D
 */

/**
 * @addtogroup DFNsTest
 * @{
 */

#include <FeaturesExtraction3D/IssDetector3D.hpp>
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

class IssDetector3DTestInterface : public DFNTestInterface
{
	public:

		IssDetector3DTestInterface(const std::string& dfnName, int buttonWidth, int buttonHeight, std::string inputFile = DEFAULT_INPUT_FILE);
		~IssDetector3DTestInterface();

	private:

		IssDetector3D* iss;

		pcl::PointCloud<pcl::PointXYZ>::Ptr pclCloud;
		PointCloudConstPtr inputCloud;

		std::string outputWindowName;

		void SetupParameters();
		void DisplayResult();
};

IssDetector3DTestInterface::IssDetector3DTestInterface(const std::string& dfnName, int buttonWidth, int buttonHeight, std::string inputFile)
	: DFNTestInterface(dfnName, buttonWidth, buttonHeight)
{
	iss = new IssDetector3D;
	SetDFN(iss);

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
	iss->pointcloudInput(*inputCloud);

	outputWindowName = "Iss Detector 3D Result";
}

IssDetector3DTestInterface::~IssDetector3DTestInterface()
{
	delete iss;
	delete inputCloud;
}

void IssDetector3DTestInterface::SetupParameters()
{
	AddParameter("GeneralParameters", "SalientRadius", 0.0001, 1, 0.0001);
	AddParameter("GeneralParameters", "NonMaximaSupressionRadius", 0.0001, 1, 0.0001);
	AddParameter("GeneralParameters", "NormalRadius", 0.0000, 1, 0.0001);

	AddParameter("GeneralParameters", "FirstThreshold", 0.975, 1, 0.001);
	AddParameter("GeneralParameters", "SecondThreshold", 0.975, 1, 0.001);
	AddParameter("GeneralParameters", "MinNumberOfNeighbours", 5, 100);
	AddParameter("GeneralParameters", "AngleThreshold", 1.74, 2, 0.01);
	AddParameter("GeneralParameters", "NumberOfThreads", 0, 8);
}

void IssDetector3DTestInterface::DisplayResult()
{
	const VisualPointFeatureVector3D& features = iss->featuresOutput();

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

	IssDetector3DTestInterface interface("IssDetector3D", 100, 40, inputFile);
	interface.Run();
};

/** @} */
