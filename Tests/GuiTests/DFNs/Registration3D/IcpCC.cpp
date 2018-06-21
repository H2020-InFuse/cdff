/* --------------------------------------------------------------------------
*
* (C) Copyright …
*
* ---------------------------------------------------------------------------
*/

/*!
 * @file IcpCC.cpp
 * @date 31/05/2018
 * @author Alessandro Bianco
 */

/*!
 * @addtogroup DFNsTest
 *
 * Testing application for the DFN IcpCC.
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
#include <Registration3D/IcpCC.hpp>
#include <PclPointCloudToPointCloudConverter.hpp>
#include <Errors/Assert.hpp>
#include <GuiTests/ParametersInterface.hpp>
#include <GuiTests/MainInterface.hpp>
#include <GuiTests/DFNs/DFNTestInterface.hpp>
#include <pcl/io/ply_io.h>
#include <pcl/visualization/pcl_visualizer.h>
#include <signal.h>
#include <execinfo.h>
#include <pcl/filters/voxel_grid.h>
#include <Visualizers/PclVisualizer.hpp>

using namespace dfn_ci;
using namespace Converters;
using namespace PoseWrapper;
using namespace PointCloudWrapper;

class IcpCCTestInterface : public DFNTestInterface
	{
	public:
		IcpCCTestInterface(std::string dfnName, int buttonWidth, int buttonHeight);
		~IcpCCTestInterface();

		static void SegmentationFaultHandler(int signal);

	private:
		IcpCC* icp;

		pcl::PointCloud<pcl::PointXYZ>::Ptr pclSourceCloud;
		pcl::PointCloud<pcl::PointXYZ>::Ptr pclSinkCloud;
		PointCloudConstPtr inputSourceCloud;
		PointCloudConstPtr inputSinkCloud;
		Transform3DPtr transformGuess;
		std::string outputWindowName;

		PclPointCloudToPointCloudConverter pclPointCloudToPointCloud;

		void LoadInputClouds();
		void PrepareInputs();

		void SetupParameters();
		void DisplayResult();

		void PrintInformation(Transform3DConstPtr transform);
		pcl::PointCloud<pcl::PointXYZ>::ConstPtr PrepareOutputCloud(Transform3DConstPtr transform);
		void VisualizeClouds(pcl::PointCloud<pcl::PointXYZ>::ConstPtr correspondenceCloud);
	};

IcpCCTestInterface::IcpCCTestInterface(std::string dfnName, int buttonWidth, int buttonHeight)
	: DFNTestInterface(dfnName, buttonWidth, buttonHeight)
	{
	icp = new IcpCC();
	SetDFN(icp);

	LoadInputClouds();
	PrepareInputs();

	icp->sourceCloudInput(*inputSourceCloud);
	icp->sinkCloudInput(*inputSinkCloud);
	icp->useGuessInput(false);
	icp->transformGuessInput(*transformGuess);

	outputWindowName = "ICP 3D Result";
	}

IcpCCTestInterface::~IcpCCTestInterface()
	{
	delete(icp);
	delete(inputSourceCloud);
	delete(inputSinkCloud);
	delete(transformGuess);
	}

void IcpCCTestInterface::SegmentationFaultHandler(int sig)
	{
	void *array[10];
	size_t size;

	// get void*'s for all entries on the stack
	size = backtrace(array, 10);

	// print out all the frames to stderr
	fprintf(stderr, "Error: signal %d:\n", sig);
	backtrace_symbols_fd(array, size, STDERR_FILENO);
	exit(1);
	}

void IcpCCTestInterface::LoadInputClouds()
	{
	pcl::PointCloud<pcl::PointXYZ>::Ptr pclInputCloud = boost::make_shared<pcl::PointCloud<pcl::PointXYZ> >();
	pclSourceCloud = boost::make_shared<pcl::PointCloud<pcl::PointXYZ> >();
	pclSinkCloud = boost::make_shared<pcl::PointCloud<pcl::PointXYZ> >();
	pcl::io::loadPLYFile("../../tests/Data/PointClouds/bunny0.ply", *pclInputCloud);

	unsigned selectionCounter = 0;
	unsigned selectionRatio = 10;
	for(unsigned pointIndex = 0; pointIndex < pclInputCloud->size(); pointIndex++)
		{
		pcl::PointXYZ tempPoint = pclInputCloud->points.at(pointIndex);
		bool validPoint = tempPoint.x == tempPoint.x && tempPoint.y == tempPoint.y && tempPoint.z == tempPoint.z;
		if (validPoint && selectionCounter == 0)
			{
			pclSinkCloud->points.push_back(tempPoint);
			}
		if (validPoint)
			{
			selectionCounter = (selectionCounter+1)%selectionRatio;
			}
		}
	for(unsigned pointIndex = 2*pclSinkCloud->points.size()/3; pointIndex < 2*pclSinkCloud->points.size()/3 + pclSinkCloud->points.size()/10; pointIndex++)
		{
		pcl::PointXYZ point = pclSinkCloud->points.at(pointIndex);
		point.x = point.x + 0.2;
		point.y = point.y + 0.1;
		pclSourceCloud->points.push_back( point );
		}

	pcl::VoxelGrid<pcl::PointXYZ> grid;
	const float LEAFT_SIZE = 0.005f;
	grid.setLeafSize (LEAFT_SIZE, LEAFT_SIZE, LEAFT_SIZE);
	grid.setInputCloud (pclSinkCloud);
	grid.filter (*pclSinkCloud);
	grid.setInputCloud (pclSourceCloud);
	grid.filter (*pclSourceCloud);

	transformGuess = NewPose3D();
	SetPosition(*transformGuess, 0.19, 0.09, 0.01);
	SetOrientation(*transformGuess, 0, 0, 0, 1);
	}

void IcpCCTestInterface::PrepareInputs()
	{
	inputSourceCloud = pclPointCloudToPointCloud.Convert(pclSourceCloud);
	inputSinkCloud = pclPointCloudToPointCloud.Convert(pclSinkCloud);
	}

void IcpCCTestInterface::SetupParameters()
	{
	AddParameter("GeneralParameters", "ConvergenceType", 0, 2);
	AddParameter("GeneralParameters", "MinimumErrorReduction", 1e-5, 1e-2, 1e-8);
	AddParameter("GeneralParameters", "MaximumNumberOfIterations", 20, 50000);
	AddParameter("GeneralParameters", "ScaleIsAdjustable", 0, 1);
	AddParameter("GeneralParameters", "FarthestPointsAreFilteredOut", 0, 1);
	AddParameter("GeneralParameters", "SamplingLimit", 50000, 100000);
	AddParameter("GeneralParameters", "FinalOverlapRatio", 1.0, 1.0, 1e-3);
	AddParameter("GeneralParameters", "MaximumNumberOfThreads", 0, 10);
	}

void IcpCCTestInterface::DisplayResult()
	{
	bool success = icp->successOutput();
	if (!success)
		{
		PRINT_TO_LOG("Icp was not successful", "");
		return;
		}

	const Transform3D& outputTransform = icp->transformOutput();
	Transform3DPtr transform = NewPose3D();
	Copy(outputTransform, *transform);

	PrintInformation(transform);
	pcl::PointCloud<pcl::PointXYZ>::ConstPtr correspondenceCloud = PrepareOutputCloud(transform);

	VisualizeClouds(correspondenceCloud);

	delete(transform);
	}

void IcpCCTestInterface::PrintInformation(Transform3DConstPtr transform)
	{
	PRINT_TO_LOG("The processing took (seconds): ", GetLastProcessingTimeSeconds() );
	PRINT_TO_LOG("Virtual Memory used (Kb): ", GetTotalVirtualMemoryUsedKB() );
	PRINT_TO_LOG("Sink Cloud Size: ", GetNumberOfPoints(*inputSinkCloud) );
	PRINT_TO_LOG("Source Cloud Size: ", GetNumberOfPoints(*inputSourceCloud) );
	std::stringstream positionStream, orientationStream;
	positionStream << "(" << GetXPosition(*transform) <<", "<<GetYPosition(*transform)<<", "<<GetZPosition(*transform)<<")";
	orientationStream << "(" << GetXOrientation(*transform) <<", "<<GetYOrientation(*transform)<<", "<<GetZOrientation(*transform)<<", "<<GetWOrientation(*transform)<<")";
	PRINT_TO_LOG("Transform Position: ", positionStream.str() );
	PRINT_TO_LOG("Transform Orientation: ", orientationStream.str() );
	}

pcl::PointCloud<pcl::PointXYZ>::ConstPtr IcpCCTestInterface::PrepareOutputCloud(Transform3DConstPtr transform)
	{
	Eigen::Quaternionf eigenRotation( GetWOrientation(*transform), GetXOrientation(*transform), GetYOrientation(*transform), GetZOrientation(*transform));
	Eigen::Translation<float, 3> eigenTranslation( GetXPosition(*transform), GetYPosition(*transform), GetZPosition(*transform));

	pcl::PointCloud<pcl::PointXYZ>::Ptr correspondenceCloud = boost::make_shared<pcl::PointCloud<pcl::PointXYZ> >();
	for(unsigned pointIndex = 0; pointIndex < pclSourceCloud->points.size(); pointIndex++)
		{
		pcl::PointXYZ sourcePoint = pclSourceCloud->points.at(pointIndex);
		Eigen::Vector3f eigenPoint(sourcePoint.x, sourcePoint.y, sourcePoint.z);
		Eigen::Vector3f transformedPoint = eigenTranslation * eigenRotation * eigenPoint;
		pcl::PointXYZ transformedPclPoint;
		transformedPclPoint.x = transformedPoint.x();
		transformedPclPoint.y = transformedPoint.y();
		transformedPclPoint.z = transformedPoint.z();
		correspondenceCloud->points.push_back(transformedPclPoint);
		}

	return correspondenceCloud;
	}

void IcpCCTestInterface::VisualizeClouds(pcl::PointCloud<pcl::PointXYZ>::ConstPtr correspondenceCloud)
	{
	Visualizers::PclVisualizer::Enable();
	std::vector< pcl::PointCloud<pcl::PointXYZ>::ConstPtr > pointCloudsList = {pclSourceCloud, pclSinkCloud, correspondenceCloud};
	Visualizers::PclVisualizer::ShowPointClouds(pointCloudsList);
	}

int main(int argc, char** argv)
	{
	signal(SIGSEGV, IcpCCTestInterface::SegmentationFaultHandler);
	IcpCCTestInterface interface("IcpCC", 100, 40);
	interface.Run();
	}

/** @} */
