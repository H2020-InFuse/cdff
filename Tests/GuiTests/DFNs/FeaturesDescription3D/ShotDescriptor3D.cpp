/**
 * @author Alessandro Bianco
 */

/**
 * Test application for the DFN ShotDescriptor3D
 */

/**
 * @addtogroup DFNsTest
 * @{
 */

#include <FeaturesDescription3D/ShotDescriptor3D.hpp>
#include <Converters/PclPointCloudToPointCloudConverter.hpp>
#include <Converters/PclNormalsCloudToPointCloudConverter.hpp>
#include <Errors/Assert.hpp>
#include <GuiTests/ParametersInterface.hpp>
#include <GuiTests/MainInterface.hpp>
#include <GuiTests/DFNs/DFNTestInterface.hpp>
#include <pcl/io/ply_io.h>
#include <pcl/visualization/pcl_visualizer.h>

using namespace CDFF::DFN::FeaturesDescription3D;
using namespace Converters;
using namespace VisualPointFeatureVector3DWrapper;
using namespace PointCloudWrapper;

class ShotDescriptor3DTestInterface : public DFNTestInterface
{
	public:

		ShotDescriptor3DTestInterface(const std::string& dfnName, int buttonWidth, int buttonHeight);
		~ShotDescriptor3DTestInterface();

	private:

		static const unsigned SHOT_DESCRIPTOR_SIZE;

		ShotDescriptor3D* shot;

		pcl::PointCloud<pcl::PointXYZ>::Ptr pclCloud;
		PointCloudConstPtr inputCloud;

		VisualPointFeatureVector3DConstPtr inputFeatures;

		pcl::PointCloud<pcl::Normal>::Ptr pclNormals;
		PointCloudConstPtr inputNormals;

		pcl::PointCloud<pcl::PointXYZ>::Ptr PreparePointCloudInput();

		std::string outputWindowName;

		void SetupParameters();
		void DisplayResult();

		pcl::PointCloud<pcl::PointXYZRGB>::Ptr PrepareOutputCloud(VisualPointFeatureVector3DConstPtr features);

		void GetComponentRange(VisualPointFeatureVector3DConstPtr features, int componentIndex, float& min, float& max);
};

const unsigned ShotDescriptor3DTestInterface::SHOT_DESCRIPTOR_SIZE = 352;

ShotDescriptor3DTestInterface::ShotDescriptor3DTestInterface(const std::string& dfnName, int buttonWidth, int buttonHeight)
	: DFNTestInterface(dfnName, buttonWidth, buttonHeight)
{
	shot = new ShotDescriptor3D;
	SetDFN(shot);

	pclCloud = PreparePointCloudInput();
	inputCloud = PclPointCloudToPointCloudConverter().Convert(pclCloud);
	shot->pointcloudInput(*inputCloud);

	VisualPointFeatureVector3DPtr features = NewVisualPointFeatureVector3D();
	ClearPoints(*features);
	for (unsigned index = 20; index < pclCloud->points.size(); index += 20)
	{
		AddPoint(*features, index);
	}
	inputFeatures = features;
	shot->featuresInput(*inputFeatures);

	pclNormals = boost::make_shared<pcl::PointCloud<pcl::Normal> >();
	inputNormals = PclNormalsCloudToPointCloudConverter().Convert(pclNormals);
	shot->normalsInput(*inputNormals);

	outputWindowName = "SHOT Descriptor 3D Result";
}

ShotDescriptor3DTestInterface::~ShotDescriptor3DTestInterface()
{
	delete shot;
	delete inputCloud;
	delete inputFeatures;
	delete inputNormals;
}

pcl::PointCloud<pcl::PointXYZ>::Ptr ShotDescriptor3DTestInterface::PreparePointCloudInput()
{
	pcl::PointCloud<pcl::PointXYZ>::Ptr baseCloud = boost::make_shared<pcl::PointCloud<pcl::PointXYZ> >();
	pcl::io::loadPLYFile("../../tests/Data/PointClouds/bunny0.ply", *baseCloud);

	pcl::PointCloud<pcl::PointXYZ>::Ptr outputCloud = boost::make_shared<pcl::PointCloud<pcl::PointXYZ> >();
	unsigned selectionCounter = 0;
	unsigned const SELECTION_RATIO = 5;
	for (unsigned pointIndex = 0; pointIndex < baseCloud->points.size(); pointIndex++)
	{
		pcl::PointXYZ point = baseCloud->points.at(pointIndex);
		if (point.x == point.x && point.y == point.y && point.z == point.z)
		{
			if (selectionCounter == 0)
			{
				outputCloud->points.push_back(point);
			}
			selectionCounter = (selectionCounter+1)%SELECTION_RATIO;
		}
	}

	return outputCloud;
}

void ShotDescriptor3DTestInterface::SetupParameters()
{
	AddParameter("GeneralParameters", "LocalReferenceFrameEstimationRadius", 0.10, 1.00, 0.01);
	AddParameter("GeneralParameters", "SearchRadius", 0.10, 1.00, 0.01);
	AddParameter("GeneralParameters", "OutputFormat", 0, 2);
	AddParameter("GeneralParameters", "EnableNormalsEstimation", 1, 1);
	AddParameter("GeneralParameters", "ForceNormalsEstimation", 1, 1);
	AddParameter("NormalEstimationParameters", "SearchRadius", 0.10, 1.00, 0.01);
	AddParameter("NormalEstimationParameters", "NeighboursSetSize", 0, 100);
}

void ShotDescriptor3DTestInterface::DisplayResult()
{
	const VisualPointFeatureVector3D& features = shot->featuresOutput();

	PRINT_TO_LOG("Processing time (seconds): ", GetLastProcessingTimeSeconds());
	PRINT_TO_LOG("Virtual memory used (Kb): ", GetTotalVirtualMemoryUsedKB());
	PRINT_TO_LOG("Number of keypoints: ", GetNumberOfPoints(features));

	pcl::PointCloud<pcl::PointXYZRGB>::Ptr featureCloud = PrepareOutputCloud(&features);

	pcl::visualization::PCLVisualizer viewer(outputWindowName);
	pcl::visualization::PointCloudColorHandlerCustom<pcl::PointXYZ> pclCloudColor(pclCloud, 255, 255, 255);
	pcl::visualization::PointCloudColorHandlerRGBField<pcl::PointXYZRGB> rgbField(featureCloud);
	viewer.addPointCloud(pclCloud, pclCloudColor, "input");
	viewer.addPointCloud<pcl::PointXYZRGB> (featureCloud, rgbField, "Keypoints");

	while (!viewer.wasStopped())
	{
		viewer.spinOnce();
		pcl_sleep(0.01);
	}

}

pcl::PointCloud<pcl::PointXYZRGB>::Ptr ShotDescriptor3DTestInterface::PrepareOutputCloud(VisualPointFeatureVector3DConstPtr features)
{
	const int featureIndexForRedColor = 2;
	const int featureIndexForGreenColor = 4;
	const int featureIndexForBlueColor = 5;

	float minR, maxR, minG, maxG, minB, maxB;
	GetComponentRange(features, featureIndexForRedColor, minR, maxR);
	GetComponentRange(features, featureIndexForGreenColor, minG, maxG);
	GetComponentRange(features, featureIndexForBlueColor, minB, maxB);

	pcl::PointCloud<pcl::PointXYZRGB>::Ptr featureCloud = boost::make_shared<pcl::PointCloud<pcl::PointXYZRGB> >();
	for (int pointIndex = 0; pointIndex < GetNumberOfPoints(*features); pointIndex++)
	{
		pcl::PointXYZRGB newPoint;
		newPoint.x = GetXCoordinate(*features, pointIndex);
		newPoint.y = GetYCoordinate(*features, pointIndex);
		newPoint.z = GetZCoordinate(*features, pointIndex);
		newPoint.r = 250-150*( (GetDescriptorComponent(*features, pointIndex, featureIndexForRedColor) - minR)/(maxR - minR) );
		newPoint.g = 50+150*( (GetDescriptorComponent(*features, pointIndex, featureIndexForGreenColor) - minG)/(maxG - minG) );
		newPoint.b = 50+150*( (GetDescriptorComponent(*features, pointIndex, featureIndexForBlueColor) - minB)/(maxB - minB) );
		featureCloud->points.push_back(newPoint);
	}

	return featureCloud;
}

void ShotDescriptor3DTestInterface::GetComponentRange(VisualPointFeatureVector3DConstPtr features, int componentIndex, float& min, float& max)
{
	if (GetNumberOfPoints(*features) == 0)
	{
		return;
	}

	min = GetDescriptorComponent(*features, 0, componentIndex);
	max = min;
	for (int featureIndex = 1; featureIndex < GetNumberOfPoints(*features); featureIndex++)
	{
		ASSERT(GetNumberOfDescriptorComponents(*features, featureIndex) == SHOT_DESCRIPTOR_SIZE,
			"SHOT descriptor size does not match size of received feature");
		if (max < GetDescriptorComponent(*features, featureIndex, componentIndex) )
		{
			max = GetDescriptorComponent(*features, featureIndex, componentIndex);
		}
		if (min > GetDescriptorComponent(*features, featureIndex, componentIndex) )
		{
			min = GetDescriptorComponent(*features, featureIndex, componentIndex);
		}
	}
	if (max == min)
	{
		max = min + 1;
	}
}

int main(int argc, char **argv)
{
	ShotDescriptor3DTestInterface interface("ShotDescriptor3D", 100, 40);
	interface.Run();
};

/** @} */
