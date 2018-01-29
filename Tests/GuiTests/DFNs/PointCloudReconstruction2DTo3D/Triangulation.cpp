/* --------------------------------------------------------------------------
*
* (C) Copyright …
*
* ---------------------------------------------------------------------------
*/

/*!
 * @file Triangulation.cpp
 * @date 29/01/2018
 * @author Alessandro Bianco
 */

/*!
 * @addtogroup DFNsTest
 * 
 * Testing application for the DFN Triangulation.
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
#include <PointCloudReconstruction2DTo3D/Triangulation.hpp>
#include <Stubs/Common/ConversionCache/CacheHandler.hpp>
#include <ConversionCache/ConversionCache.hpp>
#include <PointCloudToPclPointCloudConverter.hpp>
#include <PclPointCloudToPointCloudConverter.hpp>
#include <MatToVisualPointFeatureVector3DConverter.hpp>
#include <Mocks/Common/Converters/PointCloudToPclPointCloudConverter.hpp>
#include <Mocks/Common/Converters/MatToVisualPointFeatureVector3DConverter.hpp>
#include <Errors/Assert.hpp>
#include <GuiTests/ParametersInterface.hpp>
#include <GuiTests/MainInterface.hpp>
#include <GuiTests/DFNs/DFNTestInterface.hpp>
#include<pcl/io/ply_io.h>
#include <pcl/visualization/pcl_visualizer.h>


using namespace dfn_ci;
using namespace Converters;
using namespace Common;
using namespace PoseWrapper;
using namespace CorrespondenceMap2DWrapper;
using namespace PointCloudWrapper;

class TriangulationTestInterface : public DFNTestInterface
	{
	public:
		TriangulationTestInterface(std::string dfnName, int buttonWidth, int buttonHeight);
		~TriangulationTestInterface();
	protected:

	private:
		Triangulation* triangulation;

		std::string outputWindowName;

		void SetupMocksAndStubs();
		void SetupParameters();
		void DisplayResult();
	};

TriangulationTestInterface::TriangulationTestInterface(std::string dfnName, int buttonWidth, int buttonHeight)
	: DFNTestInterface(dfnName, buttonWidth, buttonHeight)
	{
	triangulation = new Triangulation();
	SetDFN(triangulation);

	CorrespondenceMap2DPtr correspondenceMap = new CorrespondenceMap2D;
	ClearCorrespondences(*correspondenceMap);
	for(unsigned index = 0; index < 15; index++)
		{
		double indexDouble = (double)index;
		BaseTypesWrapper::Point2D sink, source;
		source.x = indexDouble;
		source.y = indexDouble/2;
		sink.x = source.x + 1;
		sink.y = source.y + 0;
		AddCorrespondence(*correspondenceMap, source, sink, 1);
		}
	triangulation->correspondenceMapInput(correspondenceMap);

	Transform3DPtr transform = new Transform3D();
	SetTranslation(*transform, 1, 0, 0);
	SetRotation(*transform, 0, 0, 0, 1);
	triangulation->transformInput(transform);

	outputWindowName = "Triangulation 2D to 3D Result";
	}

TriangulationTestInterface::~TriangulationTestInterface()
	{
	delete(triangulation);
	}

void TriangulationTestInterface::SetupMocksAndStubs()
	{

	}

void TriangulationTestInterface::SetupParameters()
	{

	}

void TriangulationTestInterface::DisplayResult()
	{
	PointCloudWrapper::PointCloudConstPtr pointCloud= triangulation->pointCloudOutput();

	PRINT_TO_LOG("The processing took (seconds): ", GetLastProcessingTimeSeconds() );
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

	delete(pointCloud);
	}


int main(int argc, char** argv)
	{
	TriangulationTestInterface interface("Triangulation 2D to 3D", 100, 40);
	interface.Run();
	};

/** @} */
