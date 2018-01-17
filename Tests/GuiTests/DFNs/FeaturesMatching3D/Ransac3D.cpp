/* --------------------------------------------------------------------------
*
* (C) Copyright …
*
* ---------------------------------------------------------------------------
*/

/*!
 * @file Ransac3D.cpp
 * @date 17/01/2018
 * @author Alessandro Bianco
 */

/*!
 * @addtogroup DFNsTest
 * 
 * Testing application for the DFN Ransac3D.
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
#include <FeaturesMatching3D/Ransac3D.hpp>
#include <Stubs/Common/ConversionCache/CacheHandler.hpp>
#include <ConversionCache/ConversionCache.hpp>
#include <PointCloudToPclPointCloudConverter.hpp>
#include <PclPointCloudToPointCloudConverter.hpp>
#include <Mocks/Common/Converters/PointCloudToPclPointCloudConverter.hpp>
#include <Errors/Assert.hpp>
#include <GuiTests/ParametersInterface.hpp>
#include <GuiTests/MainInterface.hpp>
#include <GuiTests/DFNs/DFNTestInterface.hpp>
#include<pcl/io/ply_io.h>
#include <pcl/visualization/pcl_visualizer.h>
#include <signal.h>
#include <execinfo.h>
#include <pcl/sample_consensus/sac_model_registration.h>
#include <pcl/registration/sample_consensus_prerejective.h>
#include <pcl/search/search.h>
#include <pcl/sample_consensus/sac_model_plane.h>
#include <pcl/sample_consensus/sac_model_sphere.h>
#include <pcl/sample_consensus/ransac.h>

using namespace dfn_ci;
using namespace Converters;
using namespace Common;
using namespace CorrespondenceMap3DWrapper;
using namespace PointCloudWrapper;

class Ransac3DTestInterface : public DFNTestInterface
	{
	public:
		Ransac3DTestInterface(std::string dfnName, int buttonWidth, int buttonHeight);
		~Ransac3DTestInterface();
	protected:

	private:
		Stubs::CacheHandler<PointCloudConstPtr, pcl::PointCloud<pcl::PointXYZ>::ConstPtr >* stubInputCache;
		Mocks::PointCloudToPclPointCloudConverter* mockInputConverter;
		//Stubs::CacheHandler<cv::Mat, VisualPointFeatureVector3DConstPtr>* stubOutputCache;
		//Mocks::MatToVisualPointFeatureVector3DConverter* mockOutputConverter;
		Ransac3D* ransac;

		pcl::PointCloud<pcl::PointXYZ>::Ptr pclSourceCloud;
		pcl::PointCloud<pcl::PointXYZ>::Ptr pclSinkCloud;
		PointCloudConstPtr inputSourceCloud;
		PointCloudConstPtr inputSinkCloud;
		std::string outputWindowName;

		void SetupMocksAndStubs();
		void SetupParameters();
		void DisplayResult();
	};

Ransac3DTestInterface::Ransac3DTestInterface(std::string dfnName, int buttonWidth, int buttonHeight)
	: DFNTestInterface(dfnName, buttonWidth, buttonHeight)
	{
	ransac = new Ransac3D();
	SetDFN(ransac);

	PclPointCloudToPointCloudConverter converter;
	pclSourceCloud = boost::make_shared<pcl::PointCloud<pcl::PointXYZ> >();
	pclSinkCloud = boost::make_shared<pcl::PointCloud<pcl::PointXYZ> >();
	pcl::PointCloud<pcl::PointXYZ>::Ptr pclTempCloud = boost::make_shared<pcl::PointCloud<pcl::PointXYZ> >();
	pcl::io::loadPLYFile("../../tests/Data/PointClouds/bunny0.ply", *pclTempCloud);
	for(unsigned pointIndex = 0; pointIndex < pclTempCloud->size(); pointIndex++)
		{
		pcl::PointXYZ tempPoint = pclTempCloud->points.at(pointIndex);
		bool validPoint = tempPoint.x == tempPoint.x && tempPoint.y == tempPoint.y && tempPoint.z == tempPoint.z;
		if (validPoint)
			pclSinkCloud->points.push_back(tempPoint);
		}
	for(unsigned pointIndex = pclSinkCloud->size()/2; pointIndex < pclSinkCloud->size()/2 + pclSinkCloud->size()/10; pointIndex++)
		{
		pcl::PointXYZ copyPoint = pclSinkCloud->points.at(pointIndex);
		pcl::PointXYZ sourcePoint(copyPoint.x + 0.1, copyPoint.y + 0.05, copyPoint.z);
		pclSourceCloud->points.push_back(sourcePoint);
		}

	inputSourceCloud = converter.Convert(pclSourceCloud);
	inputSinkCloud = converter.Convert(pclSinkCloud);
	ransac->sourceCloudInput(inputSourceCloud);
	ransac->sinkCloudInput(inputSourceCloud);

	outputWindowName = "Ransac 3D Result";
	}

Ransac3DTestInterface::~Ransac3DTestInterface()
	{
	delete(stubInputCache);
	delete(mockInputConverter);
	//delete(stubOutputCache);
	//delete(mockOutputConverter);
	delete(ransac);
	delete(inputSinkCloud);
	delete(inputSourceCloud);
	}

void Ransac3DTestInterface::SetupMocksAndStubs()
	{
	stubInputCache = new Stubs::CacheHandler<PointCloudConstPtr, pcl::PointCloud<pcl::PointXYZ>::ConstPtr>();
	mockInputConverter = new Mocks::PointCloudToPclPointCloudConverter();
	ConversionCache<PointCloudConstPtr, pcl::PointCloud<pcl::PointXYZ>::ConstPtr, PointCloudToPclPointCloudConverter>::Instance(stubInputCache, mockInputConverter);

	//stubOutputCache = new Stubs::CacheHandler<cv::Mat, VisualPointFeatureVector3DConstPtr>();
	//mockOutputConverter = new Mocks::MatToVisualPointFeatureVector3DConverter();
	//ConversionCache<cv::Mat, VisualPointFeatureVector3DConstPtr, MatToVisualPointFeatureVector3DConverter>::Instance(stubOutputCache, mockOutputConverter);
	}

void Ransac3DTestInterface::SetupParameters()
	{
	AddParameter("GeneralParameters", "MaxIterationsNumber", 1000, 100000, 100);
	AddParameter("GeneralParameters", "OutliersFreeProbability", 0.99, 1.00, 0.01);
	AddParameter("GeneralParameters", "DistanceThreshold", 0.01, 1.00, 0.01);
	AddParameter("GeneralParameters", "SamplesMaxDistance", 0.00, 1.00, 0.01);
	}

void Ransac3DTestInterface::DisplayResult()
	{
	CorrespondenceMap3DConstPtr correspodenceMap= ransac->correspondenceMapOutput();

	PRINT_TO_LOG("The processing took (seconds): ", GetLastProcessingTimeSeconds() );
	PRINT_TO_LOG("Virtual Memory used (Kb): ", GetTotalVirtualMemoryUsedKB() );
	PRINT_TO_LOG("Number of correspondences: ", GetNumberOfCorrespondences(*correspodenceMap) );

	pcl::PointCloud<pcl::PointXYZ>::Ptr correspondenceCloud = boost::make_shared<pcl::PointCloud<pcl::PointXYZ> >();
	for(int correspodenceIndex = 0; correspodenceIndex < GetNumberOfCorrespondences(*correspodenceMap); correspodenceIndex++)
		{
		BaseTypesWrapper::Point3D sourcePoint = GetSource(*correspodenceMap, correspodenceIndex);
		BaseTypesWrapper::Point3D sinkPoint = GetSink(*correspodenceMap, correspodenceIndex);
		pcl::PointXYZ sourceCloudPoint(sourcePoint.x, sourcePoint.y, sourcePoint.z);
		pcl::PointXYZ sinkCloudPoint(sinkPoint.x, sinkPoint.y, sinkPoint.z);
		correspondenceCloud->points.push_back(sourceCloudPoint);
		correspondenceCloud->points.push_back(sinkCloudPoint);
		}

	pcl::visualization::PCLVisualizer viewer (outputWindowName);
    	pcl::visualization::PointCloudColorHandlerCustom<pcl::PointXYZ> pclSourceCloudColor(pclSourceCloud, 255, 255, 255);
    	pcl::visualization::PointCloudColorHandlerCustom<pcl::PointXYZ> pclSinkCloudColor(pclSinkCloud, 255, 255, 0);
    	pcl::visualization::PointCloudColorHandlerCustom<pcl::PointXYZ> correspondenceCloudColor(correspondenceCloud, 255, 0, 0);
    	viewer.addPointCloud(pclSourceCloud,pclSourceCloudColor,"input_source");
    	viewer.addPointCloud(pclSinkCloud,pclSinkCloudColor,"input_sink");
    	viewer.addPointCloud(correspondenceCloud,correspondenceCloudColor,"correspondeces");

    	while (!viewer.wasStopped ())
    		{
        	viewer.spinOnce();
        	pcl_sleep (0.01);
    		} 

	delete(correspodenceMap);
	}

void handler(int sig) {
  void *array[10];
  size_t size;

  // get void*'s for all entries on the stack
  size = backtrace(array, 10);

  // print out all the frames to stderr
  fprintf(stderr, "Error: signal %d:\n", sig);
  backtrace_symbols_fd(array, size, STDERR_FILENO);
  exit(1);
}

int main(int argc, char** argv)
	{
	signal(SIGSEGV, handler);
	Ransac3DTestInterface interface("Ransac3D", 100, 40);
	interface.Run();
	}

/** @} */
