/* --------------------------------------------------------------------------
*
* (C) Copyright …
*
* ---------------------------------------------------------------------------
*/

/*!
 * @file SelectionTester.cpp
 * @date 11/05/2018
 * @author Alessandro Bianco
 */

/*!
 * @addtogroup GuiTests
 * 
 * Implementation of the SelectionTester class.
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
#include "SelectionTester.hpp"
#include <opencv2/highgui/highgui.hpp>
#include<pcl/io/ply_io.h>
#include <ctime>

using namespace dfn_ci;
using namespace Converters;
using namespace VisualPointFeatureVector3DWrapper;
using namespace PointCloudWrapper;
using namespace Common;
using namespace CorrespondenceMap3DWrapper;
using namespace SupportTypes;
using namespace PoseWrapper;

#define DELETE_IF_NOT_NULL(pointer) \
	if (pointer != NULL) \
		{ \
		delete(pointer); \
		}

/* --------------------------------------------------------------------------
 *
 * Public Member Functions
 *
 * --------------------------------------------------------------------------
 */
SelectionTester::SelectionTester() 
	{
	featuresDescriptorConfigurationFilePath = "";
	featuresMatcherConfigurationFilePath = "";

	inputSourceCloud = NULL;
	inputSinkCloud = NULL;
	inputSourceKeypointsVector = NULL;
	inputSinkKeypointsVector = NULL;
	sourceFeaturesVector = NULL;
	sinkFeaturesVector = NULL;
	outputCorrespondenceMap = NULL;
	referenceCorrespondenceMap = NULL;
	sourcePoseInSink = NULL;

	matcher = NULL;
	descriptor = NULL;

	dfnsWereLoaded = false;
	inputCloudsWereLoaded = false;
	inputKeypointsWereLoaded = false;
	precisionReferenceWasLoaded = false;

	SetUpMocksAndStubs();
	}

SelectionTester::~SelectionTester()
	{
	delete(stubCloudCache);
	delete(mockCloudConverter);

	delete(stubInverseCloudCache);
	delete(mockInverseCloudConverter);

	delete(stubVector3dCache);
	delete(mockVector3dConverter);

	delete(stubNormalsCache);
	delete(mockNormalsConverter);

	delete(stubFeaturesCloudCache);
	delete(mockFeaturesCloudConverter);

	delete(stubTransformCache);
	delete(mockTransformConverter);
	
	DELETE_IF_NOT_NULL(inputSourceCloud);
	DELETE_IF_NOT_NULL(inputSinkCloud);
	DELETE_IF_NOT_NULL(inputSourceKeypointsVector);
	DELETE_IF_NOT_NULL(inputSinkKeypointsVector);
	DELETE_IF_NOT_NULL(sourceFeaturesVector);
	DELETE_IF_NOT_NULL(sinkFeaturesVector);
	DELETE_IF_NOT_NULL(outputCorrespondenceMap);
	DELETE_IF_NOT_NULL(referenceCorrespondenceMap);
	DELETE_IF_NOT_NULL(sourcePoseInSink);
	}

void SelectionTester::SetDfns(dfn_ci::FeaturesDescription3DInterface* descriptor, dfn_ci::FeaturesMatching3DInterface* matcher)
	{
	this->descriptor = descriptor;
	this->matcher = matcher;

	if (featuresDescriptorConfigurationFilePath != "")
		{
		ConfigureDfns();
		}
	}

void SelectionTester::SetConfigurationFilePaths(std::string featuresDescriptorConfigurationFilePath, std::string featuresMatcherConfigurationFilePath)
	{
	this->featuresDescriptorConfigurationFilePath = featuresDescriptorConfigurationFilePath;
	this->featuresMatcherConfigurationFilePath = featuresMatcherConfigurationFilePath;

	if (descriptor != NULL && matcher != NULL)
		{
		ConfigureDfns();
		}
	}

void SelectionTester::SetInputFilesPaths(std::string sourceCloudFilePath, std::string sinkCloudFilePath, std::string correspondencesFilePath)
	{
	this->sourceCloudFilePath = sourceCloudFilePath;
	this->sinkCloudFilePath = sinkCloudFilePath;
	this->correspondencesFilePath = correspondencesFilePath;

	LoadInputCloud(sourceCloudFilePath, inputSourceCloud);
	LoadInputCloud(sinkCloudFilePath, inputSinkCloud);
	inputCloudsWereLoaded = true;

	LoadReferenceCorrespondenceMap();
	inputKeypointsWereLoaded = true;
	precisionReferenceWasLoaded = true;
	}

#define PROCESS_AND_MEASURE_TIME(dfn) \
	{ \
	beginTime = clock(); \
	dfn->process(); \
	endTime = clock(); \
	processingTime += float(endTime - beginTime) / CLOCKS_PER_SEC; \
	}

void SelectionTester::ExecuteDfns()
	{
	clock_t beginTime, endTime;
	float processingTime = 0;

	descriptor->pointcloudInput(*inputSourceCloud);
	descriptor->featuresInput(*inputSourceKeypointsVector);
	PROCESS_AND_MEASURE_TIME(descriptor);
	DELETE_IF_NOT_NULL(sourceFeaturesVector);
	VisualPointFeatureVector3DPtr newSourceFeaturesVector = NewVisualPointFeatureVector3D();
	Copy (descriptor->featuresOutput(), *newSourceFeaturesVector);
	sourceFeaturesVector = newSourceFeaturesVector;

	descriptor->pointcloudInput(*inputSinkCloud);
	descriptor->featuresInput(*inputSinkKeypointsVector);
	PROCESS_AND_MEASURE_TIME(descriptor);
	DELETE_IF_NOT_NULL(sinkFeaturesVector);
	VisualPointFeatureVector3DPtr newSinkFeaturesVector = NewVisualPointFeatureVector3D();
	Copy (descriptor->featuresOutput(), *newSinkFeaturesVector);
	sinkFeaturesVector = newSinkFeaturesVector;

	matcher->sourceFeaturesInput(*sourceFeaturesVector);
	matcher->sinkFeaturesInput(*sinkFeaturesVector);
	PROCESS_AND_MEASURE_TIME(matcher);
	DELETE_IF_NOT_NULL(sourcePoseInSink);
	Pose3DPtr newSourcePoseInSink = NewPose3D();
	Copy( matcher->transformOutput(), *newSourcePoseInSink);
	sourcePoseInSink = newSourcePoseInSink;
	matcherSuccess = matcher->successOutput();

	PRINT_TO_LOG("Processing took (seconds): ", processingTime);
	}

bool SelectionTester::AreCorrespondencesValid(float percentageThreshold)
	{
	ASSERT(inputCloudsWereLoaded && inputKeypointsWereLoaded && precisionReferenceWasLoaded, "Error: some inputs were not correctly loaded");

	PRINT_TO_LOG("Source Cloud size is", GetNumberOfPoints(*inputSourceCloud));
	PRINT_TO_LOG("Sink Cloud size is", GetNumberOfPoints(*inputSinkCloud));
	PRINT_TO_LOG("The number of correspondences is", GetNumberOfCorrespondences(*referenceCorrespondenceMap));

	if (!matcherSuccess)
		{
		PRINT_TO_LOG("3D features matching failed", "");
		return false;
		}
	
	PRINT_TO_LOG("Estimate source pose in sink is:", ToString(*sourcePoseInSink));
	ComputeMatchesFromPose();
	bool correspondencesAreValid = ValidateCorrespondences(percentageThreshold);
	if (!correspondencesAreValid)
		{
		PRINT_TO_LOG("Correspondences are not valid according to percentage threshold", percentageThreshold);
		}

	return correspondencesAreValid;
	}

/* --------------------------------------------------------------------------
 *
 * Private Member Functions
 *
 * --------------------------------------------------------------------------
 */
void SelectionTester::SetUpMocksAndStubs()
	{
	stubCloudCache = new Stubs::CacheHandler<pcl::PointCloud<pcl::PointXYZ>::ConstPtr, PointCloudConstPtr>;
	mockCloudConverter = new Mocks::PclPointCloudToPointCloudConverter();
	ConversionCache<pcl::PointCloud<pcl::PointXYZ>::ConstPtr, PointCloudConstPtr, PclPointCloudToPointCloudConverter>::Instance(stubCloudCache, mockCloudConverter);

	stubInverseCloudCache = new Stubs::CacheHandler<PointCloudConstPtr, pcl::PointCloud<pcl::PointXYZ>::ConstPtr>;
	mockInverseCloudConverter = new Mocks::PointCloudToPclPointCloudConverter();
	ConversionCache<PointCloudConstPtr, pcl::PointCloud<pcl::PointXYZ>::ConstPtr, PointCloudToPclPointCloudConverter>::Instance(stubInverseCloudCache, mockInverseCloudConverter);

	stubVector3dCache = new Stubs::CacheHandler<cv::Mat, VisualPointFeatureVector3DConstPtr>();
	mockVector3dConverter = new Mocks::MatToVisualPointFeatureVector3DConverter();
	ConversionCache<cv::Mat, VisualPointFeatureVector3DConstPtr, MatToVisualPointFeatureVector3DConverter>::Instance(stubVector3dCache, mockVector3dConverter);

	stubNormalsCache = new Stubs::CacheHandler<PointCloudConstPtr, pcl::PointCloud<pcl::Normal>::ConstPtr>();
	mockNormalsConverter = new Mocks::PointCloudToPclNormalsCloudConverter();
	ConversionCache<PointCloudConstPtr, pcl::PointCloud<pcl::Normal>::ConstPtr, PointCloudToPclNormalsCloudConverter>::Instance(stubNormalsCache, mockNormalsConverter);

	stubFeaturesCloudCache = new Stubs::CacheHandler<VisualPointFeatureVector3DConstPtr, PointCloudWithFeatures>();
	mockFeaturesCloudConverter = new Mocks::VisualPointFeatureVector3DToPclPointCloudConverter();
	ConversionCache<VisualPointFeatureVector3DConstPtr, PointCloudWithFeatures, VisualPointFeatureVector3DToPclPointCloudConverter>::Instance(stubFeaturesCloudCache, mockFeaturesCloudConverter);

	stubTransformCache = new Stubs::CacheHandler<Eigen::Matrix4f, Transform3DConstPtr>();
	mockTransformConverter = new Mocks::EigenTransformToTransform3DConverter();
	ConversionCache<Eigen::Matrix4f, Transform3DConstPtr, EigenTransformToTransform3DConverter>::Instance(stubTransformCache, mockTransformConverter);
	}

void SelectionTester::LoadInputCloud(const std::string& cloudFilePath, PointCloudWrapper::PointCloudConstPtr& cloud)
	{
	pcl::PointCloud<pcl::PointXYZ>::Ptr pclPointCloud = boost::make_shared<pcl::PointCloud<pcl::PointXYZ> >();
	pcl::io::loadPLYFile(cloudFilePath, *pclPointCloud);

	DELETE_IF_NOT_NULL(cloud);
	cloud = pointCloudConverter.Convert(pclPointCloud);
	}

void SelectionTester::LoadReferenceCorrespondenceMap()
	{
	ASSERT(inputCloudsWereLoaded, "input clouds should be loaded before loading the correspondences");
	cv::FileStorage opencvFile(correspondencesFilePath, cv::FileStorage::READ);

	cv::Mat cvCorrespondenceMap;
	opencvFile["CorrespondenceMap"] >> cvCorrespondenceMap;
	opencvFile.release();

	ASSERT(cvCorrespondenceMap.rows > 0, "Error: correspondence map contains no keypoints");
	ASSERT(cvCorrespondenceMap.cols == 2, "Error: correspondence map has invalid format. It should contain a row for each correspondence, and two integers representing the points indices");
	ASSERT(cvCorrespondenceMap.type() == CV_32SC1, "Error: correspondence map invalid format. It should contain a row for each correspondence, and two integers representing the points indices");

	VisualPointFeatureVector3DPtr newInputSourceKeypointsVector = NewVisualPointFeatureVector3D();
	VisualPointFeatureVector3DPtr newInputSinkKeypointsVector = NewVisualPointFeatureVector3D();
	CorrespondenceMap3DPtr newReferenceCorrespondenceMap = NewCorrespondenceMap3D();

	for(int row = 0; row < cvCorrespondenceMap.rows; row++)
		{
		int32_t sourceIndex = cvCorrespondenceMap.at<int32_t>(row, 0);
		int32_t sinkIndex = cvCorrespondenceMap.at<int32_t>(row, 1);
		AddPoint(*newInputSourceKeypointsVector, sourceIndex);
		AddPoint(*newInputSinkKeypointsVector, sinkIndex);

		BaseTypesWrapper::Point3D sourcePoint, sinkPoint;
		sourcePoint.x = GetXCoordinate(*inputSourceCloud, sourceIndex);
		sourcePoint.y = GetYCoordinate(*inputSourceCloud, sourceIndex);
		sourcePoint.z = GetZCoordinate(*inputSourceCloud, sourceIndex);
		sinkPoint.x = GetXCoordinate(*inputSinkCloud, sinkIndex);
		sinkPoint.y = GetYCoordinate(*inputSinkCloud, sinkIndex);
		sinkPoint.z = GetZCoordinate(*inputSinkCloud, sinkIndex);
		AddCorrespondence(*newReferenceCorrespondenceMap, sourcePoint, sinkPoint, 1);
		}

	inputSourceKeypointsVector = newInputSourceKeypointsVector;
	inputSinkKeypointsVector = newInputSinkKeypointsVector;
	referenceCorrespondenceMap = newReferenceCorrespondenceMap;
	}

void SelectionTester::ConfigureDfns()
	{
	descriptor->setConfigurationFile(featuresDescriptorConfigurationFilePath);
	descriptor->configure();

	matcher->setConfigurationFile(featuresMatcherConfigurationFilePath);
	matcher->configure();

	dfnsWereLoaded = true;
	}

bool SelectionTester::ValidateCorrespondences(float percentageThreshold)
	{
	int outputCorrespondecesNumber = GetNumberOfCorrespondences(*outputCorrespondenceMap);
	int referenceCorrespondencesNumber = GetNumberOfCorrespondences(*referenceCorrespondenceMap);

	int correctMatchesNumber = 0;
	for(int referenceIndex = 0; referenceIndex < referenceCorrespondencesNumber; referenceIndex++)
		{
		for (int outputIndex = 0; outputIndex < outputCorrespondecesNumber; outputIndex++)
			{
			if (CorrespondencesAreTheSame(referenceIndex, outputIndex))
				{
				correctMatchesNumber++;
				}
			}
		}
	float correctMatchesPercentage = ((float)correctMatchesNumber) / ((float)referenceCorrespondencesNumber);
	PRINT_TO_LOG("Percentage of correct matches", correctMatchesPercentage);

	return (correctMatchesPercentage >= percentageThreshold);
	}

bool SelectionTester::CorrespondencesAreTheSame(int referenceIndex, int outputIndex)
	{
	BaseTypesWrapper::Point3D referenceSource = GetSource(*referenceCorrespondenceMap, referenceIndex);
	BaseTypesWrapper::Point3D referenceSink = GetSink(*referenceCorrespondenceMap, referenceIndex);		
	BaseTypesWrapper::Point3D outputSource = GetSource(*outputCorrespondenceMap, outputIndex);
	BaseTypesWrapper::Point3D outputSink = GetSink(*outputCorrespondenceMap, outputIndex);	

	bool sameSource = (referenceSource.x == outputSource.x && referenceSource.y == outputSource.y && referenceSource.z == outputSource.z);
	bool sameSink = (referenceSink.x == outputSink.x && referenceSink.y == outputSink.y && referenceSink.z == outputSink.z);
	return ( sameSource && sameSink );
	}

void SelectionTester::ComputeMatchesFromPose()
	{
	Eigen::Quaternion<float> rotation(GetWOrientation(*sourcePoseInSink), GetXOrientation(*sourcePoseInSink), GetYOrientation(*sourcePoseInSink), GetZOrientation(*sourcePoseInSink));
	Eigen::Translation<float, 3> translation(GetXPosition(*sourcePoseInSink), GetYPosition(*sourcePoseInSink), GetZPosition(*sourcePoseInSink));
	AffineTransform affineTransform = rotation * translation;

	float totalDistance;
	CorrespondenceMap3DPtr newOutputCorrespondenceMap = NewCorrespondenceMap3D();
	for(int sourcePointIndex = 0; sourcePointIndex < GetNumberOfPoints(*inputSourceKeypointsVector); sourcePointIndex++)
		{
		int32_t sourcePointReferenceIndex = GetReferenceIndex(*inputSourceKeypointsVector, sourcePointIndex);
		BaseTypesWrapper::Point3D sourcePoint;
		sourcePoint.x = GetXCoordinate(*inputSourceCloud, sourcePointReferenceIndex);
		sourcePoint.y = GetYCoordinate(*inputSourceCloud, sourcePointReferenceIndex);
		sourcePoint.z = GetZCoordinate(*inputSourceCloud, sourcePointReferenceIndex);

		BaseTypesWrapper::Point3D transformedSourcePoint = TransformPoint(sourcePoint, affineTransform);
		float closestDistance;
		BaseTypesWrapper::Point3D sinkPoint = FindClosestSinkPointTo(transformedSourcePoint, closestDistance);	
		totalDistance += closestDistance;

		AddCorrespondence(*newOutputCorrespondenceMap, sourcePoint, sinkPoint, 1);
		}

	DELETE_IF_NOT_NULL(outputCorrespondenceMap);
	outputCorrespondenceMap = newOutputCorrespondenceMap;

	PRINT_TO_LOG("Average source-sink matched keypoints distance is:", (totalDistance/(float)GetNumberOfCorrespondences(*outputCorrespondenceMap)) );
	}

BaseTypesWrapper::Point3D SelectionTester::TransformPoint(const BaseTypesWrapper::Point3D& point, const AffineTransform& affineTransform)
	{
	Eigen::Vector3f eigenPoint(point.x, point.y, point.z);
	Eigen::Vector3f eigenTransformedPoint = affineTransform * eigenPoint;
	BaseTypesWrapper::Point3D transformedPoint;
	transformedPoint.x = eigenTransformedPoint.x();
	transformedPoint.y = eigenTransformedPoint.y();
	transformedPoint.z = eigenTransformedPoint.z();
	return transformedPoint;
	}

BaseTypesWrapper::Point3D SelectionTester::FindClosestSinkPointTo(const BaseTypesWrapper::Point3D& sourcePoint, float& closestDistance)
	{
	BaseTypesWrapper::Point3D closestPoint;
	closestDistance = -1;
	for(int sinkPointIndex = 0; sinkPointIndex < GetNumberOfPoints(*inputSinkKeypointsVector); sinkPointIndex++)
		{
		int32_t sinkPointReferenceIndex = GetReferenceIndex(*inputSinkKeypointsVector, sinkPointIndex);
		BaseTypesWrapper::Point3D sinkPoint;
		sinkPoint.x = GetXCoordinate(*inputSinkCloud, sinkPointReferenceIndex);
		sinkPoint.y = GetYCoordinate(*inputSinkCloud, sinkPointReferenceIndex);
		sinkPoint.z = GetZCoordinate(*inputSinkCloud, sinkPointReferenceIndex);

		float differenceX = sourcePoint.x - sinkPoint.x;
		float differenceY = sourcePoint.y - sinkPoint.y;
		float differenceZ = sourcePoint.z - sinkPoint.z;
		float distance = std::sqrt(differenceX*differenceX + differenceY*differenceY + differenceZ*differenceZ);
		
		if ( closestDistance == -1 || (closestDistance > distance) )
			{
			closestDistance = distance;
			closestPoint = sinkPoint;	
			}
		}

	return closestPoint;	
	}

/** @} */
