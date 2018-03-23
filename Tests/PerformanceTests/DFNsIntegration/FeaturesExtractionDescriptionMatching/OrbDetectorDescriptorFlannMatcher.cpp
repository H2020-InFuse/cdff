/* --------------------------------------------------------------------------
*
* (C) Copyright …
*
* ---------------------------------------------------------------------------
*/

/*!
 * @file OrbDetectorDescriptorFlannMatcher.cpp
 * @date 22/03/2018
 * @author Alessandro Bianco
 */

/*!
 * @addtogroup DFNsTest
 * 
 * Performance Test for the integration of Orb extractor descriptor and Flann matcher
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

#include <FeaturesExtraction2D/OrbDetectorDescriptor.hpp>
#include <FeaturesMatching2D/FlannMatcher.hpp>

#include <Stubs/Common/ConversionCache/CacheHandler.hpp>
#include <ConversionCache/ConversionCache.hpp>
#include <Mocks/Common/Converters/FrameToMatConverter.hpp>
#include <Mocks/Common/Converters/VisualPointFeatureVector2DToMatConverter.hpp>
#include <Mocks/Common/Converters/MatToVisualPointFeatureVector2DConverter.hpp>
#include <MatToFrameConverter.hpp>

#include <Errors/Assert.hpp>
#include <PerformanceTests/DFNsIntegration/PerformanceTestInterface.hpp>


using namespace dfn_ci;
using namespace Common;
using namespace Converters;
using namespace FrameWrapper;
using namespace VisualPointFeatureVector2DWrapper;
using namespace CorrespondenceMap2DWrapper;


class OrbDetectorDescriptorFlannMatcher : public PerformanceTestInterface
	{
	public:
		OrbDetectorDescriptorFlannMatcher(std::string folderPath, std::vector<std::string> baseConfigurationFileNamesList, std::string performanceMeasuresFileName);
		~OrbDetectorDescriptorFlannMatcher();
	protected:

	private:
		Stubs::CacheHandler<VisualPointFeatureVector2DConstPtr, cv::Mat >* stubVisualFeaturesCache;
		Mocks::VisualPointFeatureVector2DToMatConverter* mockVisualFeaturesConverter;

		Stubs::CacheHandler<cv::Mat, VisualPointFeatureVector2DConstPtr>* stubVisualFeaturesCache2;
		Mocks::MatToVisualPointFeatureVector2DConverter* mockVisualFeaturesConverter2;

		Stubs::CacheHandler<FrameConstPtr, cv::Mat>* stubInputCache;
		Mocks::FrameToMatConverter* mockInputConverter;

		cv::Mat cvLeftImage;
		cv::Mat cvRightImage;

		FrameConstPtr leftFrame;
		FrameConstPtr rightFrame;
		CorrespondenceMap2DConstPtr correspondenceMap;

		OrbDetectorDescriptor* orb;
		FlannMatcher* flann;
		void SetupMocksAndStubs();

		bool SetNextInputs();
		void ExecuteDfns();
		MeasuresMap ExtractMeasures();
	};

OrbDetectorDescriptorFlannMatcher::OrbDetectorDescriptorFlannMatcher(std::string folderPath, std::vector<std::string> baseConfigurationFileNamesList, std::string performanceMeasuresFileName)
	: PerformanceTestInterface(folderPath, baseConfigurationFileNamesList, performanceMeasuresFileName)
	{
	orb = new OrbDetectorDescriptor();
	flann = new FlannMatcher();
	AddDfn(orb);
	AddDfn(flann);
	SetupMocksAndStubs();

	leftFrame = NULL;
	rightFrame = NULL;
	}

OrbDetectorDescriptorFlannMatcher::~OrbDetectorDescriptorFlannMatcher()
	{
	delete(stubInputCache);
	delete(mockInputConverter);
	delete(stubVisualFeaturesCache);
	delete(mockVisualFeaturesConverter);
	delete(stubVisualFeaturesCache2);
	delete(mockVisualFeaturesConverter2);

	if (leftFrame != NULL)
		{
		delete(leftFrame);
		}
	if (rightFrame != NULL)
		{
		delete(rightFrame);
		}
	if (correspondenceMap != NULL)
		{
		delete(correspondenceMap);
		}
	}

void OrbDetectorDescriptorFlannMatcher::SetupMocksAndStubs()
	{
	stubInputCache = new Stubs::CacheHandler<FrameConstPtr, cv::Mat>();
	mockInputConverter = new Mocks::FrameToMatConverter();
	ConversionCache<FrameConstPtr, cv::Mat, FrameToMatConverter>::Instance(stubInputCache, mockInputConverter);

	stubVisualFeaturesCache = new Stubs::CacheHandler<VisualPointFeatureVector2DConstPtr, cv::Mat>();
	mockVisualFeaturesConverter = new Mocks::VisualPointFeatureVector2DToMatConverter();
	ConversionCache<VisualPointFeatureVector2DConstPtr, cv::Mat, VisualPointFeatureVector2DToMatConverter>::Instance(stubVisualFeaturesCache, mockVisualFeaturesConverter);

	stubVisualFeaturesCache2 = new Stubs::CacheHandler<cv::Mat, VisualPointFeatureVector2DConstPtr>();
	mockVisualFeaturesConverter2 = new Mocks::MatToVisualPointFeatureVector2DConverter();
	ConversionCache<cv::Mat, VisualPointFeatureVector2DConstPtr, MatToVisualPointFeatureVector2DConverter>::Instance(stubVisualFeaturesCache2, mockVisualFeaturesConverter2);
	}

bool OrbDetectorDescriptorFlannMatcher::SetNextInputs()
	{
	static unsigned time = 0;

	if (time == 0)
		{
		cvLeftImage = cv::imread("../tests/Data/Images/RectifiedChair40Left.png", cv::IMREAD_COLOR);
		cvRightImage = cv::imread("../tests/Data/Images/RectifiedChair40Right.png", cv::IMREAD_COLOR);
		ASSERT( cvLeftImage.size() == cvRightImage.size(), "Performance Test Error: input images do not have same size");

		MatToFrameConverter converter;
		leftFrame = converter.Convert(cvLeftImage);
		rightFrame = converter.Convert(cvRightImage);

		time++;
		return true;
		}
	
	return false;
	}

void OrbDetectorDescriptorFlannMatcher::ExecuteDfns()
	{
	orb->imageInput(leftFrame);
	orb->process();
	VisualPointFeatureVector2DConstPtr leftFeaturesVector = orb->featuresSetOutput();

	orb->imageInput(rightFrame);
	orb->process();
	VisualPointFeatureVector2DConstPtr rightFeaturesVector = orb->featuresSetOutput();

	flann->sinkFeaturesVectorInput( leftFeaturesVector );
	flann->sourceFeaturesVectorInput( rightFeaturesVector );
	flann->process();
	correspondenceMap = flann->correspondenceMapOutput();
	}

OrbDetectorDescriptorFlannMatcher::MeasuresMap OrbDetectorDescriptorFlannMatcher::ExtractMeasures()
	{
	static unsigned testId = 0;
	testId++;
	MeasuresMap measuresMap;

	float outOfLineCost = 0;
	for(unsigned correspondenceIndex = 0; correspondenceIndex < GetNumberOfCorrespondences(*correspondenceMap); correspondenceIndex++)
		{
		BaseTypesWrapper::Point2D source = GetSource(*correspondenceMap, correspondenceIndex); 
		BaseTypesWrapper::Point2D sink = GetSink(*correspondenceMap, correspondenceIndex);

 		outOfLineCost += std::abs( source.y - sink.y );
		}
	measuresMap["OutOfLineCost"] = (outOfLineCost / static_cast<float>( GetNumberOfCorrespondences(*correspondenceMap) ) );
	measuresMap["NumberOfCorrespondences"] = GetNumberOfCorrespondences(*correspondenceMap);

	delete(correspondenceMap);
	correspondenceMap = NULL;
	return measuresMap;
	}


int main(int argc, char** argv)
	{
	std::vector<std::string> baseConfigurationFiles =
		{
		"OrbExtractorDescriptor_PerformanceTest_1.yaml",
		"FlannMatcher_PerformanceTest_1.yaml"
		};
	OrbDetectorDescriptorFlannMatcher interface("../tests/ConfigurationFiles/DFNsIntegration/FeaturesExtractionDescriptionMatching", baseConfigurationFiles, "OrbExtractorDescriptor_FlannMatcher.txt");
	interface.Run();
	};

/** @} */
