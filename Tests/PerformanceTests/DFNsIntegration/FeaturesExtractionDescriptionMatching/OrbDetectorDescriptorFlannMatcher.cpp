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

#include <MatToFrameConverter.hpp>

#include <Errors/Assert.hpp>
#include <PerformanceTests/DFNsIntegration/PerformanceTestInterface.hpp>


using namespace dfn_ci;
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
		cv::Mat cvLeftImage;
		cv::Mat cvRightImage;

		FrameConstPtr leftFrame;
		FrameConstPtr rightFrame;
		VisualPointFeatureVector2DConstPtr leftFeaturesVector;
		VisualPointFeatureVector2DConstPtr rightFeaturesVector;
		CorrespondenceMap2DConstPtr correspondenceMap;

		OrbDetectorDescriptor* orb;
		FlannMatcher* flann;

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

	leftFrame = NULL;
	rightFrame = NULL;
	leftFeaturesVector = NULL;
	rightFeaturesVector = NULL;
	correspondenceMap = NULL;
	}

OrbDetectorDescriptorFlannMatcher::~OrbDetectorDescriptorFlannMatcher()
	{
	if (leftFrame != NULL)
		{
		delete(leftFrame);
		}
	if (rightFrame != NULL)
		{
		delete(rightFrame);
		}
	if (leftFeaturesVector != NULL)
		{
		delete(leftFeaturesVector);
		}
	if (rightFeaturesVector != NULL)
		{
		delete(rightFeaturesVector);
		}
	if (correspondenceMap != NULL)
		{
		delete(correspondenceMap);
		}
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
	if (leftFeaturesVector != NULL)
		{
		delete(leftFeaturesVector);
		}
	orb->frameInput(*leftFrame);
	orb->process();
	VisualPointFeatureVector2DPtr newLeftFeaturesVector = NewVisualPointFeatureVector2D();
	Copy( orb->featuresOutput(), *newLeftFeaturesVector);
	leftFeaturesVector = newLeftFeaturesVector;

	if (rightFeaturesVector != NULL)
		{
		delete(rightFeaturesVector);
		}
	orb->frameInput(*rightFrame);
	orb->process();
	VisualPointFeatureVector2DPtr newRightFeaturesVector = NewVisualPointFeatureVector2D();
	Copy( orb->featuresOutput(), *newRightFeaturesVector);
	rightFeaturesVector = newRightFeaturesVector;

	if (correspondenceMap != NULL)
		{
		delete(correspondenceMap);
		}
	flann->sinkFeaturesInput( *leftFeaturesVector );
	flann->sourceFeaturesInput( *rightFeaturesVector );
	flann->process();
	CorrespondenceMap2DPtr newCorrespondenceMap = NewCorrespondenceMap2D();
	Copy( flann->matchesOutput(), *newCorrespondenceMap);
	correspondenceMap = newCorrespondenceMap;
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
