/* --------------------------------------------------------------------------
*
* (C) Copyright …
*
* ---------------------------------------------------------------------------
*/

/*!
 * @file OrbFlannRansacDecomposition.cpp
 * @date 26/03/2018
 * @author Alessandro Bianco
 */

/*!
 * @addtogroup DFNsTest
 * 
 * Performance Test for the integration of Orb extractor descriptor, Flann matcher, fundamental matrix RANSAC, and essential matrix decomposition
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
#include <FundamentalMatrixComputation/FundamentalMatrixRansac.hpp>
#include <CamerasTransformEstimation/EssentialMatrixDecomposition.hpp>

#include <Stubs/Common/ConversionCache/CacheHandler.hpp>
#include <ConversionCache/ConversionCache.hpp>
#include <Mocks/Common/Converters/FrameToMatConverter.hpp>
#include <Mocks/Common/Converters/VisualPointFeatureVector2DToMatConverter.hpp>
#include <Mocks/Common/Converters/MatToVisualPointFeatureVector2DConverter.hpp>
#include <Mocks/Common/Converters/MatToTransform3DConverter.hpp>
#include <MatToFrameConverter.hpp>

#include <Errors/Assert.hpp>
#include <PerformanceTests/DFNsIntegration/PerformanceTestInterface.hpp>


using namespace dfn_ci;
using namespace Common;
using namespace Converters;
using namespace FrameWrapper;
using namespace VisualPointFeatureVector2DWrapper;
using namespace CorrespondenceMap2DWrapper;
using namespace PoseWrapper;
using namespace MatrixWrapper;


class OrbFlannRansacDecomposition : public PerformanceTestInterface
	{
	public:
		OrbFlannRansacDecomposition(std::string folderPath, std::vector<std::string> baseConfigurationFileNamesList, std::string performanceMeasuresFileName);
		~OrbFlannRansacDecomposition();
	protected:

	private:
		Stubs::CacheHandler<VisualPointFeatureVector2DConstPtr, cv::Mat >* stubVisualFeaturesCache;
		Mocks::VisualPointFeatureVector2DToMatConverter* mockVisualFeaturesConverter;

		Stubs::CacheHandler<cv::Mat, VisualPointFeatureVector2DConstPtr>* stubVisualFeaturesCache2;
		Mocks::MatToVisualPointFeatureVector2DConverter* mockVisualFeaturesConverter2;

		Stubs::CacheHandler<FrameConstPtr, cv::Mat>* stubInputCache;
		Mocks::FrameToMatConverter* mockInputConverter;

		Stubs::CacheHandler<cv::Mat, Pose3DConstPtr>* stubPoseCache;
		Mocks::MatToTransform3DConverter* mockPoseConverter;

		cv::Mat cvLeftImage;
		cv::Mat cvRightImage;

		FrameConstPtr leftFrame;
		FrameConstPtr rightFrame;
		VisualPointFeatureVector2DConstPtr leftFeaturesVector;
		VisualPointFeatureVector2DConstPtr rightFeaturesVector;
		CorrespondenceMap2DConstPtr correspondenceMap;
		bool ransacSuccess;
		Matrix3dConstPtr fundamentalMatrix;
		bool decompositionSuccess;
		Pose3DConstPtr pose;

		OrbDetectorDescriptor* orb;
		FlannMatcher* flann;
		FundamentalMatrixRansac* ransac;
		EssentialMatrixDecomposition* decomposition;
		void SetupMocksAndStubs();

		bool SetNextInputs();
		void ExecuteDfns();
		MeasuresMap ExtractMeasures();
	};

OrbFlannRansacDecomposition::OrbFlannRansacDecomposition(std::string folderPath, std::vector<std::string> baseConfigurationFileNamesList, std::string performanceMeasuresFileName)
	: PerformanceTestInterface(folderPath, baseConfigurationFileNamesList, performanceMeasuresFileName)
	{
	orb = new OrbDetectorDescriptor();
	flann = new FlannMatcher();
	ransac = new FundamentalMatrixRansac();
	decomposition = new EssentialMatrixDecomposition();
	AddDfn(orb);
	AddDfn(flann);
	AddDfn(ransac);
	AddDfn(decomposition);
	SetupMocksAndStubs();

	leftFrame = NULL;
	rightFrame = NULL;
	leftFeaturesVector = NULL;
	rightFeaturesVector = NULL;
	correspondenceMap = NULL;
	fundamentalMatrix = NULL;
	pose = NULL;

	ransacSuccess = false;
	decompositionSuccess = false;
	}

OrbFlannRansacDecomposition::~OrbFlannRansacDecomposition()
	{
	delete(stubInputCache);
	delete(mockInputConverter);
	delete(stubVisualFeaturesCache);
	delete(mockVisualFeaturesConverter);
	delete(stubVisualFeaturesCache2);
	delete(mockVisualFeaturesConverter2);
	delete(stubPoseCache);
	delete(mockPoseConverter);

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
	if (fundamentalMatrix != NULL)
		{
		delete(fundamentalMatrix);
		}
	if (pose != NULL)
		{
		delete(pose);
		}
	}

void OrbFlannRansacDecomposition::SetupMocksAndStubs()
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

	stubPoseCache = new Stubs::CacheHandler<cv::Mat, Pose3DConstPtr>();
	mockPoseConverter = new Mocks::MatToTransform3DConverter;
	ConversionCache<cv::Mat, Pose3DConstPtr, MatToTransform3DConverter>::Instance(stubPoseCache, mockPoseConverter);
	}

bool OrbFlannRansacDecomposition::SetNextInputs()
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

void OrbFlannRansacDecomposition::ExecuteDfns()
	{
	if (leftFeaturesVector != NULL)
		{
		delete(leftFeaturesVector);
		}
	orb->imageInput(leftFrame);
	orb->process();
	leftFeaturesVector = orb->featuresSetOutput();

	if (rightFeaturesVector != NULL)
		{
		delete(rightFeaturesVector);
		}
	orb->imageInput(rightFrame);
	orb->process();
	rightFeaturesVector = orb->featuresSetOutput();

	if (correspondenceMap != NULL)
		{
		delete(correspondenceMap);
		}
	flann->sinkFeaturesVectorInput( leftFeaturesVector );
	flann->sourceFeaturesVectorInput( rightFeaturesVector );
	flann->process();
	correspondenceMap = flann->correspondenceMapOutput();

	if (fundamentalMatrix != NULL)
		{
		delete(fundamentalMatrix);
		}
	ransac->correspondenceMapInput(correspondenceMap);
	ransac->process();
	ransacSuccess = ransac->successOutput();
	fundamentalMatrix = ransac->fundamentalMatrixOutput();

	if (pose != NULL)
		{
		delete(pose);
		}
	if (ransacSuccess)
		{
		decomposition->fundamentalMatrixInput(fundamentalMatrix);
		decomposition->correspondenceMapInput(correspondenceMap);
		decomposition->process();
		decompositionSuccess = decomposition->successOutput();
		pose = decomposition->transformOutput();
		}
	else
		{
		decompositionSuccess = false;
		pose = NULL;
		}
	}

OrbFlannRansacDecomposition::MeasuresMap OrbFlannRansacDecomposition::ExtractMeasures()
	{
	static unsigned testId = 0;
	testId++;
	MeasuresMap measuresMap;

	measuresMap["RansacSuccess"] = ransacSuccess;
	measuresMap["DecompositionSuccess"] = decompositionSuccess;

	return measuresMap;
	}


int main(int argc, char** argv)
	{
	std::vector<std::string> baseConfigurationFiles =
		{
		"OrbExtractorDescriptor_PerformanceTest_1.yaml",
		"FlannMatcher_PerformanceTest_1.yaml",
		"FundamentalMatrixRansac_PerformanceTest_1.yaml",
		"EssentialMatrixDecomposition_PerformanceTest_1.yaml"
		};
	OrbFlannRansacDecomposition interface("../tests/ConfigurationFiles/DFNsIntegration/VisualOdometry", baseConfigurationFiles, "Orb_Flann_Ransac_Decomposition.txt");
	interface.Run();
	};

/** @} */
