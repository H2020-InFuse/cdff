/**
 * @author Alessandro Bianco
 */

/**
 * @addtogroup DFNs
 * @{
 */

#ifndef DISPARITYMAPPING_HPP
#define DISPARITYMAPPING_HPP

#include "StereoReconstructionInterface.hpp"

#include <PointCloud.hpp>
#include <FrameToMatConverter.hpp>
#include <Helpers/ParametersListHelper.hpp>

#include <opencv2/core/core.hpp>

namespace dfn_ci
{
	/**
	 * Scene reconstruction (as a 3D pointcloud) from 2D stereo images, using
	 * the Konolige disparity mapping algorithm.
	 *
	 * Processing steps: (i) grayscaling of the images, (ii) computation of the
	 * Konolige disparity map, (iii) scene reconstruction based on the disparity
	 * map using a reprojection algorithm, (iv) downsampling of the generated
	 * pointcloud.
	 *
	 * @param prefilter.maximum
	 * @param prefilter.size
	 * @param prefilter.type
	 *
	 * @param disparities.minimum
	 *        disparity threshold
	 * @param disparities.numberOfIntervals
	 *        number of detected disparity intervals
	 * @param disparities.useMaximumDifference
	 * @param disparities.maximumDifference
	 * @param disparities.speckleRange
	 * @param disparities.speckleWindow
	 *
	 * @param blocksMatching.blockSize
	 *        dimension of the pixel blocks that are matched during disparity
	 *        computation, must be odd and larger than or equal to 5
	 * @param blocksMatching.uniquenessRatio
	 * @param blocksMatching.textureThreshold
	 * @param blocksMatching.smallerBlockSize
	 *
	 * @param firstRegionOfInterest
	 * @param secondRegionOfInterest
	 *
	 * @param pointCloudSamplingDensity
	 *        downsampling ratio: a number between 0 and 1 that describes how
	 *        much downsampling of the generated pointcloud is desired. The
	 *        pointcloud is subsampled at positions that are multiples of n,
	 *        where n = 1/pointCloudSamplingDensity.
	 *
	 * @param useDisparityToDepthMap
	 *        defines whether the camera parameters are provided as a disparity-
	 *        to-depth matrix or as the focal length, principle points, and
	 *        baseline
	 *
	 * @param disparityToDepthMap
	 *        camera parameters in the form of a 4-by-4 disparity-to-depth
	 *        matrix: provide the elements of this matrix via parameters called
	 *        Element_X_Y, where X and Y are between 0 and 3
	 *
	 * @param stereoCameraParameters
	 *        camera parameters in the form of the focal length and principal
	 *        point of the left camera and the distance between the two cameras:
	 *        the parameters to use to provide this information are called
	 *        LeftFocalLength, LeftPrinciplePointX, LeftPrinciplePointY, and
	 *        Baseline, respectively
	 *
	 * @param reconstructionSpace
	 *        a bounding box for the reconstructed scene, provided via
	 *        parameters called LimitX, LimitY, LimitZ. A reconstructed point
	 *        of coordinates (x,y,z) is accepted into the pointcloud if
	 *        -LimitX <= x <= LimitX, -LimitY <= y <= LimitY, 0 < z <= LimitZ.
	 */
	class DisparityMapping : public StereoReconstructionInterface
	{
		public:

			DisparityMapping();
			virtual ~DisparityMapping();

			virtual void configure();
			virtual void process();

		private:

			static const float EPSILON;

			enum PrefilterType
			{
				NORMALIZED_RESPONSE,
				XSOBEL
			};
			class PrefilterTypeHelper : public Helpers::ParameterHelper<PrefilterType, std::string>
			{
				public:
					PrefilterTypeHelper(const std::string& parameterName, PrefilterType& boundVariable, const PrefilterType& defaultValue);
				private:
					PrefilterType Convert(const std::string& value);
			};

			struct ReconstructionSpace
			{
				float limitX;
				float limitY;
				float limitZ;
			};

			struct Rectangle
			{
				int topLeftColumn;
				int topLeftRow;
				int numberOfColumns;
				int numberOfRows;
			};

			struct PrefilterOptionsSet
			{
				int size;
				PrefilterType type;
				int maximum;
			};

			struct DisparitiesOptionsSet
			{
				int minimum;
				int numberOfIntervals;
				bool useMaximumDifference;
				int maximumDifference;
				int speckleRange;
				int speckleWindow;
			};

			struct BlocksMatchingOptionsSet
			{
				int blockSize;
				int smallerBlockSize;
				int textureThreshold;
				int uniquenessRatio;
			};

			struct StereoCameraParameters
			{
				float leftFocalLength;
				float leftPrinciplePointX;
				float leftPrinciplePointY;
				float baseline;
			};

			typedef float DisparityToDepthMap[16];
			struct DisparityMappingOptionsSet
			{
				ReconstructionSpace reconstructionSpace;
				PrefilterOptionsSet prefilter;
				DisparitiesOptionsSet disparities;
				Rectangle firstRegionOfInterest;
				Rectangle secondRegionOfInterest;
				BlocksMatchingOptionsSet blocksMatching;
				DisparityToDepthMap disparityToDepthMap;
				float pointCloudSamplingDensity;
				float voxelGridLeafSize;
				bool useDisparityToDepthMap;
				StereoCameraParameters stereoCameraParameters;
			};

			cv::Mat disparityToDepthMap;

			Helpers::ParametersListHelper parametersHelper;
			DisparityMappingOptionsSet parameters;
			static const DisparityMappingOptionsSet DEFAULT_PARAMETERS;

			cv::Mat ComputePointCloud(cv::Mat leftImage, cv::Mat rightImage);

			PointCloudWrapper::PointCloudConstPtr Convert(cv::Mat cvPointCloud);
			PointCloudWrapper::PointCloudConstPtr ConvertWithPeriodicSampling(cv::Mat cvPointCloud);
			PointCloudWrapper::PointCloudConstPtr ConvertWithVoxelFilter(cv::Mat cvPointCloud);
			cv::Mat Convert(DisparityToDepthMap disparityToDepthMap);
			Converters::FrameToMatConverter frameToMat;

			cv::Mat ComputePointCloudFromDisparity(cv::Mat disparity);

			void ValidateParameters();

			#ifdef TESTING
				#define SAVE_DISPARITY_MATRIX(disparity) disparityMatrix = disparity
			#else
				#define SAVE_DISPARITY_MATRIX(disparity)
			#endif
	};
}

#endif // DISPARITYMAPPING_HPP

/** @} */
