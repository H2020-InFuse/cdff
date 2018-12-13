/**
 * @author Alessandro Bianco
 */

/**
 * @addtogroup DFNs
 * @{
 */

#ifndef STEREORECONSTRUCTION_SCANLINEOPTIMIZATION_HPP
#define STEREORECONSTRUCTION_SCANLINEOPTIMIZATION_HPP

#include "StereoReconstructionInterface.hpp"

#include <Types/CPP/PointCloud.hpp>
#include <Types/CPP/Frame.hpp>
#include <Helpers/ParametersListHelper.hpp>

#include <opencv2/core/core.hpp>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>

namespace CDFF
{
namespace DFN
{
namespace StereoReconstruction
{
	/**
	 * Scene reconstruction (as a 3D pointcloud) from 2D stereo images, using
	 * the Adaptive-Cost 2-Pass Scanline Optimization disparity mapping
	 * algorithm (by Tombari).
	 *
	 * Processing steps: (i) conversion of the images to PCL representation,
	 * (ii) computation of a disparity map using Tombari's algorithm, (iii)
	 * scene reconstruction based on the disparity map using a reprojection
	 * algorithm, (iv) downsampling of the generated pointcloud.
	 *
	 * @param costAggregationRadius
	 * @param spatialBandwidth
	 * @param colorBandwidth
	 * @param weakSmoothnessPenalty
	 * @param strongSmoothnessPenalty
	 *
	 * @param matchingOptionsSet.numberOfDisparities
	 *        number of detected disparity intervals
	 * @param matchingOptionsSet.horizontalOffset
	 * @param matchingOptionsSet.ratioFilter
	 * @param matchingOptionsSet.peakFilter
	 * @param matchingOptionsSet.usePreprocessing
	 * @param matchingOptionsSet.useLeftRightConsistencyCheck
	 * @param matchingOptionsSet.leftRightConsistencyThreshold
	 *
	 * @param pointCloudSamplingDensity
	 *        downsampling ratio: a number between 0 and 1 that describes how
	 *        much downsampling of the generated pointcloud is desired. The
	 *        pointcloud is subsampled at positions that are multiples of n,
	 *        where n = 1/pointCloudSamplingDensity.
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
	 *
	 * @reference The algorithm is adapted from Liang Wang, Miao Liao, Minglun
	 *            Gong, Ruigang Yang, and David Nister (2006), "High Quality
	 *            Real-Time Stereo using Adaptive Cost Aggregation and Dynamic
	 *            Programming", Third IEEE International Symposium on 3D Data
	 *            Processing, Visualization, and Transmission, 798-805.
	 */
	class ScanlineOptimization : public StereoReconstructionInterface
	{
		public:

			ScanlineOptimization();
			virtual ~ScanlineOptimization();

			virtual void configure() override;
			virtual void process() override;

		private:

			static const float EPSILON;

			typedef pcl::PointCloud<pcl::RGB> PclImage;
			typedef pcl::PointCloud<pcl::RGB>::Ptr PclImagePtr;
			typedef pcl::PointCloud<pcl::RGB>::ConstPtr PclImageConstPtr;

			typedef pcl::PointCloud<pcl::PointXYZ> PclPointCloud;
			typedef pcl::PointCloud<pcl::PointXYZ>::Ptr PclPointCloudPtr;
			typedef pcl::PointCloud<pcl::PointXYZ>::ConstPtr PclPointCloudConstPtr;

			struct ReconstructionSpace
			{
				float limitX;
				float limitY;
				float limitZ;
			};

			struct CameraParameters
			{
				float leftPrinciplePointX;
				float leftPrinciplePointY;
				float leftFocalLength;
				float baseline;
			};

			struct MatchingOptionsSet
			{
				int numberOfDisparities;
				int horizontalOffset;
				int ratioFilter;
				int peakFilter;
				bool usePreprocessing;
				bool useLeftRightConsistencyCheck;
				int leftRightConsistencyThreshold;
			};

			struct ScanlineOptimizationOptionsSet
			{
				int costAggregationRadius;
				int spatialBandwidth;
				int colorBandwidth;
				int strongSmoothnessPenalty;
				int weakSmoothnessPenalty;
				float pointCloudSamplingDensity;
				float voxelGridLeafSize;
				MatchingOptionsSet matchingOptionsSet;
				CameraParameters cameraParameters;
				ReconstructionSpace reconstructionSpace;
			};

			Helpers::ParametersListHelper parametersHelper;
			ScanlineOptimizationOptionsSet parameters;
			static const ScanlineOptimizationOptionsSet DEFAULT_PARAMETERS;

			PclPointCloudPtr ComputePointCloud(PclImagePtr leftImage, PclImagePtr rightImage);

			PclImagePtr Convert(FrameWrapper::FrameConstPtr frame);

			PointCloudWrapper::PointCloudConstPtr SampleCloud(PclPointCloudConstPtr pointCloud);
			PointCloudWrapper::PointCloudConstPtr SampleCloudWithPeriodicSampling(PclPointCloudConstPtr pointCloud);
			PointCloudWrapper::PointCloudConstPtr SampleCloudWithVoxelGrid(PclPointCloudConstPtr pointCloud);
			cv::Mat PclImageToCvMatrix(PclImagePtr pclImage);

			void ValidateParameters();

			#ifdef TESTING
				#define SAVE_DISPARITY_MATRIX(visualMap) disparityMatrix = PclImageToCvMatrix(visualMap);
			#else
				#define SAVE_DISPARITY_MATRIX(visualMap)
			#endif
	};
}
}
}

#endif // STEREORECONSTRUCTION_SCANLINEOPTIMIZATION_HPP

/** @} */
