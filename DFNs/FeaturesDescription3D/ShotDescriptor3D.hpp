/* --------------------------------------------------------------------------
*
* (C) Copyright …
*
* --------------------------------------------------------------------------
*/

/*!
 * @file ShotDescriptor3D.hpp
 * @date 24/01/2018
 * @author Alessandro Bianco
 */

/*!
 * @addtogroup DFNs
 * 
 *  @brief This DFN executes computes the SHOT descriptors of a set of keypoints according to the implementation in PCL library. 
 * 
 * This DFN implementation requires the following parameters:
 * @param localReferenceFrameEstimationRadius, 
 * @param searchRadius, when computing the descriptor, neighbour points within this distance are used in the computation
 * @param forceNormalsEstimation, this determines whether the surface normals have to be recomputed again, regardless of whether they are in the input or not;
 * @param enableNormalsEstimation, this determines whether the absence of normals in the input will generate an error or will cause the DFN to compute the normals;
 * @param outputFormat, this defines the output format of the keypoints cloud. If "Positions" is chosen, then the output point cloud contains the 3d coordinates of the keypoints, if "References" is chosen
 * 			then the output point cloud contains the indices of the 3d point as they appear in the input point cloud.
 * @param normalEstimationOptions.searchRadius, when computing the normals on a point, neighbour points within this distance are used in the computation
 * @param normalEstimationOptions.neighboursSetSize, when computing the normals on a point, the closest neighbours are used in the computation, and this parameters defines how many are included.
 *
 * @{
 */

#ifndef SHOT_DESCRIPTOR_3D_HPP
#define SHOT_DESCRIPTOR_3D_HPP

/* --------------------------------------------------------------------------
 *
 * Includes
 *
 * --------------------------------------------------------------------------
 */
#include <FeaturesDescription3D/FeaturesDescription3DInterface.hpp>
#include <PointCloud.hpp>
#include <VisualPointFeatureVector3D.hpp>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <stdlib.h>
#include <string>
#include <pcl/features/shot.h>
#include <boost/make_shared.hpp>
#include <yaml-cpp/yaml.h>
#include <Helpers/ParametersListHelper.hpp>


namespace dfn_ci {

/* --------------------------------------------------------------------------
 *
 * Class definition
 *
 * --------------------------------------------------------------------------
 */
    class ShotDescriptor3D : public FeaturesDescription3DInterface
    {
	/* --------------------------------------------------------------------
	 * Public
	 * --------------------------------------------------------------------
	 */
        public:
            ShotDescriptor3D();
            ~ShotDescriptor3D();
            void process();
            void configure();

	/* --------------------------------------------------------------------
	 * Protected
	 * --------------------------------------------------------------------
	 */
        protected:

	/* --------------------------------------------------------------------
	 * Private
	 * --------------------------------------------------------------------
	 */	
	private:

		enum OutputFormat
			{
			POSITIONS_OUTPUT,
			REFERENCES_OUTPUT
			};
		class OutputFormatHelper : public Helpers::ParameterHelper<OutputFormat, std::string>
			{
			public:
				OutputFormatHelper(const std::string& parameterName, OutputFormat& boundVariable, const OutputFormat& defaultValue);
			private:
				OutputFormat Convert(const std::string& value);
			};

		struct BaseOptionsSet
			{
			float localReferenceFrameEstimationRadius;
			double searchRadius;
			OutputFormat outputFormat;
			bool enableNormalsEstimation;
			bool forceNormalsEstimation;
			};

		struct NormalEstimationOptionsSet
			{
			double searchRadius;
			int neighboursSetSize;
			};

		struct ShotOptionsSet
			{
			BaseOptionsSet baseOptions;
			NormalEstimationOptionsSet normalEstimationOptions;
			};

		Helpers::ParametersListHelper parametersHelper;
		ShotOptionsSet parameters;
		static const ShotOptionsSet DEFAULT_PARAMETERS;

		pcl::PointCloud<pcl::SHOT352>::ConstPtr ComputeShotDescriptors
			(
			pcl::PointCloud<pcl::PointXYZ>::ConstPtr pointCloud, 
			pcl::IndicesConstPtr indicesList,
			pcl::PointCloud<pcl::Normal>::ConstPtr optionalNormalsCloud
			);
		pcl::PointCloud<pcl::Normal>::ConstPtr EstimateNormals(pcl::PointCloud<pcl::PointXYZ>::ConstPtr pointCloud, pcl::search::KdTree<pcl::PointXYZ>::Ptr kdTree);
		pcl::IndicesConstPtr Convert( const VisualPointFeatureVector3DWrapper::VisualPointFeatureVector3DConstPtr featuresVector);
		VisualPointFeatureVector3DWrapper::VisualPointFeatureVector3DConstPtr Convert
			(
			const pcl::PointCloud<pcl::PointXYZ>::ConstPtr inputCloud,
			const pcl::IndicesConstPtr indicesList,
			const pcl::PointCloud<pcl::SHOT352>::ConstPtr shotPointCloud
			);

		void ValidateParameters();
		void ValidateMandatoryInputs(pcl::PointCloud<pcl::PointXYZ>::ConstPtr pointCloud, pcl::IndicesConstPtr indicesList);
		bool IsNormalsCloudValid(pcl::PointCloud<pcl::PointXYZ>::ConstPtr pointCloud, pcl::PointCloud<pcl::Normal>::ConstPtr normalsCloud);
    };
}
#endif
/* ShotDescriptor3D.hpp */
/** @} */
