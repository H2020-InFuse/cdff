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
 *  This DFN implements the Shot Descriptor Estimation for 3D Point Clouds.
 *  
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
