/* --------------------------------------------------------------------------
*
* (C) Copyright …
*
* --------------------------------------------------------------------------
*/

/*!
 * @file HarrisDetector3D.hpp
 * @date 01/12/2017
 * @author Alessandro Bianco
 */

/*!
 * @addtogroup DFNs
 * 
 *  This DFN implements the Harris Detector for 3D Point Clouds.
 *  
 *
 * @{
 */

#ifndef HARRIS_DETECTOR_3D_HPP
#define HARRIS_DETECTOR_3D_HPP

/* --------------------------------------------------------------------------
 *
 * Includes
 *
 * --------------------------------------------------------------------------
 */
#include <FeaturesExtraction3D/FeaturesExtraction3DInterface.hpp>
#include <PointCloud.hpp>
#include <VisualPointFeatureVector3D.hpp>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <stdlib.h>
#include <string>
#include <pcl/keypoints/harris_3d.h>
#include <yaml-cpp/yaml.h>
#include <Helpers/ParametersListHelper.hpp>


namespace dfn_ci {

/* --------------------------------------------------------------------------
 *
 * Class definition
 *
 * --------------------------------------------------------------------------
 */
    class HarrisDetector3D : public FeaturesExtraction3DInterface
    {
	/* --------------------------------------------------------------------
	 * Public
	 * --------------------------------------------------------------------
	 */
        public:
            HarrisDetector3D();
            ~HarrisDetector3D();
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

		typedef pcl::HarrisKeypoint3D<pcl::PointXYZ, pcl::PointXYZI>::ResponseMethod HarrisMethod;
		class HarrisMethodHelper : public Helpers::ParameterHelper<HarrisMethod, std::string>
			{
			public:
				HarrisMethodHelper(const std::string& parameterName, HarrisMethod& boundVariable, const HarrisMethod& defaultValue);
			private:
				HarrisMethod Convert(const std::string& value);
			};

		struct HarryOptionsSet
			{
			bool nonMaxSuppression;
			float radius;
			float searchRadius;
			float detectionThreshold;
			bool enableRefinement;
			int numberOfThreads;
			HarrisMethod method;
			OutputFormat outputFormat;
			};

		Helpers::ParametersListHelper parametersHelper;
		HarryOptionsSet parameters;
		static const HarryOptionsSet DEFAULT_PARAMETERS;

		pcl::PointIndicesConstPtr ComputeHarrisPoints(pcl::PointCloud<pcl::PointXYZ>::ConstPtr pointCloud);
		VisualPointFeatureVector3DWrapper::VisualPointFeatureVector3DConstPtr Convert(const pcl::PointCloud<pcl::PointXYZ>::ConstPtr inputCloud, const pcl::PointIndicesConstPtr indicesList);

		void ValidateParameters();
		void ValidateInputs(pcl::PointCloud<pcl::PointXYZ>::ConstPtr pointCloud);
    };
}
#endif
/* HarrisDetector3D.hpp */
/** @} */
