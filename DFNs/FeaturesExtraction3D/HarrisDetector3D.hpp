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
		
		typedef pcl::HarrisKeypoint3D<pcl::PointXYZ, pcl::PointXYZI>::ResponseMethod HarrisMethod;

		struct HarryOptionsSet
			{
			bool nonMaxSuppression;
			float radius;
			float detectionThreshold;
			bool enableRefinement;
			int numberOfThreads;
			HarrisMethod method;
			};

		HarryOptionsSet parameters;

		cv::Mat ComputeHarrisPoints(pcl::PointCloud<pcl::PointXYZ>::ConstPtr pointCloud);
		HarrisMethod ConvertToMethod(std::string method);

		void ValidateParameters();
		void ValidateInputs(pcl::PointCloud<pcl::PointXYZ>::ConstPtr pointCloud);

		void Configure(const YAML::Node& configurationNode);
    };
}
#endif
/* HarrisDetector3D.hpp */
/** @} */
