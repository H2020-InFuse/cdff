/* --------------------------------------------------------------------------
*
* (C) Copyright …
*
* --------------------------------------------------------------------------
*/

/*!
 * @file Ransac3D.hpp
 * @date 17/01/2018
 * @author Alessandro Bianco
 */

/*!
 * @addtogroup DFNs
 * 
 *  This DFN implements the RANSAC for matching points between two 3D point clouds.
 *  
 *
 * @{
 */

#ifndef RANSAC_3D_HPP
#define RANSAC_3D_HPP

/* --------------------------------------------------------------------------
 *
 * Includes
 *
 * --------------------------------------------------------------------------
 */
#include <FeaturesMatching3D/FeaturesMatching3DInterface.hpp>
#include <PointCloud.hpp>
#include <CorrespondenceMap3D.hpp>
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
    class Ransac3D : public FeaturesMatching3DInterface
    {
	/* --------------------------------------------------------------------
	 * Public
	 * --------------------------------------------------------------------
	 */
        public:
            Ransac3D();
            ~Ransac3D();
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

		struct RansacOptionsSet
			{
			int maxIterationsNumber;
			float outliersFreeProbability;
			float distanceThreshold;
			float samplesMaxDistance;
			};

		RansacOptionsSet parameters;

		CorrespondenceMap3DWrapper::CorrespondenceMap3DConstPtr ComputeCorrespondences(pcl::PointCloud<pcl::PointXYZ>::ConstPtr sourceCloud, pcl::PointCloud<pcl::PointXYZ>::ConstPtr sinkCloud);

		void ValidateParameters();
		void ValidateInputs(pcl::PointCloud<pcl::PointXYZ>::ConstPtr sourceCloud, pcl::PointCloud<pcl::PointXYZ>::ConstPtr sinkCloud);

		void Configure(const YAML::Node& configurationNode);
    };
}
#endif
/* Ransac3D.hpp */
/** @} */
