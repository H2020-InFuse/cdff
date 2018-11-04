/* --------------------------------------------------------------------------
*
* (C) Copyright …
*
* ---------------------------------------------------------------------------
*/

/*!
 * @file GuiTestPointCloudModelLocalisation.hpp
 * @date 29/03/2018
 * @author Alessandro Bianco
 */

/*!
 * @addtogroup GuiTests
 * 
 * This is the main interface class for implementations of the DFPC Point Cloud Model Localisation.
 * It performs some common operations
 *
 * @{
 */

#ifndef GUI_TEST_POINT_CLOUD_MODEL_LOCALISATION_HPP
#define GUI_TEST_POINT_CLOUD_MODEL_LOCALISATION_HPP


/* --------------------------------------------------------------------------
 *
 * Includes
 *
 * --------------------------------------------------------------------------
 */
#include <PointCloudModelLocalisation/PointCloudModelLocalisationInterface.hpp>
#include <Errors/Assert.hpp>

#include <pcl/io/ply_io.h>
#include <pcl/visualization/pcl_visualizer.h>

#include <Types/CPP/VisualPointFeatureVector3D.hpp>
#include <Types/CPP/Pose.hpp>
#include <Types/CPP/PointCloud.hpp>
#include <Converters/PclPointCloudToPointCloudConverter.hpp>

#include <stdlib.h>
#include <fstream>
#include <string>
#include <vector>

/* --------------------------------------------------------------------------
 *
 * Class definition
 *
 * --------------------------------------------------------------------------
 */
class GuiTestPointCloudModelLocalisation
	{
	/* --------------------------------------------------------------------
	 * Public
	 * --------------------------------------------------------------------
	 */
	public:
		GuiTestPointCloudModelLocalisation(std::string configurationFilePath, std::string sceneFilePath, std::string modelFilePath);
		~GuiTestPointCloudModelLocalisation();

		void Run(CDFF::DFPC::PointCloudModelLocalisationInterface* detector3d);

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
		std::string modelFilePath;
		std::string sceneFilePath;
		std::string configurationFilePath;

		Converters::PclPointCloudToPointCloudConverter pointCloudConverter;

		PointCloudWrapper::PointCloudConstPtr LoadPointCloud(std::string file);

	};

#endif

/* GuiTestPointCloudModelLocalisation.hpp */
/** @} */
