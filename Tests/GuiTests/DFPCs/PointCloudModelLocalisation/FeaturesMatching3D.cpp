/* --------------------------------------------------------------------------
*
* (C) Copyright …
*
* ---------------------------------------------------------------------------
*/

/*!
 * @file FeaturesMatching3D.cpp
 * @date 26/02/2018
 * @author Alessandro Bianco
 */

/*!
 * @addtogroup DFNsTest
 * 
 * Testing application for the DFPC implementation FeaturesMatching3D.
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
#include "GuiTestPointCloudModelLocalisation.hpp"
#include <PointCloudModelLocalisation/FeaturesMatching3D.hpp>
#include <Errors/Assert.hpp>

#include <pcl/io/ply_io.h>
#include <pcl/visualization/pcl_visualizer.h>

#include <Types/CPP/VisualPointFeatureVector3D.hpp>
#include <Types/CPP/Pose.hpp>
#include <Types/CPP/PointCloud.hpp>

#include <Visualizers/OpenCVVisualizer.hpp>
#include <Visualizers/PCLVisualizer.hpp>

#include <boost/make_shared.hpp>

using namespace CDFF::DFPC::PointCloudModelLocalisation;
using namespace Converters;
using namespace VisualPointFeatureVector3DWrapper;
using namespace PoseWrapper;
using namespace PointCloudWrapper;

int main(int argc, char** argv)
	{
	ASSERT(argc == 1 || argc >= 4, "Please provide three paramters: ConfigurationFilePath, SceneFilePath, ModelFilePath");

	std::string configurationFilePath = argv[1];
	std::string sceneFilePath = argv[2];
	std::string modelFilePath = argv[3];

	GuiTestPointCloudModelLocalisation test(configurationFilePath, sceneFilePath, modelFilePath);
	FeaturesMatching3D* matching = new FeaturesMatching3D();

	test.Run(matching);

	delete(matching);
	return 0;
	};

/** @} */
