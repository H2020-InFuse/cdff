/* --------------------------------------------------------------------------
*
* (C) Copyright …
*
* ---------------------------------------------------------------------------
*/

/*!
 * @file GuiTestPointCloudModelLocalisation.cpp
 * @date 29/03/2018
 * @author Alessandro Bianco
 */

/*!
 * @addtogroup GuiTests
 * 
 * Implementation of the GuiTestPointCloudModelLocalisation class.
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

using namespace Converters;
using namespace VisualPointFeatureVector3DWrapper;
using namespace PoseWrapper;
using namespace PointCloudWrapper;

/* --------------------------------------------------------------------------
 *
 * Public Member Functions
 *
 * --------------------------------------------------------------------------
 */
GuiTestPointCloudModelLocalisation::GuiTestPointCloudModelLocalisation(std::string configurationFilePath, std::string sceneFilePath, std::string modelFilePath) 
	{
	this->configurationFilePath = configurationFilePath;
	this->sceneFilePath = sceneFilePath;
	this->modelFilePath = modelFilePath;
	}

GuiTestPointCloudModelLocalisation::~GuiTestPointCloudModelLocalisation()
	{

	}

void GuiTestPointCloudModelLocalisation::Run(CDFF::DFPC::PointCloudModelLocalisationInterface* detector3d)
	{
	detector3d->setConfigurationFile(configurationFilePath);
	detector3d->setup();

	PointCloudConstPtr scene = LoadPointCloud(sceneFilePath);
	PointCloudConstPtr model = LoadPointCloud(modelFilePath);

	detector3d->sceneInput(*scene);
	detector3d->modelInput(*model);
	detector3d->run();

	const Pose3D& pose = detector3d->poseOutput();
	bool success = detector3d->successOutput();

	delete(scene);
	delete(model);

	PRINT_TO_LOG("success", success);
	PRINT_TO_LOG("pose", ToString(pose) );
	}

/* --------------------------------------------------------------------------
 *
 * Private Member Functions
 *
 * --------------------------------------------------------------------------
 */
PointCloudConstPtr GuiTestPointCloudModelLocalisation::LoadPointCloud(std::string file)
	{
	pcl::PointCloud<pcl::PointXYZ>::Ptr pclCloud(new pcl::PointCloud<pcl::PointXYZ>() );
	pcl::io::loadPLYFile(file, *pclCloud);
	PointCloudConstPtr cloud = pointCloudConverter.Convert(pclCloud);
	return cloud;
	}

/** @} */
