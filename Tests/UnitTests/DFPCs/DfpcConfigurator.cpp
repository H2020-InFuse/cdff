/* --------------------------------------------------------------------------
*
* (C) Copyright …
*
* ---------------------------------------------------------------------------
*/

/*!
 * @file DfpcConfigurator.cpp
 * @date 21/03/2018
 * @author Alessandro Bianco
 */

/*!
 * @addtogroup DFNsTest
 * 
 * Unit Test for the DfpcConfigurator Methods.
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
#include <catch.hpp>
#include <DfpcConfigurator.hpp>
#include <ImageFiltering/ImageUndistortion.hpp>
#include <FeaturesDescription3D/ShotDescriptor3D.hpp>
#include <Errors/Assert.hpp>

using namespace dfpc_ci;
using namespace dfn_ci;

/* --------------------------------------------------------------------------
 *
 * Test Cases
 *
 * --------------------------------------------------------------------------
 */
#define EPSILON 0.0001

#define REQUIRE_CLOSE( variable, value) \
	REQUIRE( variable < value + EPSILON ); \
	REQUIRE( variable > value - EPSILON );

TEST_CASE( "Success Call to Configure (DFPC configurator)", "[configureSuccess]" ) 
	{
	DfpcConfigurator configurator;
	configurator.configure("../tests/ConfigurationFiles/DFPCs/dfns_chain_conf01.yaml");

	YAML::Node dfn0Node= YAML::LoadFile( "../tests/ConfigurationFiles/DFPCs/DFN_ZedImageUndistortion.yaml" );
	REQUIRE(  dfn0Node[0]["Name"].as<std::string>() == "GeneralParameters") ;
	REQUIRE(  dfn0Node[0]["NumberOfTestPoints"].as<int>() == 20) ;

	REQUIRE(  dfn0Node[1]["Name"].as<std::string>() == "FirstCameraMatrix") ;
	REQUIRE_CLOSE(  dfn0Node[1]["FocalLengthX"].as<float>(), 1.0) ;
	REQUIRE_CLOSE(  dfn0Node[1]["FocalLengthY"].as<float>(), 1.0) ;
	REQUIRE_CLOSE(  dfn0Node[1]["PrinciplePointX"].as<float>(), 0.0) ;
	REQUIRE_CLOSE(  dfn0Node[1]["PrinciplePointY"].as<float>(), 0.0) ;

	REQUIRE(  dfn0Node[2]["Name"].as<std::string>() == "SecondCameraMatrix") ;
	REQUIRE_CLOSE(  dfn0Node[2]["FocalLengthX"].as<float>(), 1.0) ;
	REQUIRE_CLOSE(  dfn0Node[2]["FocalLengthY"].as<float>(), 1.0) ;
	REQUIRE_CLOSE(  dfn0Node[2]["PrinciplePointX"].as<float>(), 0.0) ;
	REQUIRE_CLOSE(  dfn0Node[2]["PrinciplePointY"].as<float>(), 0.0) ;

	YAML::Node dfn1Node= YAML::LoadFile( "../tests/ConfigurationFiles/DFPCs/DFN_3dDescriptor.yaml" );
	REQUIRE(  dfn1Node[0]["Name"].as<std::string>() == "GeneralParameters") ;
	REQUIRE_CLOSE(  dfn1Node[0]["LocalReferenceFrameEstimationRadius"].as<float>(), 0.1) ;
	REQUIRE(  dfn1Node[0]["OutputFormat"].as<std::string>() == "Positions") ;
	REQUIRE(  dfn1Node[0]["EnableNormalsEstimation"].as<bool>() == true) ;
	REQUIRE(  dfn1Node[0]["ForceNormalsEstimation"].as<bool>() == true) ;
	REQUIRE_CLOSE(  dfn1Node[0]["SearchRadius"].as<float>(), 0.01) ;

	REQUIRE(  dfn1Node[1]["Name"].as<std::string>() == "NormalEstimationParameters") ;
	REQUIRE_CLOSE(  dfn1Node[1]["SearchRadius"].as<float>(), 0.01) ;
	REQUIRE(  dfn1Node[1]["NeighboursSetSize"].as<int>() == 0) ;

	REQUIRE( dynamic_cast<ImageUndistortion*>( configurator.GetDfn("ZedImageUndistortion") ) != NULL );
	REQUIRE( dynamic_cast<ShotDescriptor3D*>( configurator.GetDfn("3dDescriptor") ) != NULL);
	}


TEST_CASE( "Success Call to Configure With Chain Parameters (DFPC configurator)", "[configureSuccessChainParameters]" ) 
	{
	DfpcConfigurator configurator;
	configurator.configure("../tests/ConfigurationFiles/DFPCs/dfns_chain_conf02.yaml");

	YAML::Node dfn0Node= YAML::LoadFile( "../tests/ConfigurationFiles/DFPCs/DFN_ZedImageUndistortion.yaml" );
	REQUIRE(  dfn0Node[0]["Name"].as<std::string>() == "GeneralParameters") ;
	REQUIRE(  dfn0Node[0]["NumberOfTestPoints"].as<int>() == 20) ;

	REQUIRE(  dfn0Node[1]["Name"].as<std::string>() == "FirstCameraMatrix") ;
	REQUIRE_CLOSE(  dfn0Node[1]["FocalLengthX"].as<float>(), 1.0) ;
	REQUIRE_CLOSE(  dfn0Node[1]["FocalLengthY"].as<float>(), 1.0) ;
	REQUIRE_CLOSE(  dfn0Node[1]["PrinciplePointX"].as<float>(), 0.0) ;
	REQUIRE_CLOSE(  dfn0Node[1]["PrinciplePointY"].as<float>(), 0.0) ;

	REQUIRE(  dfn0Node[2]["Name"].as<std::string>() == "SecondCameraMatrix") ;
	REQUIRE_CLOSE(  dfn0Node[2]["FocalLengthX"].as<float>(), 1.0) ;
	REQUIRE_CLOSE(  dfn0Node[2]["FocalLengthY"].as<float>(), 1.0) ;
	REQUIRE_CLOSE(  dfn0Node[2]["PrinciplePointX"].as<float>(), 0.0) ;
	REQUIRE_CLOSE(  dfn0Node[2]["PrinciplePointY"].as<float>(), 0.0) ;

	YAML::Node dfn1Node= YAML::LoadFile( "../tests/ConfigurationFiles/DFPCs/DFN_3dDescriptor.yaml" );
	REQUIRE(  dfn1Node[0]["Name"].as<std::string>() == "GeneralParameters") ;
	REQUIRE_CLOSE(  dfn1Node[0]["LocalReferenceFrameEstimationRadius"].as<float>(), 0.1) ;
	REQUIRE(  dfn1Node[0]["OutputFormat"].as<std::string>() == "Positions") ;
	REQUIRE(  dfn1Node[0]["EnableNormalsEstimation"].as<bool>() == true) ;
	REQUIRE(  dfn1Node[0]["ForceNormalsEstimation"].as<bool>() == true) ;
	REQUIRE_CLOSE(  dfn1Node[0]["SearchRadius"].as<float>(), 0.01) ;

	REQUIRE(  dfn1Node[1]["Name"].as<std::string>() == "NormalEstimationParameters") ;
	REQUIRE_CLOSE(  dfn1Node[1]["SearchRadius"].as<float>(), 0.01) ;
	REQUIRE(  dfn1Node[1]["NeighboursSetSize"].as<int>() == 0) ;

	YAML::Node chainNode= YAML::LoadFile( "../tests/ConfigurationFiles/DFPCs/DFNsChain.yaml" );
	REQUIRE(  chainNode[0]["Name"].as<std::string>() == "GeneralParameters") ;
	REQUIRE(  chainNode[0]["UndistortionTimes"].as<int>() == 2) ;

	REQUIRE( dynamic_cast<ImageUndistortion*>( configurator.GetDfn("ZedImageUndistortion") ) != NULL );
	REQUIRE( dynamic_cast<ShotDescriptor3D*>( configurator.GetDfn("3dDescriptor") ) != NULL);
	}

/** @} */
