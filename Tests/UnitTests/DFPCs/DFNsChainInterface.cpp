/* --------------------------------------------------------------------------
*
* (C) Copyright …
*
* ---------------------------------------------------------------------------
*/

/*!
 * @file DFNsChainInterface.cpp
 * @date 26/02/2018
 * @author Alessandro Bianco
 */

/*!
 * @addtogroup DFNsTest
 * 
 * Unit Test for the DFNsChainInterface Common Methods.
 * 
 * 
 * @{
 */


/* --------------------------------------------------------------------------
 *
 * Definitions
 * Catch definition must be before the includes, otherwise catch will not compile.
 *
 * --------------------------------------------------------------------------
 */
#define CATCH_CONFIG_MAIN

/* --------------------------------------------------------------------------
 *
 * Includes
 *
 * --------------------------------------------------------------------------
 */
#include <Catch/catch.h>
#include <DFNsChainInterface.hpp>
#include <ImageFiltering/ImageUndistortion.hpp>
#include <FeaturesDescription3D/ShotDescriptor3D.hpp>
#include <Errors/Assert.hpp>

using namespace dfpc_ci;
using namespace dfn_ci;

/* --------------------------------------------------------------------------
 *
 * Concrete Class for testing (DFNsChainInterface is pure virtual)
 *
 * --------------------------------------------------------------------------
 */
class DFNsChain : public DFNsChainInterface
	{
	public:
		void process() 
			{
			REQUIRE( dynamic_cast<ImageUndistortion*>(dfnsSet["ZedImageUndistortion"]) != nullptr);
			REQUIRE( dynamic_cast<ShotDescriptor3D*>(dfnsSet["3dDescriptor"]) != nullptr);
			}
	};


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

TEST_CASE( "Success Call to Configure", "[configureSuccess]" ) 
	{
	DFNsChain dfnsChain;
	dfnsChain.setConfigurationFile("../../tests/ConfigurationFiles/DFPCs/dfns_chain_conf01.yaml");
	dfnsChain.configure();

	YAML::Node dfn0Node= YAML::LoadFile( "../../tests/ConfigurationFiles/DFPCs/DFN_0.yaml" );
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

	YAML::Node dfn1Node= YAML::LoadFile( "../../tests/ConfigurationFiles/DFPCs/DFN_1.yaml" );
	REQUIRE(  dfn1Node[0]["Name"].as<std::string>() == "GeneralParameters") ;
	REQUIRE_CLOSE(  dfn1Node[0]["LocalReferenceFrameEstimationRadius"].as<float>(), 0.1) ;
	REQUIRE(  dfn1Node[0]["OutputFormat"].as<std::string>() == "Positions") ;
	REQUIRE(  dfn1Node[0]["EnableNormalsEstimation"].as<bool>() == true) ;
	REQUIRE(  dfn1Node[0]["ForceNormalsEstimation"].as<bool>() == true) ;
	REQUIRE_CLOSE(  dfn1Node[0]["SearchRadius"].as<float>(), 0.01) ;

	REQUIRE(  dfn1Node[1]["Name"].as<std::string>() == "NormalEstimationParameters") ;
	REQUIRE_CLOSE(  dfn1Node[1]["SearchRadius"].as<float>(), 0.01) ;
	REQUIRE(  dfn1Node[1]["NeighboursSetSize"].as<int>() == 0) ;

	dfnsChain.process();
	}


/** @} */
