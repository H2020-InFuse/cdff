/* --------------------------------------------------------------------------
*
* (C) Copyright …
*
* ---------------------------------------------------------------------------
*/

/*!
 * @file ParametersHelperTest.cpp
 * @date 01/02/2018
 * @author Alessandro Bianco
 */

/*!
 * @addtogroup CommonTests
 * 
 * Testing the parameters helper classes.
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
#include <Helpers/ParametersListHelper.hpp>
#include <Catch/catch.h>
#include <Errors/Assert.hpp>

using namespace Helpers;

TEST_CASE( "Read One Parameter", "[ReadOneParameter]" )
	{
	ParametersListHelper helper;
	int parameter;

	helper.AddParameter<int>("Group", "Parameter", parameter, 54);
	helper.ReadFile("../tests/ConfigurationFiles/Common/Helpers/ParametersHelper_OneParameter.yaml");

	REQUIRE(parameter == 23);
	} 

TEST_CASE( "Read Two Parameter in One Group", "[ReadTwoParametersInOneGroup]" )
	{
	const double EPSILON = 1e-9;
	const double EXPECTED_DOUBLE = 65.1;
	ParametersListHelper helper;
	double parameterDouble;
	std::string parameterString;

	helper.AddParameter<double>("Group", "ParameterDouble", parameterDouble, 78.1);
	helper.AddParameter<std::string>("Group", "ParameterString", parameterString, "ThisString");
	helper.ReadFile("../tests/ConfigurationFiles/Common/Helpers/ParametersHelper_TwoInOneParameters.yaml");

	REQUIRE(parameterDouble < EXPECTED_DOUBLE + EXPECTED_DOUBLE*EPSILON);
	REQUIRE(parameterDouble > EXPECTED_DOUBLE - EXPECTED_DOUBLE*EPSILON);
	REQUIRE(parameterString == "AnotherString");
	} 

TEST_CASE( "Read Two Parameter in two Groups", "[ReadTwoParametersInTwoGroups]" )
	{
	const float EPSILON = 1e-5;
	const float EXPECTED_FLOAT = 21.90;
	ParametersListHelper helper;
	float parameterFloat;
	bool parameterBool;

	helper.AddParameter<float>("GroupFloat", "Parameter", parameterFloat, 38.1);
	helper.AddParameter<bool>("GroupBool", "Parameter", parameterBool, true);
	helper.ReadFile("../tests/ConfigurationFiles/Common/Helpers/ParametersHelper_TwoInTwoParameters.yaml");

	REQUIRE(parameterFloat < EXPECTED_FLOAT + EXPECTED_FLOAT*EPSILON);
	REQUIRE(parameterFloat > EXPECTED_FLOAT - EXPECTED_FLOAT*EPSILON);
	REQUIRE(parameterBool == false);
	} 

TEST_CASE( "Ignoring Unexpected Parameters", "[IgnoreUnexpectedParameters]" )
	{
	const float EPSILON = 1e-5;
	const float EXPECTED_FLOAT = 21.90;
	ParametersListHelper helper;
	float parameterFloat;
	bool parameterBool;

	helper.AddParameter<float>("GroupFloat", "Parameter", parameterFloat, 38.1);
	helper.AddParameter<bool>("GroupBool", "Parameter", parameterBool, true);
	helper.ReadFile("../tests/ConfigurationFiles/Common/Helpers/ParametersHelper_UnexpectedParameter.yaml");

	REQUIRE(parameterFloat < EXPECTED_FLOAT + EXPECTED_FLOAT*EPSILON);
	REQUIRE(parameterFloat > EXPECTED_FLOAT - EXPECTED_FLOAT*EPSILON);
	REQUIRE(parameterBool == false);
	} 

TEST_CASE( "Using Defaults", "[UseDefaultsParameters]" )
	{
	const float EPSILON = 1e-5;
	const float EXPECTED_FLOAT = 38.1;
	ParametersListHelper helper;
	float parameterFloat;
	bool parameterBool;

	helper.AddParameter<float>("GroupFloat", "Parameter", parameterFloat, 38.1);
	helper.AddParameter<bool>("GroupBool", "Parameter", parameterBool, true);
	helper.ReadFile("../tests/ConfigurationFiles/Common/Helpers/ParametersHelper_UseDefaultParameter.yaml");

	REQUIRE(parameterFloat < EXPECTED_FLOAT + EXPECTED_FLOAT*EPSILON);
	REQUIRE(parameterFloat > EXPECTED_FLOAT - EXPECTED_FLOAT*EPSILON);
	REQUIRE(parameterBool == false);
	} 

TEST_CASE( "Helper Type with conversion", "[CustomParameterHelper]" )
	{
	enum State
		{
		START,
		STOP,
		SURRENDER
		};

	class StringStateHelper : public ParameterHelper<State, std::string>
		{
		public:
			StringStateHelper(const std::string& parameterName, State& boundVariable, const State& defaultValue) : 
				ParameterHelper(parameterName, boundVariable, defaultValue)
				{
				}

		private: 
			State Convert(const std::string& value)
				{
				if (value == "START")
					{
					return START;
					}
				else if (value == "STOP")
					{
					return STOP;
					}
				else if (value == "SURRENDER")
					{
					return SURRENDER;
					}
				return SURRENDER;
				}
		};

	ParametersListHelper helper;
	State parameterState;

	helper.AddParameter<State, StringStateHelper>("Group", "Parameter", parameterState, SURRENDER);
	helper.ReadFile("../tests/ConfigurationFiles/Common/Helpers/ParametersHelper_CustomHelper.yaml");

	REQUIRE(parameterState == START);
	} 





