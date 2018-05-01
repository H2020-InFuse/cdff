/* --------------------------------------------------------------------------
*
* (C) Copyright …
*
* --------------------------------------------------------------------------
*/

/*!
 * @file MockMacro.hpp
 * @date 15/11/2017
 * @author Alessandro Bianco
 */

/*!
 * @addtogroup Mocks
 * 
 *  These macros are meant to speed-up the generation of Mock classes. They are meant to be put as the body of the methods the Mock overrides.
 *  You will need to pass as argument the baseClass, the methodName, the outputType, and the input parameters.
 *   
 *  Example Usage: 
 *  float MockCalculator::Division(int x, int y)
 *  	MOCK_METHOD(Tools::Calculator, Division, float, (x, y) )
 *
 *  Notes: 
 *  (1) the input parameters need to be wrapped in the brackets, no type should be specified;
 *  (2) if the output type contains commas (such as std::map<float, float>), you will need to create a new type with typedef.
 *
 *
 * @{
 */

#ifndef MOCK_MACRO_HPP
#define MOCK_MACRO_HPP


/* --------------------------------------------------------------------------
 *
 * Macros
 *
 * --------------------------------------------------------------------------
 */
#define MOCK_METHOD(baseClass, methodName, outputType, inputParameters) \
	{ \
	countersMap["methodName"]++; \
	 \
	outputType* plannedOutput = (outputType*) Mock::GetBehaviour(#methodName, countersMap["methodName"]); \
	 \
	if (plannedOutput == NULL) \
		{ \
		return baseClass::methodName inputParameters; \
		} \
	else \
		{ \
		return (*plannedOutput); \
		} \
	}	

#define MOCK_STATIC_METHOD(baseClass, methodName, outputType, inputParameters) \
	{ \
	staticCountersMap["methodName"]; \
	 \
	outputType* plannedOutput = (outputType*) Mock::GetStaticBehaviour(#methodName, staticCountersMap["methodName"]); \
	 \
	if (plannedOutput == NULL) \
		{ \
		return baseClass::methodName inputParameters; \
		} \
	else \
		{ \
		return (*plannedOutput); \
		} \
	}	

#define MOCK_VOID_METHOD(baseClass, methodName, outputType, inputParameter, outputParameter) \
	{ \
	countersMap["methodName"]++; \
	 \
	outputType* plannedOutput = (outputType*) Mock::GetBehaviour(#methodName, countersMap["methodName"]); \
	 \
	if (plannedOutput == NULL) \
		{ \
		baseClass::methodName (inputParameter, outputParameter); \
		} \
	else \
		{ \
		outputParameter = (*plannedOutput); \
		} \
	}

#endif

/* MockMacro.hpp */
/** @} */
