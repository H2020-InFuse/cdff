/* --------------------------------------------------------------------------
*
* (C) Copyright …
*
* --------------------------------------------------------------------------
*/

/*!
 * @file Assert.hpp
 * @date 15/11/2017
 * @author Alessandro Bianco
 */

/*!
 * @addtogroup Common
 * 
 *  This is a collection of ASSERT macros. Each macro will check whether a condition is true. If the condition is false a diagnostic message is added to the log, the logged data is visualized and  
 *  the program is halted.
 *   
 * @{
 */

#ifndef ASSERT_HPP
#define ASSERT_HPP

/* --------------------------------------------------------------------------
 *
 * Includes
 *
 * --------------------------------------------------------------------------
 */
#include <Loggers/LoggerFactory.hpp>
#include <stdlib.h>
#include <sstream>
#include <exception>


/* --------------------------------------------------------------------------
 *
 * This Assert Exception is thrown when an assertion fail. It is not meant to
 * be caught. It is just used during testing, so that CATCH does not stop.
 *
 * --------------------------------------------------------------------------
 */
class AssertException: public std::exception
	{
 	virtual const char* what() const throw()
 	 	{
 	   	return "Assert Exception: A programmer-defined impossible condition has triggered";
 	 	}
	};


/* --------------------------------------------------------------------------
 *
 * Macros
 *
 * --------------------------------------------------------------------------
 */

#define PRINT_ERROR(message) \
	{ \
	LoggerFactory::GetLogger()->AddEntry(message, Logger::MessageType::ERROR); \
	LoggerFactory::GetLogger()->Print(); \
	LoggerFactory::GetLogger()->Clear(); \
	}

#define ASSERT(condition, message) \
	{ \
	if( !(condition) ) \
		{ \
		PRINT_ERROR(message) \
		ABORT_PROGRAM() \
		} \
	}

#define ASSERT_EQUAL(expression1, expression2, message) \
	{ \
	if ( (expression1) != (expression2) ) \
		{ \
		std::stringstream stream; \
		stream << #expression1 <<" evaluates to "<<(expression1)<<", "<<#expression2<<" evaluates to "<<(expression2)<<", message: "<<message;\
		std::string abortMessage = stream.str(); \
		PRINT_ERROR(abortMessage) \
		ABORT_PROGRAM() \
		} \
	}

#define ASSERT_CLOSE(expression1, expression2, resolution, message) \
	{ \
	if ( (expression1) < (expression2 - resolution*expression2) || (expression1) > (expression2 + resolution*expression2) ) \
		{ \
		std::stringstream stream; \
		stream << #expression1 <<" evaluates to "<<(expression1)<<", "<<#expression2<<" evaluates to "<<(expression2)<<", message: "<<message;\
		std::string abortMessage = stream.str(); \
		PRINT_ERROR(abortMessage) \
		ABORT_PROGRAM() \
		} \
	}	

#define WRITE_TO_LOG(message, value) \
	{ \
	std::stringstream stream; \
	stream << message << " " << (value); \
	LoggerFactory::GetLogger()->AddEntry(stream.str()); \
	}

#define PRINT_LOG() \
	{ \
	LoggerFactory::GetLogger()->Print(); \
	}

#define PRINT_TO_LOG(message, value) \
	{ \
	WRITE_TO_LOG(message, value) \
	PRINT_LOG() \
	LoggerFactory::GetLogger()->Clear(); \
	}

#define PRINT_WARNING(message) \
	{ \
	LoggerFactory::GetLogger()->AddEntry(message, Logger::MessageType::WARNING); \
	LoggerFactory::GetLogger()->Print(); \
	LoggerFactory::GetLogger()->Clear(); \
	}

#define VERIFY(condition, message) \
	{ \
	if( !(condition) ) \
		{ \
		PRINT_WARNING(message); \
		} \
	}

#define VERIFY_REQUIREMENT(condition, message) \
	{ \
	if( !(condition) ) \
		{ \
		PRINT_ERROR(message); \
		} \
	}

#define NOTIFY_ON_EXCEPTION(expression, message) \
	{ \
	try \
		{ \
		expression; \
		} \
	catch (std::exception e) \
		{ \
		PRINT_TO_LOG(message, "") \
		throw e; \
		} \
	}

#endif

/* Assert.hpp */
/** @} */
