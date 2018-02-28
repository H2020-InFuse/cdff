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
#ifndef TESTING
	#define ABORT_PROGRAM() exit(EXIT_FAILURE);
#else
	#define ABORT_PROGRAM() throw AssertException();
#endif

#define ASSERT(condition, message) \
	{ \
	if( !(condition) ) \
		{ \
		LoggerFactory::GetLogger()->AddEntry(message); \
		LoggerFactory::GetLogger()->Print(); \
		ABORT_PROGRAM() \
		} \
	}

#ifndef TESTING
	#define ASSERT_ON_TEST(condition, message) 
#else
	#define ASSERT_ON_TEST(condition, message) ASSERT(condition, message)
#endif

#define ASSERT_EQUAL(expression1, expression2, message) \
	{ \
	if ( (expression1) != (expression2) ) \
		{ \
		std::stringstream stream; \
		stream << #expression1 <<" evaluates to "<<(expression1)<<", "<<#expression2<<" evaluates to "<<(expression2)<<", message: "<<message;\
		LoggerFactory::GetLogger()->AddEntry(stream.str()); \
		LoggerFactory::GetLogger()->Print(); \
		ABORT_PROGRAM() \
		} \
	}

#define ASSERT_CLOSE(expression1, expression2, resolution, message) \
	{ \
	if ( (expression1) < (expression2 - resolution*expression2) || (expression1) > (expression2 + resolution*expression2) ) \
		{ \
		std::stringstream stream; \
		stream << #expression1 <<" evaluates to "<<(expression1)<<", "<<#expression2<<" evaluates to "<<(expression2)<<", message: "<<message;\
		LoggerFactory::GetLogger()->AddEntry(stream.str()); \
		LoggerFactory::GetLogger()->Print(); \
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

#define VERIFY(condition, message) \
	{ \
	if( !(condition) ) \
		{ \
		LoggerFactory::GetLogger()->AddEntry(message); \
		LoggerFactory::GetLogger()->Print(); \
		LoggerFactory::GetLogger()->Clear(); \
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

#ifndef TESTING
	#define DEBUG_WRITE_TO_LOG(message, value)
	#define DEBUG_PRINT_LOG()
	#define DEBUG_PRINT_TO_LOG(message, value) 
	#define DEBUG_VERIFY(condition, message)
	#define DEBUG_NOTIFY_ON_EXCEPTION(expression, message)
#else
	#define DEBUG_WRITE_TO_LOG(message, value) WRITE_TO_LOG(message, value)
	#define DEBUG_PRINT_LOG() PRINT_LOG()
	#define DEBUG_PRINT_TO_LOG(message, value) PRINT_TO_LOG(message, value)
	#define DEBUG_VERIFY(condition, message) VERIFY(condition, message)
	#define DEBUG_NOTIFY_ON_EXCEPTION(expression, message) NOTIFY_ON_EXCEPTION(expression, message)
#endif

#endif

/* Assert.hpp */
/** @} */
