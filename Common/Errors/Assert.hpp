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
		LoggerFactory::GetLogger()->AddEntry(#message); \
		LoggerFactory::GetLogger()->Print(); \
		ABORT_PROGRAM() \
		} \
	}

#define ASSERT_EQUAL(expression1, expression2, message) \
	{ \
	if ( (expression1) != (expression2) ) \
		{ \
		std::stringstream stream; \
		stream << #expression1 <<" evaluates to "<<(expression1)<<", "<<#expression2<<" evaluates to "<<(expression2)<<", message: "<<#message;\
		LoggerFactory::GetLogger()->AddEntry(stream.str()); \
		LoggerFactory::GetLogger()->Print(); \
		ABORT_PROGRAM() \
		} \
	}	

#define WRITE_TO_LOG(message, value) \
	{ \
	std::stringstream stream; \
	stream << #message << " " << (value); \
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
		LoggerFactory::GetLogger()->AddEntry(#message); \
		LoggerFactory::GetLogger()->Print(); \
		LoggerFactory::GetLogger()->Clear(); \
		} \
	}

#endif

/* Assert.hpp */
/** @} */
