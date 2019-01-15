/* --------------------------------------------------------------------------
*
* (C) Copyright …
*
* --------------------------------------------------------------------------
*/

/*!
 * @file AssertOnTest.hpp
 * @date 15/01/2018
 * @author Alessandro Bianco
 */

/*!
 * @addtogroup Common
 * 
 *  This is a collection of ASSERT macros to activate only during testing. 
 *  Each macro will check whether a condition is true. If the condition is false a diagnostic message is added to the log, the logged data is visualized and  
 *  the program is halted.
 *   
 * @{
 */

#ifndef ASSERT_ON_TEST_HPP
#define ASSERT_ON_TEST_HPP

#ifndef TESTING //This will be active only on testing

	#define ABORT_PROGRAM() exit(EXIT_FAILURE);
	#define ASSERT_ON_TEST(condition, message)
	#define DEBUG_WRITE_TO_LOG(message, value)
	#define DEBUG_PRINT_LOG()
	#define DEBUG_PRINT_TO_LOG(message, value) 
	#define DEBUG_VERIFY(condition, message)
	#define DEBUG_NOTIFY_ON_EXCEPTION(expression, message)

#else

#include "Assert.hpp"

	#define ABORT_PROGRAM() throw AssertException();
	#define ASSERT_ON_TEST(condition, message) ASSERT(condition, message)
	#define DEBUG_WRITE_TO_LOG(message, value) WRITE_TO_LOG(message, value)
	#define DEBUG_PRINT_LOG() PRINT_LOG()
	#define DEBUG_PRINT_TO_LOG(message, value) PRINT_TO_LOG(message, value)
	#define DEBUG_VERIFY(condition, message) VERIFY(condition, message)
	#define DEBUG_NOTIFY_ON_EXCEPTION(expression, message) NOTIFY_ON_EXCEPTION(expression, message)


#endif
#endif

/* Assert.hpp */
/** @} */
