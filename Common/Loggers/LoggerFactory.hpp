/* --------------------------------------------------------------------------
*
* (C) Copyright …
*
* --------------------------------------------------------------------------
*/

/*!
 * @file LoggerFactory.hpp
 * @date 15/11/2017
 * @author Alessandro Bianco
 */

/*!
 * @addtogroup TestCommon
 * 
 *  The logger factory provides one singleton Logger to the whole application. The type of the Logger is specified in advance before the first Logger access through the SetLoggerType.
 *  After the first logger access, the logger type cannot be changed for the duration of the application. 
 * 
 * @{
 */

#ifndef LOGGER_FACTORY_HPP
#define LOGGER_FACTORY_HPP


/* --------------------------------------------------------------------------
 *
 * Includes
 *
 * --------------------------------------------------------------------------
 */
#include "Logger.hpp"


/* --------------------------------------------------------------------------
 *
 * Class definition
 *
 * --------------------------------------------------------------------------
 */
class LoggerFactory
	{
	/* --------------------------------------------------------------------
	 * Public
	 * --------------------------------------------------------------------
	 */
	public:
		enum LoggerType
			{
			STANDARD_OUTPUT
			};

		static Logger* GetLogger();
		static void SetLoggerType(LoggerType loggerType);

	/* --------------------------------------------------------------------
	 * Protected
	 * --------------------------------------------------------------------
	 */
	protected:


	/* --------------------------------------------------------------------
	 * Private
	 * --------------------------------------------------------------------
	 */
	private:
		static Logger* logger;
		static LoggerType loggerType;
		static Logger* CreateLogger();

	};

#endif
/* LoggerFactory.hpp */
/** @} */
