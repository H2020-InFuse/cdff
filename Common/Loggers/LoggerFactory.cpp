/* --------------------------------------------------------------------------
*
* (C) Copyright …
*
* ---------------------------------------------------------------------------
*/

/*!
 * @file LoggerFactory.cpp
 * @date 15/11/2017
 * @author Alessandro Bianco
 */

/*!
 * @addtogroup TestCommon
 * 
 * Implementation of the LoggerFactory class
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
#include "LoggerFactory.hpp"
#include "StandardOutputLogger.hpp"


/* --------------------------------------------------------------------------
 *
 * Public Member Functions
 *
 * --------------------------------------------------------------------------
 */

Logger* LoggerFactory::GetLogger()
	{
	if (logger == NULL)
		logger = CreateLogger();
	return logger;
	}

void LoggerFactory::SetLoggerType(LoggerType loggerType)
	{
	LoggerFactory::loggerType = loggerType;
	}


/* --------------------------------------------------------------------------
 *
 * Private Member Variables 
 *
 * --------------------------------------------------------------------------
 */

Logger* LoggerFactory::logger = NULL;
LoggerFactory::LoggerType LoggerFactory::loggerType = STANDARD_OUTPUT;


/* --------------------------------------------------------------------------
 *
 * Private Member Functions 
 *
 * --------------------------------------------------------------------------
 */

Logger* LoggerFactory :: CreateLogger()
	{
	switch(loggerType)
		{
		case STANDARD_OUTPUT: return new StandardOutputLogger();
		default: return new StandardOutputLogger();
		};
	}

/** @} */
