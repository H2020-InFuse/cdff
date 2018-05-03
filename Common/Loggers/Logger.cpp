/* --------------------------------------------------------------------------
*
* (C) Copyright …
*
* ---------------------------------------------------------------------------
*/

/*!
 * @file Logger.cpp
 * @date 15/11/2017
 * @author Alessandro Bianco
 */

/*!
 * @addtogroup TestCommon
 * 
 * Implementation of the Logger class
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
#include "Logger.hpp"


/* --------------------------------------------------------------------------
 *
 * Public Member Functions
 *
 * --------------------------------------------------------------------------
 */

void Logger::AddEntry(std::string logEntry, MessageType messageType)
	{
	logEntriesVector.push_back(logEntry);
	logEntriesTypeVector.push_back(messageType);
	}

void Logger::Clear()
	{
	logEntriesVector.clear();
	logEntriesTypeVector.clear();
	}


/* --------------------------------------------------------------------------
 *
 * Protected Member Functions
 *
 * --------------------------------------------------------------------------
 */

Logger::Logger()
	{

	}

Logger::~Logger()
	{

	}

/** @} */

