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
 * Implementation of the StandardOutputLogger class
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
#include "StandardOutputLogger.hpp"


/* --------------------------------------------------------------------------
 *
 * Public Member Functions
 *
 * --------------------------------------------------------------------------
 */

StandardOutputLogger::StandardOutputLogger()
	{

	}

StandardOutputLogger::~StandardOutputLogger()
	{

	}
		
void StandardOutputLogger::Print()
	{
	for(std::vector<std::string>::iterator logEntry = logEntriesVector.begin(); logEntry != logEntriesVector.end(); logEntry++)
		{
		std::cout<<(*logEntry)<<std::endl;
		}	
	}

/** @} */
