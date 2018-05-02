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

void StandardOutputLogger::SetColorRed()
	{
	std::cout<<"\033[1;31m";
	}

void StandardOutputLogger::SetColorNormal()
	{
	std::cout<<"\033[0m";
	}

/** @} */
