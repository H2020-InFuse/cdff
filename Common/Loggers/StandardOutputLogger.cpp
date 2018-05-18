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
	static const std::string RESET_COLOR_CODE = "\033[0m";
	for(std::vector<std::string>::iterator logEntry = logEntriesVector.begin(); logEntry != logEntriesVector.end(); logEntry++)
		{
		int logEntryIndex = logEntry - logEntriesVector.begin();
		std::string colorCode = MessageTypeToColorCode( logEntriesTypeVector.at(logEntryIndex) );

		std::cout << colorCode << (*logEntry);
		if (colorCode != "")
			{
			std::cout << RESET_COLOR_CODE;	
			}
		std::cout << std::endl;
		}	
	}

<<<<<<< HEAD
void StandardOutputLogger::SetColorRed()
	{
	std::cout<<"\033[1;31m";
	}

void StandardOutputLogger::SetColorNormal()
	{
	std::cout<<"\033[0m";
=======
/* --------------------------------------------------------------------------
 *
 * Private Member Functions
 *
 * --------------------------------------------------------------------------
 */
std::string StandardOutputLogger::MessageTypeToColorCode(MessageType messageType)
	{
	static const std::string RED_COLOR_CODE = "\033[1;31m";
	static const std::string GREEN_COLOR_CODE = "\033[1;32m";
	static const std::string YELLOW_COLOR_CODE = "\033[1;33m";
	
	switch(messageType)
		{
		case ERROR: 
			{
			return RED_COLOR_CODE;
			}
		case WARNING:
			{
			return YELLOW_COLOR_CODE;
			}
		case SUCCESS:
			{
			return GREEN_COLOR_CODE;
			}
		}
	return "";
>>>>>>> origin/master
	}

/** @} */
