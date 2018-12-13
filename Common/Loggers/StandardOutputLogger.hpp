/* --------------------------------------------------------------------------
*
* (C) Copyright …
*
* --------------------------------------------------------------------------
*/

/*!
 * @file StandardOutputLogger.hpp
 * @date 15/11/2017
 * @author Alessandro Bianco
 */

/*!
 * @addtogroup TestCommon
 * 
 *  The standard output loggers manage the storage of diagnostic string messages and offers methods for printing the messages on the standard output. 
 * 
 * @{
 */

#ifndef STANDARD_OUTPUT_LOGGER_HPP
#define STANDARD_OUTPUT_LOGGER_HPP


/* --------------------------------------------------------------------------
 *
 * Includes
 *
 * --------------------------------------------------------------------------
 */
#include "Logger.hpp"
#include <stdlib.h>
#include <iostream>


/* --------------------------------------------------------------------------
 *
 * Class definition
 *
 * --------------------------------------------------------------------------
 */
class StandardOutputLogger : public Logger
	{
	/* --------------------------------------------------------------------
	 * Public
	 * --------------------------------------------------------------------
	 */
	public:
		StandardOutputLogger();
		~StandardOutputLogger();

		void Print() override;

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
		std::string MessageTypeToColorCode(MessageType messageType);

	};

#endif
/* StandardOutputLogger.hpp */
/** @} */
