/* --------------------------------------------------------------------------
*
* (C) Copyright …
*
* --------------------------------------------------------------------------
*/

/*!
 * @file Logger.hpp
 * @date 15/11/2017
 * @author Alessandro Bianco
 */

/*!
 * @addtogroup TestCommon
 * 
 *  A logger has the purpose to store string messages and visualize it; Logger is an abstract class, it does not offer any visualization function, which needs to be implemented in a dreived class. 
 * 
 * @{
 */

#ifndef LOGGER_HPP
#define LOGGER_HPP

/* --------------------------------------------------------------------------
 *
 * Includes
 *
 * --------------------------------------------------------------------------
 */
#include <stdlib.h>
#include <vector>
#include <string>


/* --------------------------------------------------------------------------
 *
 * Class definition
 *
 * --------------------------------------------------------------------------
 */
class Logger
	{
	/* --------------------------------------------------------------------
	 * Public
	 * --------------------------------------------------------------------
	 */
	public:
		enum MessageType
			{
			DIAGNOSTIC,
			WARNING,
			ERROR,
			SUCCESS
			};

		void AddEntry(std::string logEntry, MessageType messageType = DIAGNOSTIC);
		void Clear();

		virtual void Print() = 0;
		//virtual void SetColorRed() = 0;
		//virtual void SetColorNormal() = 0;

	/* --------------------------------------------------------------------
	 * Protected
	 * --------------------------------------------------------------------
	 */
	protected:
		std::vector<std::string> logEntriesVector;
		std::vector<MessageType> logEntriesTypeVector;

		Logger();
		~Logger();

	/* --------------------------------------------------------------------
	 * Private
	 * --------------------------------------------------------------------
	 */
	private:

	};

#endif
/* Logger.hpp */
/** @} */

