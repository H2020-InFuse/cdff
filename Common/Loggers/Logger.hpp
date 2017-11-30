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
		void AddEntry(std::string logEntry);
		void Clear();

		virtual void Print() = 0;

	/* --------------------------------------------------------------------
	 * Protected
	 * --------------------------------------------------------------------
	 */
	protected:
		std::vector<std::string> logEntriesVector;

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

