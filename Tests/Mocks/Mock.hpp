/* --------------------------------------------------------------------------
*
* (C) Copyright …
*
* ---------------------------------------------------------------------------
*/

/*!
 * @file Mock.hpp
 * @date 21/11/2017
 * @author Alessandro Bianco
 */

/*!
 * @addtogroup Mocks
 * 
 * This is the main mocking class, it supports the programming of ad hoc behaviours. Every Mock should inherit from this class.
 * The main idea is that when a mock is created:
 * 1) AddBehaviour(functionName, t, value) is used to set up the value of the t-th time the function is called, t is a string, hence also the value "Always" can be specified.
 * 2) GetBehaviour(functionName, t) is used to take the value that should be the t-th output of the function
 * 3) A static version of the above is used to mock static methods
 * 4) When a class is derived from mock, it should also derive from the class you want to fake. All mocked methods should be overriden and they should use GetBehaviour to look for the mocked output,
 *    if no mocked output is found, the original class method should be called.
 *
 * @{
 */

#ifndef MOCK_HPP
#define MOCK_HPP


/* --------------------------------------------------------------------------
 *
 * Includes
 *
 * --------------------------------------------------------------------------
 */
#include <stdlib.h>
#include <sstream>
#include <string>
#include <map>

namespace Mocks {

/* --------------------------------------------------------------------------
 *
 * Class definition
 *
 * --------------------------------------------------------------------------
 */
class Mock
	{
	/* --------------------------------------------------------------------
	 * Public
	 * --------------------------------------------------------------------
	 */
	public:	
		void AddBehaviour(const std::string& functionName, const std::string& time, void* value);
		void* GetBehaviour(const std::string& functionName, unsigned time);
		static void AddStaticBehaviour(const std::string& functionName, const std::string& time, void* value);
		static void* GetStaticBehaviour(const std::string& functionName, unsigned time);

	/* --------------------------------------------------------------------
	 * Protected
	 * --------------------------------------------------------------------
	 */
        protected:
		std::map<std::string, unsigned> countersMap;
		static std::map<std::string, unsigned> staticCountersMap;
	/* --------------------------------------------------------------------
	 * Private
	 * --------------------------------------------------------------------
	 */
	private:
		std::map<std::string, std::map<std::string, void*> > behavioursMap;
		static std::map<std::string, std::map<std::string, void*> > staticBehavioursMap;


		static void AddBehaviour(std::map<std::string, std::map<std::string, void*> >& map, const std::string& functionName, const std::string& time, void* value);	
		static void* GetBehaviour(std::map<std::string, std::map<std::string, void*> >& map, const std::string& functionName, unsigned time);
	};


}
#endif

/* Mock.hpp */
/** @} */
