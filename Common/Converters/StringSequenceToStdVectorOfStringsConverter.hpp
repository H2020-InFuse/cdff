/* --------------------------------------------------------------------------
*
* (C) Copyright …
*
* --------------------------------------------------------------------------
*/

/*!
 * @file StringSequenceToStdVectorOfStringsConverter.hpp
 * @date 12/09/2018
 * @author Irene Sanz
 */

/*!
 * @addtogroup Converters
 * 
 *  This is the class for type conversion from StringSequence to std::vector<std::string>.
 *  
 *
 * @{
 */

#ifndef STRINGSEQUENCE_TO_VECTOROFSTRINGS_CONVERTER_HPP
#define STRINGSEQUENCE_TO_VECTOROFSTRINGS_CONVERTER_HPP


#include <Sequences.h>
#include <vector>
#include <string>

namespace Converters
{
    class StringSequenceToStdVectorOfStringsConverter
	{
        public:
            virtual const std::vector<std::string> Convert(const asn1SccStringSequence& stringSequence);
	};
}

#endif //STRINGSEQUENCE_TO_VECTOROFSTRINGS_CONVERTER_HPP

/** @} */