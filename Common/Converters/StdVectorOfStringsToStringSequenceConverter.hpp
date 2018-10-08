/* --------------------------------------------------------------------------
*
* (C) Copyright …
*
* --------------------------------------------------------------------------
*/

/*!
 * @file StdVectorOfStringsToStringSequenceConverter.hpp
 * @date 12/09/2018
 * @author Irene Sanz
 */

/*!
 * @addtogroup Converters
 * 
 *  This is the class for type conversion from std::vector<std::string> to StringSequence.
 *  
 *
 * @{
 */

#ifndef VECTOROFSTRINGS_TO_STRINGSEQUENCE_CONVERTER_HPP
#define VECTOROFSTRINGS_TO_STRINGSEQUENCE_CONVERTER_HPP

#include <Sequences.h>
#include <string>
#include <vector>

namespace Converters
{
    class StdVectorOfStringsToStringSequenceConverter
	{
        public:
            virtual const asn1SccStringSequence Convert(const std::vector<std::string> stringVector);
	};
}

#endif //VECTOROFSTRINGS_TO_STRINGSEQUENCE_CONVERTER_HPP

/** @} */