//
// Created by tch on 25/09/18.
//

#ifndef CDFF_VALIDATORS_NUMBER_HPP
#define CDFF_VALIDATORS_NUMBER_HPP

namespace CDFF {
    namespace Validators {
        namespace Number {
            template<class NumberType>
            void IsGreaterThan(NumberType value, NumberType min_val) {
                ASSERT(value > min_val, "Validators/Number - Should be greater than specified value");
            }

            template <typename NumberType>
            void IsPositive(NumberType value) {
                ASSERT(value >= static_cast<NumberType>(0.), "Validators/Number - Should be positive");
            }

            template<typename NumberType>
            void IsOdd(NumberType value) {
                ASSERT(value % 2 != 0, "Validators/Number - Should be odd");
            }
        }
    }
}

#endif // CDFF_VALIDATORS_NUMBER_HPP
