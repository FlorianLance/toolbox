
/*******************************************************************************
** Toolbox-base                                                               **
** MIT License                                                                **
** Copyright (c) [2018] [Florian Lance]                                       **
**                                                                            **
** Permission is hereby granted, free of charge, to any person obtaining a    **
** copy of this software and associated documentation files (the "Software"), **
** to deal in the Software without restriction, including without limitation  **
** the rights to use, copy, modify, merge, publish, distribute, sublicense,   **
** and/or sell copies of the Software, and to permit persons to whom the      **
** Software is furnished to do so, subject to the following conditions:       **
**                                                                            **
** The above copyright notice and this permission notice shall be included in **
** all copies or substantial portions of the Software.                        **
**                                                                            **
** THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR **
** IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,   **
** FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL    **
** THE AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER **
** LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING    **
** FROM, OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER        **
** DEALINGS IN THE SOFTWARE.                                                  **
**                                                                            **
********************************************************************************/
#pragma once

// std
#include <limits>

// local
#include "utility/constants.hpp"

namespace tool {

template <typename acc>
constexpr acc ipow(acc num, unsigned int pow){
    return (pow >= sizeof(unsigned int)*8) ? 0 : pow == 0 ? 1 : num * ipow(num, pow-1);
}

// constexpr function not available in current std library
template<typename acc> constexpr acc abs(acc v){ return v<0?-v:v;}

template<class acc>
constexpr static acc deg_2_rad(const acc deg)noexcept {return deg*PI_180<acc>;}

template<class acc>
constexpr static acc rad_2_deg(const acc rad)noexcept {return rad*d180_PI<acc>;}

template<class acc>
constexpr typename std::enable_if<std::numeric_limits<acc>::is_integer, bool>::type almost_equal(acc x, acc y, int ulp = 1) noexcept{
    (void)ulp;
    return x == y;
}

template<class acc>
constexpr typename std::enable_if<!std::numeric_limits<acc>::is_integer, bool>::type almost_equal(acc x, acc y, int ulp = 1) noexcept{
    // the machine epsilon has to be scaled to the magnitude of the values used
    // and multiplied by the desired precision in ULPs (units in the last place)
    const acc diff = abs<acc>(x-y);
    return diff < std::numeric_limits<acc>::epsilon() * abs<acc>(x+y) * ulp
    // unless the result is subnormal
           || diff < std::numeric_limits<acc>::min();
}
}
