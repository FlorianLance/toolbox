
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
#include <type_traits>


namespace tool {

template<class acc>
[[maybe_unused]] constexpr acc E             = acc{2.71828182845904523536};

template<class acc>
[[maybe_unused]] constexpr static acc LOG2E         = acc{1.44269504088896340736};

template<class acc>
[[maybe_unused]] constexpr acc LOG10E        = acc{0.434294481903251827651};

template<class acc>
[[maybe_unused]] constexpr acc LN2           = acc{0.693147180559945309417};

template<class acc>
[[maybe_unused]] constexpr acc LN10          = acc{2.30258509299404568402};

template<class acc>
[[maybe_unused]] constexpr acc PI            = acc{3.14159265358979323846};

template<class acc>
[[maybe_unused]] constexpr static acc two_PI        = acc{2}*PI<acc>;

template<class acc>
[[maybe_unused]] constexpr static acc PI_180        = PI<acc>/acc{180};

template<class acc>
[[maybe_unused]] constexpr acc d180_PI       = acc{180}/PI<acc>;

template<class acc>
[[maybe_unused]] constexpr acc PI_2          = acc{(PI<acc>)/2};

template<class acc>
[[maybe_unused]] constexpr acc PI_4          = acc{(PI<acc>)/4};

template<class acc>
[[maybe_unused]] constexpr acc SQRT2         = acc{1.41421356237309504880};

template<class acc>
[[maybe_unused]] constexpr acc SQRT3         = acc{1.732050807568877};

template<class acc>
[[maybe_unused]] constexpr acc SQRT1_2       = acc{0.707106781186547524401};

template<class T>
[[maybe_unused]] constexpr T EPSILON = T(std::is_same<float, T>::value ? 1.192092896e-07F : (std::is_same<double, T>::value ? 2.2204460492503131e-016 : 0));

}

