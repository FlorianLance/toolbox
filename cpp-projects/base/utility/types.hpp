
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
#include <cstddef>
#include <cstdint>
#include <variant>
#include <cmath>


// local
#include "utility/vector.hpp"
#include "utility/math.hpp"

namespace tool {

// ############################################################################### Aliases
using cbool     = const bool;       /**< const bool alias */
using cint      = const int;        /**< const int alias */
using cfloat    = const float;      /**< const float alias */
using cdouble   = const double;     /**< const double alias */
using uint      = unsigned int;     /**< unsigned int alias */
using cuint     = const uint ;      /**< const unsigned int alias */
using csize_t   = const std::size_t;/**< const size_t alias */
using byte      = std::uint8_t;

// ############################################################################### Names types
// # ids
//struct GlobalId    {int v;};    // unique id > 0
//struct Id          {int v;};    // id > 0
// # time
struct Seconds     {double v;}; // seconds
struct Milliseconds{double v;}; // milliseconds
struct SecondsTS     {double v;}; // seconds timestamp
struct MillisecondsTS{double v;}; // milliseconds timestamp
struct Ratio       {double v;}; // between 0 and 1
// # ui
struct WidthPix    {int v;}; // width in pixels
struct HeightPix   {int v;}; // height in pixels
struct Width       {double v;};
struct Height      {double v;};

struct Index       {int v;};


template <typename T>
constexpr static size_t to_unsigned(T v) {
    return static_cast<size_t>(v);
}

template <typename T>
constexpr static int to_signed(T v) {
    return static_cast<int>(v);
}

constexpr static size_t id(int id){
    return static_cast<size_t>(id);
}

constexpr static int qt_id(size_t id){
    return static_cast<int>(id);
}


struct Decimal;
using DecimalList   = std_v1<Decimal>;
using DoubleList    = std_v1<double>;
using RealList      = DoubleList;

struct VoidV{};
struct UndefinedV{};

struct Decimal{

    using DecimalV      = std::variant<int,float,double>;

    Decimal() : v(0){}
    explicit Decimal(bool value)    : v(value ? 1 : 0){}
    explicit Decimal(int value)     : v(value){}
    explicit Decimal(float value)   : v(value){}
    explicit Decimal(double value)  : v(value){}

    constexpr bool is_int() const {return std::holds_alternative<int>(v);}
    constexpr bool is_float() const{return std::holds_alternative<float>(v);}
    constexpr bool is_double() const{return std::holds_alternative<double>(v);}

    constexpr bool is_null() const{
        return almost_equal(to_double(), 0.0);
    }

    constexpr int to_int() const{
        if(is_int()){
            return std::get<int>(v);
        }else if(is_float()){
            return static_cast<int>(std::get<float>(v));
        }
        return static_cast<int>(std::get<double>(v));
    }

    constexpr float to_float() const{

        if(is_int()){
            return static_cast<float>(std::get<int>(v));
        }else if(is_float()){
            return std::get<float>(v);
        }
        return static_cast<float>(std::get<double>(v));
    }

    constexpr double to_double() const{

        if(is_int()){
            return static_cast<double>(std::get<int>(v));
        }else if(is_float()){
            return static_cast<double>(std::get<float>(v));
        }
        return std::get<double>(v);
    }

    static Decimal add(Decimal d1, Decimal d2){

        if(d1.is_int()){

            if(d2.is_int()){
                return Decimal(d1.to_int() + d2.to_int());
            }else if(d2.is_float()){
                return Decimal(d1.to_float() + d2.to_float());
            }
            return Decimal(d1.to_double() + d2.to_double());
        }else if(d1.is_float()){

            if(d2.is_double()){
                return Decimal(d1.to_double() + d2.to_double());
            }
            return Decimal(d1.to_float() + d2.to_float());
        }
        return Decimal(d1.to_double() + d2.to_double());
    }

    static Decimal sub(Decimal d1, Decimal d2){

        if(d1.is_int()){

            if(d2.is_int()){
                return Decimal(d1.to_int() - d2.to_int());
            }else if(d2.is_float()){
                return Decimal(d1.to_float() - d2.to_float());
            }
            return Decimal(d1.to_double() - d2.to_double());
        }else if(d1.is_float()){

            if(d2.is_double()){
                return Decimal(d1.to_double() - d2.to_double());
            }
            return Decimal(d1.to_float() - d2.to_float());
        }
        return Decimal(d1.to_double() - d2.to_double());
    }

    static Decimal mul(Decimal d1, Decimal d2){

        if(d1.is_int()){

            if(d2.is_int()){
                return Decimal(d1.to_int() * d2.to_int());
            }else if(d2.is_float()){
                return Decimal(d1.to_float() * d2.to_float());
            }
            return Decimal(d1.to_double() * d2.to_double());
        }else if(d1.is_float()){

            if(d2.is_double()){
                return Decimal(d1.to_double() * d2.to_double());
            }
            return Decimal(d1.to_float() * d2.to_float());
        }
        return Decimal(d1.to_double() * d2.to_double());
    }

    static Decimal div(Decimal d1, Decimal d2){

        if(d1.is_int()){

            if(d2.is_int()){
                return Decimal(d1.to_int() / d2.to_int());
            }else if(d2.is_float()){
                return Decimal(d1.to_float() / d2.to_float());
            }
            return Decimal(d1.to_double() / d2.to_double());
        }else if(d1.is_float()){

            if(d2.is_double()){
                return Decimal(d1.to_double() / d2.to_double());
            }
            return Decimal(d1.to_float() / d2.to_float());
        }
        return Decimal(d1.to_double() / d2.to_double());
    }

    static Decimal mod(Decimal d1, Decimal d2){

        if(d1.is_int()){

            if(d2.is_int()){
                return Decimal(d1.to_int() % d2.to_int());
            }else if(d2.is_float()){
                return Decimal(std::fmodf(d1.to_float(), d2.to_float()));
            }
            return Decimal(std::fmod(d1.to_double(), d2.to_double()));
        }else if(d1.is_float()){

            if(d2.is_double()){
                return Decimal(std::fmod(d1.to_double(), d2.to_double()));
            }
            return Decimal(std::fmodf(d1.to_float(), d2.to_float()));
        }
        return Decimal(std::fmod(d1.to_double(),d2.to_double()));
    }

    static Decimal cos(Decimal d){
        if(d.is_int()){
            return Decimal(std::cos(d.to_int()));
        }else if(d.is_float()){
            return Decimal(std::cosf(d.to_float()));
        }
        return Decimal(std::cos(d.to_double()));
    }

    static Decimal sin(Decimal d){
        if(d.is_int()){
            return Decimal(std::sin(d.to_int()));
        }else if(d.is_float()){
            return Decimal(std::sinf(d.to_float()));
        }
        return Decimal(std::sin(d.to_double()));
    }

    static Decimal tan(Decimal d){
        if(d.is_int()){
            return Decimal(std::tan(d.to_int()));
        }else if(d.is_float()){
            return Decimal(std::tanf(d.to_float()));
        }
        return Decimal(std::tan(d.to_double()));
    }

    static Decimal acos(Decimal d){
        if(d.is_int()){
            return Decimal(std::acos(d.to_int()));
        }else if(d.is_float()){
            return Decimal(std::acosf(d.to_float()));
        }
        return Decimal(std::acos(d.to_double()));
    }

    static Decimal asin(Decimal d){
        if(d.is_int()){
            return Decimal(std::asin(d.to_int()));
        }else if(d.is_float()){
            return Decimal(std::asinf(d.to_float()));
        }
        return Decimal(std::asin(d.to_double()));
    }

    static Decimal atan(Decimal d){
        if(d.is_int()){
            return Decimal(std::atan(d.to_int()));
        }else if(d.is_float()){
            return Decimal(std::atanf(d.to_float()));
        }
        return Decimal(std::atan(d.to_double()));
    }

    static Decimal cosh(Decimal d){
        if(d.is_int()){
            return Decimal(std::cosh(d.to_int()));
        }else if(d.is_float()){
            return Decimal(std::coshf(d.to_float()));
        }
        return Decimal(std::cosh(d.to_double()));
    }

    static Decimal sinh(Decimal d){
        if(d.is_int()){
            return Decimal(std::sinh(d.to_int()));
        }else if(d.is_float()){
            return Decimal(std::sinhf(d.to_float()));
        }
        return Decimal(std::sinh(d.to_double()));
    }

    static Decimal tanh(Decimal d){
        if(d.is_int()){
            return Decimal(std::tanh(d.to_int()));
        }else if(d.is_float()){
            return Decimal(std::tanhf(d.to_float()));
        }
        return Decimal(std::tanh(d.to_double()));
    }

    static Decimal acosh(Decimal d){
        if(d.is_int()){
            return Decimal(std::acosh(d.to_int()));
        }else if(d.is_float()){
            return Decimal(std::acoshf(d.to_float()));
        }
        return Decimal(std::acosh(d.to_double()));
    }

    static Decimal asinh(Decimal d){
        if(d.is_int()){
            return Decimal(std::asinh(d.to_int()));
        }else if(d.is_float()){
            return Decimal(std::asinhf(d.to_float()));
        }
        return Decimal(std::asinh(d.to_double()));
    }

    static Decimal atanh(Decimal d){
        if(d.is_int()){
            return Decimal(std::atanh(d.to_int()));
        }else if(d.is_float()){
            return Decimal(std::atanhf(d.to_float()));
        }
        return Decimal(std::atanh(d.to_double()));
    }

    static bool inferior(Decimal d1, Decimal d2){
        return d1.to_double() < d2.to_double();
    }

    static bool superior(Decimal d1, Decimal d2){
        return d1.to_double() > d2.to_double();
    }

    static bool inferior_or_equal(Decimal d1, Decimal d2){

        if(almost_equal<double>(d1.to_double(),d2.to_double())){
            return true;
        }
        return inferior(d1,d2);
    }

    static bool superior_or_equal(Decimal d1, Decimal d2){

        if(almost_equal<double>(d1.to_double(),d2.to_double())){
            return true;
        }
        return superior(d1,d2);
    }

    static bool equal(Decimal d1, Decimal d2){
        return almost_equal<double>(d1.to_double(),d2.to_double());
    }

    static bool no_equal(Decimal d1, Decimal d2){
        return !almost_equal<double>(d1.to_double(),d2.to_double());
    }


    DecimalV v;
};





}
