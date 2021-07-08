
/*******************************************************************************
** Toolbox-qt-utility                                                         **
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

// base
#include "utility/tuple_array.hpp"

namespace tool::ex {
	
	using namespace std::literals::string_view_literals;
	
	enum class UnityType : int {
        System_boolean,
        System_sbyte, System_int16, System_int32, System_int64, System_decimal,
        System_byte, System_uint16, System_uint32, System_uint64,
        System_single, System_double,
        System_char, System_string,
        None, Mixed, Unknow, SizeEnum};
	
    using UniT = UnityType;
    using Name    = std::string_view;
    using StrType = std::string_view;

    using TUnityType = std::tuple<UnityType, Name, StrType>;
    static constexpr TupleArray<UnityType::SizeEnum, TUnityType> unityTypes {{
        TUnityType
        {UnityType::System_boolean,    "bool"sv,       "System.Boolean"sv },
        {UnityType::System_single,     "float"sv,      "System.Single"sv  },
        {UnityType::System_double,     "double"sv,     "System.Double"sv  },
        {UnityType::System_sbyte,      "sbyte"sv,      "System.Sbyte"sv   },
        {UnityType::System_int16,      "short"sv,      "System.In16"sv    },
        {UnityType::System_int32,      "int"sv,        "System.Int32"sv   },
        {UnityType::System_int64,      "long"sv,       "System.Int64"sv   },
        {UnityType::System_decimal,    "decimal"sv,    "System.Decimal"sv },
        {UnityType::System_byte,       "byte"sv,       "System.Byte"sv    },
        {UnityType::System_uint16,     "ushort"sv,     "System.UIn16"sv   },
        {UnityType::System_uint32,     "uint"sv,       "System.UInt32"sv  },
        {UnityType::System_uint64,     "ulong"sv,      "System.UInt64"sv  },
        {UnityType::System_char,       "char"sv,       "System.Char"sv    },
        {UnityType::System_string,     "string"sv,     "System.String"sv  },
        {UnityType::None,              "none"sv,       "None"sv           },
        {UnityType::Unknow,            "unknow"sv,     "Unknow"sv         },
        {UnityType::Mixed,             "mixed types"sv,"None"sv           },
    }};
	
	constexpr StrType get_unity_type_string(UnityType t) {
        return unityTypes.at<0,2>(t);
    }

    constexpr std::optional<UnityType> get_unity_type(StrType strType){
        return unityTypes.optional_at<2,0>(strType);
    }
}
