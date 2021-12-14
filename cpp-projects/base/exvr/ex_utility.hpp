

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
#include <array>

typedef void (__stdcall * StackTraceCB)(const char*);
typedef void (__stdcall * LogCB)(const char*);
typedef void (__stdcall * LogWarningCB)(const char*);
typedef void (__stdcall * LogErrorCB)(const char*);
typedef long (__stdcall * EllapsedTimeExpMsCB)();
typedef long (__stdcall * EllapsedTimeRoutineMsCB)();
typedef int (__stdcall * GetCB)(const char*);
typedef int (__stdcall * IsInitializedCB)(int);
typedef int (__stdcall * IsVisibleCB)(int);
typedef int (__stdcall * IsUpdatingCB)(int);
typedef int (__stdcall * IsClosedCB)(int);
typedef void (__stdcall * NextCB)();
typedef void (__stdcall * PreviousCB)();
typedef void (__stdcall * CloseCB)(int);
typedef void (__stdcall * SignalBoolCB)(int, int,int);
typedef void (__stdcall * SignalIntCB)(int, int,int);
typedef void (__stdcall * SignalFloatCB)(int, int,float);
typedef void (__stdcall * SignalDoubleCB)(int, int,double);
typedef void (__stdcall * SignalStringCB)(int, int,const char*);


namespace tool::ex {

    enum class ParametersContainer : int {
        InitConfig=0, CurrentConfig=1, Dynamic=2
    };


    static constexpr std::array<std::tuple<ParametersContainer, const char*>,static_cast<size_t>(3)> mapping ={{
        {ParametersContainer::InitConfig,       "init"},
        {ParametersContainer::CurrentConfig,    "current"},
        {ParametersContainer::Dynamic,          "dynamic"}
    }};
}
