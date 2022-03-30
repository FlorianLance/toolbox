

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

#include "ex_component.hpp"

using namespace tool::ex;

bool ExComponent::is_visible(int cKey){
    return (*isVisibleCBP)(cKey);
}

bool ExComponent::is_updating(int cKey){
    return (*isUpdatingCBP)(cKey);
}

bool ExComponent::is_closed(int cKey){
    return (*isClosedCBP)(cKey);
}

long ExComponent::ellapsed_time_exp_ms(){
    return (*ellapsedTimeExpMsCBP)();
}

long ExComponent::ellapsed_time_routine_ms(){
    return (*ellapsedTimeRoutineMsCBP)();
}

void ExComponent::signal_bool(int index, bool value){
    (*signalBoolCBP)(key(), index, value ? 1 : 0);
}

void ExComponent::signal_int(int index, int value){
    (*signalIntCBP)(key(), index, value);
}

void ExComponent::signal_float(int index, float value){
    (*signalFloatCBP)(key(), index, value);
}

void ExComponent::signal_double(int index, double value){
    (*signalDoubleCBP)(key(), index, value);
}

void ExComponent::signal_string(int index, std::string value){
    (*signalStringCBP)(key(), index, value.c_str());
}

