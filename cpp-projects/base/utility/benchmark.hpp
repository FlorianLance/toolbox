

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
#include <string_view>
#include <vector>
#include <memory>

namespace tool {

    using namespace std::literals::string_view_literals;
    using BenchId = std::string_view;

    enum class BenchUnit{
        milliseconds, microseconds, nanoseconds
    };

    constexpr static std::string_view milliUnitStr =  "ms"sv;
    constexpr static std::string_view microUnitStr =  "μs"sv;
    constexpr static std::string_view nanoUnitStr  =  "ns"sv;

    struct Bench{

        Bench();

        static void disable_display();

        static void reset();
        static void check();
        static void start(std::string_view id, bool display = false);
        static void stop(std::string_view id = ""sv);
        static void display(BenchUnit unit = BenchUnit::milliseconds, std::int64_t minTime = -1, bool sort = false);

        static constexpr std::string_view unit_to_str(BenchUnit unit);
        static std::string to_string(BenchUnit unit, std::int64_t minTime, bool sort);

        static bool is_started(BenchId id);
        static int level(BenchId id);
        static size_t calls_count(BenchId id);

        static std::int64_t compute_total_time(BenchUnit unit, BenchId id);
        static std::vector<std::pair<std::string_view, std::int64_t>> all_total_times(BenchUnit unit, std::int64_t minTime = -1, bool sort = false);

    private:

        struct Impl;
        std::unique_ptr<Impl> m_p = nullptr;
    };

    struct BenchGuard{

        std::string_view id;

        BenchGuard() = delete;
        BenchGuard(BenchId id, bool display = false) : id(id){
            Bench::start(id, display);
        }

        ~BenchGuard(){
            Bench::stop(id);
        }
    };
}
