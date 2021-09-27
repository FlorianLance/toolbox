

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

#include "benchmark.hpp"

// std
#include <chrono>
#include <iostream>
#include <stack>
#include <unordered_map>
#include <unordered_set>
//#include <thread>

using namespace tool;
using namespace std::chrono;

struct Bench::Impl{

    struct TimesInfo{
        size_t callsCount = 0;
        bool started = false;
        int level = 0;
        std::vector<std::chrono::steady_clock::time_point> startTime;
        std::vector<std::chrono::steady_clock::time_point> stopTime;
    };

    static inline bool displayEnabled = true;
    static inline std::unordered_map<BenchId,TimesInfo> times = {};
    static inline std::stack<BenchId> stack  = {};
    static inline std::vector<BenchId> order  = {};
    static inline int currentLevel = 0;
    static inline std::vector<std::unique_ptr<std::string>> idStrs;
    static inline std::unordered_set<std::string_view> idStrsView;

    // static inline std::thread::id initThreadId = std::this_thread::get_id();

    // TODO thread id (make if thread-safe)
//    struct I{
//    static inline std::unordered_map<BenchId,TimesInfo> times = {};
//    static inline std::stack<BenchId> stack  = {};
//    static inline std::vector<BenchId> order  = {};
//    static inline int currentLevel = 0;
//    static inline std::vector<std::unique_ptr<std::string>> idStrs;
//    static inline std::unordered_set<std::string_view> idStrsView;
//    };

//    std::unordered_map<std::thread::id, I> tData;
//    static inline std::mutex lock;

    Impl(){}
};

Bench::Bench() : m_p(std::make_unique<Impl>()){

}

void Bench::disable_display(){
    Bench::Impl::displayEnabled = false;
}

void Bench::reset(){

    Bench::Impl::times.clear();
    Bench::Impl::stack = std::stack<std::string_view>();
    Bench::Impl::order.clear();
    Bench::Impl::currentLevel = 0;
}

void Bench::check(){

    for(auto &id : Bench::Impl::order){
        const auto &t = Bench::Impl::times[id];
        if(t.startTime.size() != t.stopTime.size()){
            std::cerr << std::format("Bench::Error: Id [{}] has not been stopped, (started:{}, stopped:{}).\n",
                id, t.startTime.size(), t.stopTime.size()
            );
        }
    }
}

void Bench::start(std::string_view id, bool display){

//    if(auto idT = std::this_thread::get_id(); idT != Impl::initThreadId){
//        std::stringstream ss;
//        ss << idT;
//         std::cerr << std::format("[Bench::Start call with id [{}] from another thread {}]\n", id, ss.str());
//    }

    using namespace std::chrono;
    if(id.size() == 0 && Impl::displayEnabled){
        std::cerr << std::format("Bench::Error: empty id\n");
        return;
    }

    if(display && Impl::displayEnabled){
        std::cout << std::format("[Bench::Start{}]\n", id);
    }

    auto idV = Bench::Impl::idStrsView.find(id);
    if (idV == Bench::Impl::idStrsView.end()) {
        auto str = std::make_unique<std::string>(id);
        Impl::idStrsView.insert(*str);
        Impl::idStrs.emplace_back(std::move(str));
        idV = Bench::Impl::idStrsView.find(id);
    }

    bool exists = Bench::Impl::times.count(*idV) != 0;
    if(!exists){
        Bench::Impl::order.emplace_back(*idV);
        Bench::Impl::times[*idV] = {};
    }

    // check if already running
    if(Bench::Impl::times[*idV].started){
        if(Impl::displayEnabled){
            std::cerr << std::format("Error with id {}, already started\n", id);
        }
        return;
    }

    Bench::Impl::times[*idV].started = true;
    Bench::Impl::times[*idV].callsCount++;
    Bench::Impl::times[*idV].startTime.emplace_back(high_resolution_clock::now());
    Bench::Impl::times[*idV].level = Bench::Impl::currentLevel;
    Bench::Impl::stack.emplace(*idV);
    ++Bench::Impl::currentLevel;
}

void Bench::stop(std::string_view id){

    using namespace std::chrono;
    if(Bench::Impl::stack.size() == 0){
        return;
    }

    if(id.length() == 0){
        id = Bench::Impl::stack.top();
        Bench::Impl::stack.pop();
    }

    if(Bench::Impl::times.count(id) == 0){
        if(Bench::Impl::displayEnabled){
            std::cerr << std::format("Bench::Error: cannot stop id [{}] \n.", id);
        }
        return;
    }

    Bench::Impl::times[id].stopTime.emplace_back(high_resolution_clock::now());
    Bench::Impl::times[id].started = false;

    if(Bench::Impl::currentLevel > 0){
        --Bench::Impl::currentLevel;
    }else{
        if(Bench::Impl::displayEnabled){
            std::cerr << "Bench::Error: Invalid level.\n";
        }
        Bench::Impl::currentLevel = 0;
    }
}

void Bench::display(BenchUnit unit, int64_t minTime, bool sort){
    if(Bench::Impl::displayEnabled){
        std::cout << to_string(unit, minTime, sort);
    }
}

constexpr std::string_view Bench::unit_to_str(BenchUnit unit){
    switch (unit) {
    case BenchUnit::milliseconds:
        return milliUnitStr;
    case BenchUnit::microseconds:
        return microUnitStr;
    case BenchUnit::nanoseconds:
        return nanoUnitStr;
    }
}

std::int64_t Bench::compute_total_time(BenchUnit unit, std::string_view id){

    const auto &t = Bench::Impl::times[id];

    std::int64_t total = 0;
    for(size_t ii = 0; ii < std::min(t.startTime.size(),t.stopTime.size()); ++ii){
        total += std::chrono::duration_cast<std::chrono::nanoseconds>(t.stopTime[ii]-t.startTime[ii]).count();
    }

    nanoseconds totalNs(total);
    std::int64_t time;
    switch (unit) {
    case BenchUnit::milliseconds:
        time = std::chrono::duration_cast<std::chrono::milliseconds>(totalNs).count();
        break;
    case BenchUnit::microseconds:
        time = std::chrono::duration_cast<std::chrono::microseconds>(totalNs).count();
        break;
    case BenchUnit::nanoseconds:
        time = totalNs.count();
        break;
    }

    return time;
}

std::vector<std::tuple<std::string_view, int64_t, size_t>> Bench::all_total_times(BenchUnit unit, std::int64_t minTime, bool sort){

    std::vector<std::tuple<std::string_view, int64_t, size_t>> times;
    times.reserve(Bench::Impl::order.size());
    for(auto &id : Bench::Impl::order){
        std::int64_t time = compute_total_time(unit, id);        
        if(time > minTime){
            times.emplace_back(id, time, Bench::Impl::times[id].callsCount);
        }
    }

    // sort by time
    auto sortInfo = [](const std::tuple<std::string_view, int64_t, size_t> &lhs, const std::tuple<std::string_view, int64_t, size_t> &rhs){
        return std::get<1>(lhs) < std::get<1>(rhs);
    };
    if(sort){
        std::sort(std::begin(times), std::end(times), sortInfo);
    }

    return times;
}

std::string Bench::to_string(BenchUnit unit, int64_t minTime, bool sort){

    using namespace std::chrono;

    auto totalTimes = all_total_times(unit, minTime, sort);
    std::ostringstream flux;
    if(totalTimes.size() > 0){
        flux << "\n[BENCH START] ############### \n";
        for(const auto &totalTime : totalTimes){
            auto id = std::get<0>(totalTime);
            const auto &timeI = Bench::Impl::times[id];
            flux << std::format("[ID:{}][L:{}][T:{} {}][C:{}][U:{} {}]\n",
                id,
                timeI.level,
                std::get<1>(totalTime),
                unit_to_str(unit),
                timeI.callsCount,
                (std::get<1>(totalTime)/timeI.callsCount),
                unit_to_str(unit)
            );
        }
        flux << "[BENCH END] ############### \n\n";
    }
    return flux.str();
}

bool Bench::is_started(BenchId id){
    return Bench::Impl::times[id].started;
}

int Bench::level(BenchId id){
    return Bench::Impl::times[id].level;
}

size_t Bench::calls_count(BenchId id){
    return Bench::Impl::times[id].callsCount;
}

