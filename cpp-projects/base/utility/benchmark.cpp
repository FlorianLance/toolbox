

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
#include <mutex>
#include <algorithm>

using namespace tool;
using namespace std::chrono;

struct TimesInfo{
    size_t callsCount = 0;
    bool started = false;
    int level = 0;
    std::vector<steady_clock::time_point> startTime;
    std::vector<steady_clock::time_point> stopTime;
};

struct I{
    std::unordered_map<BenchId,TimesInfo> times = {};
    std::vector<BenchId> stack = {};
    std::vector<BenchId> order  = {};
    int currentLevel = 0;
    std::vector<std::unique_ptr<std::string>> idStrs;
    std::unordered_set<std::string_view> idStrsView;
    std::thread::id tId;
    std::string tIdStr;
};

struct Bench::Impl{


    static inline std::atomic_bool displayEnabled = true;
    static inline std::unordered_map<std::thread::id, I> tData = {};
    static inline std::mutex lock;

    static void check_thread_id(std::thread::id tId){
        if(Impl::tData.count(tId)==0){
            {
                const std::lock_guard<std::mutex> g(Impl::lock);
                Impl::tData[tId] = {};
                Impl::tData[tId].tId = tId;
            }
            std::stringstream ss;
            ss << tId;
            Impl::tData[tId].tIdStr = ss.str();
            std::cout << std::format("[Bench::First start call from thread {}]\n", Impl::tData[tId].tIdStr);
        }
    }

    Impl(){}
};

Bench::Bench() : m_p(std::make_unique<Impl>()){
}

void Bench::disable_display(){
    Impl::displayEnabled = false;
}

void Bench::reset(){

    auto tId = std::this_thread::get_id();
    Impl::check_thread_id(tId);

    Impl::tData[tId].times.clear();
    Impl::tData[tId].stack.clear();
    Impl::tData[tId].order.clear();
    Impl::tData[tId].currentLevel = 0;
}

void Bench::check(){

    auto tId = std::this_thread::get_id();
    Impl::check_thread_id(tId);

    for(auto &id : Impl::tData[tId].order){
        const auto &t = Impl::tData[tId].times[id];
        if(t.startTime.size() != t.stopTime.size()){
            std::cerr << std::format("Bench::Error: Id [{}] has not been stopped, (started:{}, stopped:{}).\n",
                id, t.startTime.size(), t.stopTime.size()
            );
        }
    }
}

void Bench::start(std::string_view id, bool display){

    auto tId = std::this_thread::get_id();
    Impl::check_thread_id(tId);
    auto &d = Impl::tData[tId];


    if(id.size() == 0 && Impl::displayEnabled){
        std::cerr << std::format("Bench::Error: empty id\n");
        return;
    }

    if(display && Impl::displayEnabled){
        std::cout << std::format("[Bench::Start{}]\n", id);
    }

    auto idV = d.idStrsView.find(id);
    if (idV == d.idStrsView.end()) {
        auto str = std::make_unique<std::string>(id);
        d.idStrsView.insert(*str);
        d.idStrs.emplace_back(std::move(str));
        idV = d.idStrsView.find(id);
    }

    bool exists = d.times.count(*idV) != 0;
    if(!exists){
        d.order.emplace_back(*idV);
        d.times[*idV] = {};
    }

    // check if already running
    if(d.times[*idV].started){
        if(Impl::displayEnabled){
            std::cerr << std::format("Error with id {}, already started\n", id);
        }
        return;
    }

    d.times[*idV].started = true;
    d.times[*idV].callsCount++;
    d.times[*idV].startTime.emplace_back(high_resolution_clock::now());
    d.times[*idV].level = d.currentLevel;
    d.stack.push_back(*idV);
    ++d.currentLevel;
}

void Bench::stop(std::string_view id){

    auto tId = std::this_thread::get_id();
    Impl::check_thread_id(tId);
    auto &d = Impl::tData[tId];

    if(d.stack.size() == 0){
        return;
    }

    if(id.length() == 0){
        id = d.stack[d.stack.size()-1];
        d.stack.pop_back();
    }else{
        auto it = std::find(d.stack.begin(), d.stack.end(), id);
        if(it != d.stack.end()){
            d.stack.erase(std::remove(d.stack.begin(), d.stack.end(), id), d.stack.end());
        }else{
            std::cerr << std::format("Bench::Error: cannot stop id [{}], no element in stack. \n.", id);
            return;
        }
    }

    if(d.times.count(id) == 0){
        if(Impl::displayEnabled){
            std::cerr << std::format("Bench::Error: cannot stop id [{}] \n.", id);
        }
        return;
    }

    d.times[id].stopTime.emplace_back(high_resolution_clock::now());
    d.times[id].started = false;

    if(d.currentLevel > 0){
        --d.currentLevel;
    }else{
        if(Impl::displayEnabled){
            std::cerr << "Bench::Error: Invalid level.\n";
        }
        d.currentLevel = 0;
    }
}

void Bench::display(BenchUnit unit, int64_t minTime, bool sort){
    if(Impl::displayEnabled){
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

    auto tId = std::this_thread::get_id();
    Impl::check_thread_id(tId);

    const auto &t = Impl::tData[tId].times[id];

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

    auto tId = std::this_thread::get_id();
    Impl::check_thread_id(tId);

    std::vector<std::tuple<std::string_view, int64_t, size_t>> times;
    times.reserve(Impl::tData[tId].order.size());
    for(auto &id : Impl::tData[tId].order){
        std::int64_t time = compute_total_time(unit, id);        
        if(time > minTime){
            times.emplace_back(id, time, Impl::tData[tId].times[id].callsCount);
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

    auto tId = std::this_thread::get_id();
    Impl::check_thread_id(tId);

    auto totalTimes = all_total_times(unit, minTime, sort);
    std::ostringstream flux;
    if(totalTimes.size() > 0){
        flux << "\n[BENCH START] ############### \n";
        for(const auto &totalTime : totalTimes){
            auto id = std::get<0>(totalTime);
            const auto &timeI = Impl::tData[tId].times[id];
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

    auto tId = std::this_thread::get_id();
    Impl::check_thread_id(tId);

    return Impl::tData[tId].times[id].started;
}

int Bench::level(BenchId id){

    auto tId = std::this_thread::get_id();
    Impl::check_thread_id(tId);

    return Impl::tData[tId].times[id].level;
}

size_t Bench::calls_count(BenchId id){

    auto tId = std::this_thread::get_id();
    Impl::check_thread_id(tId);

    return Impl::tData[tId].times[id].callsCount;
}

