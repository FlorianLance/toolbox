
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

#include <atomic>

namespace tool {

/*
* An std::atomic_flag-based spin-lock.
* Meets the Lockable concept.
* Therefore, usable with std::lock_guard.
*/
class SpinLock{
    std::atomic_flag m_flag = ATOMIC_FLAG_INIT;
public:
    inline void lock()     noexcept {   while(m_flag.test_and_set(std::memory_order_acquire)); }
    inline void unlock()   noexcept {         m_flag.clear(std::memory_order_release);         }
    inline bool try_lock() noexcept { return !m_flag.test_and_set(std::memory_order_acquire);  }
};
}



//struct spin_mutex {
//    spin_mutex() noexcept = default;
//    ~spin_mutex() noexcept = default;
//    spin_mutex(spin_mutex const&) = delete;
//    spin_mutex& operator=(const spin_mutex &) = delete;
//    spin_mutex(spin_mutex &&) = delete;
//    spin_mutex& operator=(spin_mutex &&) = delete;

//    void lock() noexcept {
//        while (true) {
//            while (!state.load(std::memory_order_relaxed)) {
//                std::this_thread::yield();
//            }

//            if (try_lock()) {
//                break;
//            }
//        }
//    }

//    bool try_lock() noexcept {
//        return state.exchange(false, std::memory_order_acquire);
//    }

//    void unlock() noexcept {
//        state.store(true, std::memory_order_release);
//    }

//private:
//    std::atomic<bool> state {true};
//};

