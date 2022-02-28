#pragma once
#include <chrono>
#include <atomic>

namespace jnet {
    template <class Clock = std::chrono::high_resolution_clock>
    class Stopwatch
    {
    public:
        Stopwatch(bool start = true);
        void Restart();
        void Stop();
        template <class Rep = float>
        Rep Time() const;
        bool IsRunning() const;
    private:
        typename Clock::time_point start;
        mutable typename Clock::time_point end;
        bool running;
    };

    template <class Clock>
    Stopwatch<Clock>::Stopwatch(bool start) :
        start(Clock::now()),
        end(Clock::now()),
        running(start) {
    }

    template <class Clock>
    void Stopwatch<Clock>::Restart() {
        start = Clock::now();
        running = true;
    }

    template <class Clock>
    void Stopwatch<Clock>::Stop() {
        end = Clock::now();
        running = false;
    }

    template <class Clock>
    template <class Rep>
    Rep Stopwatch<Clock>::Time() const {
        std::atomic_thread_fence(std::memory_order_relaxed);
        if (running)
            end = Clock::now();
        auto countedTime = std::chrono::duration_cast<std::chrono::microseconds>(end - start).count() / 1'000'000.0;
        std::atomic_thread_fence(std::memory_order_relaxed);
        return static_cast<Rep>(countedTime);
    }

    template <class Clock>
    bool Stopwatch<Clock>::IsRunning() const {
        return running;
    }
}
