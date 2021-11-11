#pragma once

namespace Pronounce
{
    template<int L>
    class RunningAverage {
    private:
        double runningArray[L];
    public:
        RunningAverage();
        void add(double x);
        double getAverage();
        ~RunningAverage();
    };

} // namespace Pronounce
