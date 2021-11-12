#include "runningAverage.hpp"

namespace Pronounce {


    template<int L>
    RunningAverage<L>::RunningAverage() {
        //runningArray = new double[L];
    }

    template<int L>
    void RunningAverage<L>::add(double x) {
        for (int i = 1; i < L; i++) {
            this->runningArray[i - 1] = runningArray[i];
        }
        this->runningArray[L - 1] = x;
    }

    template<int L>
    double RunningAverage<L>::getAverage() {
        double sum = 0;
        for (double i : runningArray) {
            sum += i;
        }
        return sum / L;
    }

    template<int L>
    RunningAverage<L>::~RunningAverage() {
    }
} // namespace Pronounce