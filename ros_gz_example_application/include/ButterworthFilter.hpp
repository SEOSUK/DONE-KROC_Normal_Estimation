#ifndef BUTTERWORTHFILTER_HPP
#define BUTTERWORTHFILTER_HPP

#include <vector>

class ButterworthFilter {
public:
    ButterworthFilter(double cutoff_freq, double sampling_time);
    double Filter(double input);

private:
    double a0, a1, a2, b1, b2;
    std::vector<double> prev_input, prev_output;
};

#endif // BUTTERWORTHFILTER_HPP

