#include "ButterworthFilter.hpp"
#include <cmath>

ButterworthFilter::ButterworthFilter(double cutoff_freq, double sampling_time) {
    double omega = 2.0 * M_PI * cutoff_freq;
    double ts = sampling_time;

    double tan_omega = tan(omega * ts / 2.0);
    double norm = 1.0 / (1.0 + sqrt(2.0) * tan_omega + tan_omega * tan_omega);

    a0 = tan_omega * tan_omega * norm;
    a1 = 2.0 * a0;
    a2 = a0;
    b1 = 2.0 * (tan_omega * tan_omega - 1.0) * norm;
    b2 = (1.0 - sqrt(2.0) * tan_omega + tan_omega * tan_omega) * norm;

    prev_input = {0.0, 0.0};
    prev_output = {0.0, 0.0};
}

double ButterworthFilter::Filter(double input) {
    double output = a0 * input + a1 * prev_input[0] + a2 * prev_input[1]
                    - b1 * prev_output[0] - b2 * prev_output[1];

    // Shift values
    prev_input[1] = prev_input[0];
    prev_input[0] = input;

    prev_output[1] = prev_output[0];
    prev_output[0] = output;

    return output;
}

