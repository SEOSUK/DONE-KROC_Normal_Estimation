#ifndef FILTERED_VECTOR_HPP
#define FILTERED_VECTOR_HPP

#include <Eigen/Dense>
#include <vector>
#include "ButterworthFilter.hpp"

class FilteredVector {
private:
    std::vector<ButterworthFilter> filters;
    size_t size;

public:
    FilteredVector(size_t vectorSize) : size(vectorSize) {
        filters.resize(size, ButterworthFilter());
    }

    Eigen::VectorXd apply(const Eigen::VectorXd& input) {
        Eigen::VectorXd filtered(input.size());
        for (size_t i = 0; i < size; ++i) {
            filtered[i] = filters[i].apply(input[i]);
        }
        return filtered;
    }
};

#endif  // FILTERED_VECTOR_HPP

