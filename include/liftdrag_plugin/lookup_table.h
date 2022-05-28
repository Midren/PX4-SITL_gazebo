#pragma once 
#include <cmath>
#include <iostream>

#include <optional>
#include <utility>
#include <vector>

#define assertm(exp, msg) assert(((void) msg, exp))

static inline double lerp(double a, double b, double t) noexcept {
    return a + t * (b - a);
}

template <typename T>
class LookUpTable {
public:
    LookUpTable() = default;

    LookUpTable(std::vector<double> indeces, std::vector<T> values)
        : _indeces(std::move(indeces)),
          _values(std::move(values)) {
        bool is_sorted = std::is_sorted(_indeces.begin(), _indeces.end());
        assertm(is_sorted, "Values should be passed sorted");
    }

    std::optional<T> get(double point) {
        assertm(_indeces.size() >= 0, "Table should not be empty at get");
        // FIXME: dat file should contain radian instead of degrees
        point *= 180.0/M_PI;
        size_t i = get_closest_idx(point);

        if ((i < 0) || (i >= _indeces.size()))
            return {};

        if (i == 0 || i == _indeces.size() - 1)
            return _values[i];

        auto t = (point - _indeces[i]) / _indeces[i + 1];
        return lerp(_values[i], _values[i + 1], t);
    }

private:
    int get_closest_idx(double point) {
        if (_indeces[0] > point)
            return -1;
        for (auto i = 0U; i < _indeces.size(); i++)
            if (_indeces[i] >= point)
                return i;
        return _indeces.size();
    }

//private:
public:
    std::vector<double> _indeces{};
    std::vector<T> _values{};
};
