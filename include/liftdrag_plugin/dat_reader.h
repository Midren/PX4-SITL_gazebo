#pragma once

#include <filesystem>
#include <fstream>
#include <vector>

namespace fs = std::filesystem;

template <typename T>
class DatReader {
public:
    DatReader(const fs::path& file) {
        if (!fs::exists(file))
            throw std::runtime_error("File does not exist: " + file.string());

        _fin.open(file);

        if (!_fin.is_open())
            throw std::runtime_error("Cannot open file: fstream is not opened");
    };

    std::tuple<std::vector<double>, std::vector<T>> read() {
        std::vector<double> indeces;
        std::vector<T> values;

        double idx{};
        T value;

        while (_fin) {
            _fin >> idx >> value;
            indeces.push_back(idx);
            values.push_back(value);
        }

        return {indeces, values};
    }



private:
    std::ifstream _fin;
};
