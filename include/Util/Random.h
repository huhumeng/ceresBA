#pragma once

#include <random>

inline double gaussion(double miu, double sigma){ 
    static std::default_random_engine generator;
    static std::normal_distribution<double> distribution(miu, sigma);
    return distribution(generator);
}

inline double uniform(double from, double to){

    static std::default_random_engine generator;
    static std::uniform_real_distribution<double> distribution(from, to);
    return distribution(generator);

}

inline int uniform(int from, int to){
    static std::default_random_engine generator;
    static std::uniform_int_distribution<int> distribution(from, to);
    return distribution(generator);
}
