#pragma once

#include <iostream>
#include <cmath>
#include <random>
#include <fstream>


int myChk(float a);

#define PI 3.141592653589793f
const float  EPSILON = 0.00001;
const float Infinity = std::numeric_limits<float>::max();

#define Print(x) (std::cout << x << std::endl)

inline float RNG()
{
    std::random_device dev;
    std::mt19937 rng(dev());
    std::uniform_real_distribution<float> dist(0.f, 1.f); // distribution in range [1, 6]

    return dist(rng);
}
