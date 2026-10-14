//
// Created by Siwei Wang on 2026/10/11.
//

#include "math.hpp"

float limit(float a, float a_min, float a_max)
{
    if ( a > a_max) return a_max;
    if ( a < a_min) return a_min;
    return a;
}