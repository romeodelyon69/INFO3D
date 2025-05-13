#pragma once

#include "cgp/cgp.hpp"

#define PI 3.14159265358979323846f

inline float angleBetweenSigned(cgp::vec3 a, cgp::vec3 b, cgp::vec3 ref_axis) {
    cgp::vec3 a_n = cgp::normalize(a);
    cgp::vec3 b_n = cgp::normalize(b);
    float angle = std::acos(cgp::clamp(dot(a_n, b_n), -1.0f, 1.0f));

    // Déterminer le signe via le produit vectoriel projeté sur ref_axis
    float sign = dot(cross(a_n, b_n), ref_axis);
    if (sign < 0)
        angle = -angle;

    return angle; // Angle en radians entre -π et π
}

inline float angleBetween(cgp::vec3 a, cgp::vec3 b) {
    cgp::vec3 a_n = cgp::normalize(a);
    cgp::vec3 b_n = cgp::normalize(b);
    return std::acos(cgp::clamp(dot(a_n, b_n), -1.0f, 1.0f)); // Angle en radians
}