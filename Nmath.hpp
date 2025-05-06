#include "cgp/cgp.hpp"

float angleBetweenSigned(vec3 a, vec3 b, vec3 ref_axis) {
    vec3 a_n = cgp::normalize(a);
    vec3 b_n = cgp::normalize(b);
    float angle = std::acos(clamp(dot(a_n, b_n), -1.0f, 1.0f));

    // Déterminer le signe via le produit vectoriel projeté sur ref_axis
    float sign = dot(cross(a_n, b_n), ref_axis);
    if (sign < 0)
        angle = -angle;

    return angle; // Angle en radians entre -π et π
}

float angleBetween(vec3 a, vec3 b) {
    vec3 a_n = cgp::normalize(a);
    vec3 b_n = cgp::normalize(b);
    return std::acos(clamp(dot(a_n, b_n), -1.0f, 1.0f)); // Angle en radians
}