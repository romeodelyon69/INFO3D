#pragma once
#include "cgp/cgp.hpp"
#include "stdlib.h"
#include "environment.hpp"

class Segment{
    public:
        cgp::curve_drawable segment;
        //constructor
        Segment();
        void initialize();
        void draw(environment_structure& environment, cgp::vec3 a, cgp::vec3 b, cgp::vec3 color = cgp::vec3(1,1,1));
};