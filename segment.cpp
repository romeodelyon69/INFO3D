#include "segment.hpp"


Segment::Segment(){}
void Segment::initialize()
{
    segment.display_type = cgp::curve_drawable_display_type::Segments;
    segment.initialize_data_on_gpu({{0,0,0},{1,0,0}});
}

void Segment::draw(environment_structure& environment, cgp::vec3 a, cgp::vec3 b, cgp::vec3 color)
{
    segment.color = color;
    segment.vbo_position.update(numarray<cgp::vec3>{a, b});
    cgp::draw(segment, environment);
}