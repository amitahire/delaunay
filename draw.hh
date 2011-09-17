#ifndef INCLUDED_draw_HH
#define INCLUDED_draw_HH

struct AABB;
struct Plane;

void DrawPlane(const AABB& bounds, const Plane& plane);

#endif

