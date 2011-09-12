#ifndef INCLUDED_debugdraw_HH
#define INCLUDED_debugdraw_HH

#include "math/math.hh"

struct AABB;
struct Plane;
void DebugDrawAABB(const AABB& aabb);
void DebugDrawPlane(const AABB& aabb, const Plane& plane);
void DebugDrawPoint(Vec3_arg pos, float r, float g, float b);
void DebugDrawSphere(Vec3_arg center, float radius, float r, float g, float b, float a);
void RenderDebugDraw();
void ClearDebugDraw();

#endif

