#pragma once

#include "math/math.hh"

struct AABB;
struct Plane;
void DebugDrawAABB(const AABB& aabb);
void DebugDrawPlane(const AABB& aabb, const Plane& plane);
void DebugDrawPoint(Vec3_arg pos, float r, float g, float b);
void DebugDrawSphere(Vec3_arg center, float radius, float r, float g, float b, float a);
void DebugDrawTriangle(Vec3_arg pos0, Vec3_arg pos1, Vec3_arg pos2, float r, float g, float b, float a);
void DebugDrawVector(Vec3_arg from, Vec3_arg vec, float r, float g, float b, float a);
void DebugDrawLine(Vec3_arg from, Vec3_arg to, float r, float g, float b, float a);
void RenderDebugDraw();
void ClearDebugDraw();
void EnableDebugDraw();
void DisableDebugDraw();

