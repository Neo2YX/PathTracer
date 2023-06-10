#pragma once
#include "Eigen/Eigen"
#include "Ray.h"
#include "Intersection.h"
#include "AABB.h"
class Object
{
public:
	Object() {}
    virtual ~Object() {}
    virtual bool intersect(const Ray& ray) = 0;
    virtual bool intersect(const Ray& ray, float&, uint32_t&) const = 0;
    virtual Intersection getIntersection(Ray _ray) = 0;
    virtual void getSurfaceProperties(const Eigen::Vector3f&, const Eigen::Vector3f&, const uint32_t&, const Eigen::Vector2f&, Eigen::Vector3f&, Eigen::Vector2f&) const = 0;
    virtual Eigen::Vector3f evalDiffuseColor(const Eigen::Vector2f&) const = 0;
    virtual AABB getAABB() = 0;
    virtual float getArea() = 0;
    virtual void Sample(Intersection& pos, float& pdf) = 0;
    virtual bool hasEmit() = 0;
};