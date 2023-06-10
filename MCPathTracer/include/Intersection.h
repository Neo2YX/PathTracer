#pragma once
#include "Eigen/Eigen"
class Material;

class Object;
struct Intersection
{
    Intersection() {
        happened = false;
        coords = Eigen::Vector3f(0.f, 0.f,0.f);
        normal = Eigen::Vector3f(0.f, 0.f,0.f);
        distance = std::numeric_limits<double>::max();
        obj = nullptr;
        m = nullptr;
    }
    bool happened;
    Eigen::Vector3f coords;
    Eigen::Vector3f tcoords;
    Eigen::Vector3f normal;
    Eigen::Vector3f emit;
    double distance;
    Object* obj;
    Material* m;
};