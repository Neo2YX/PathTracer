#pragma once

#include "Eigen/Eigen"

enum SOURCE { NONE, DIRECT, DIFFUSE_REFLECT, SPECULAR_REFLECT, TRANSMISSON, REFLECT };


//一根光线的解析表达式：R(t) = origin + t*direction
struct Ray {
    Eigen::Vector3f origin;
    Eigen::Vector3f direction, direction_inv;
    double t;
    double t_min, t_max;
    SOURCE source;

    Ray(const Eigen::Vector3f& ori = Eigen::Vector3f(0.f,0.f,0.f), const Eigen::Vector3f& dir = Eigen::Vector3f(0.f, 0.f, 0.f), SOURCE _source = SOURCE::NONE,const double temp_t = 0.0) : origin(ori), direction(dir), source(_source), t(temp_t) {
        direction_inv = Eigen::Vector3f(1. / direction.x(), 1. / direction.y(), 1. / direction.z());
        t_min = 0.0;
        t_max = std::numeric_limits<double>::max();

    }

    //获得t值下ray上的点
    Eigen::Vector3f operator()(double t) const { return origin + direction * t; }

    friend std::ostream& operator<<(std::ostream& os, const Ray& r) {
        os << "[origin:=" << r.origin << ", direction=" << r.direction << ", time=" << r.t << "]\n";
        return os;
    }

    bool refract(const Eigen::Vector3f& normal, float nit, Eigen::Vector3f& refract_direction)
    {
        float ndoti = normal.dot(direction),
            k = 1.0f - nit * nit * (1.0f - ndoti * ndoti);
        if (k >= 0.0f) {
            refract_direction = nit * direction - normal * (nit * ndoti + sqrt(k));
            return true;
        }
        else return false;
    }

    Eigen::Vector3f reflect(const Eigen::Vector3f& normal)
    {
        return direction - 2 * normal.dot(direction) * normal;
    }
};