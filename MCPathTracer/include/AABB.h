#pragma once

#include "Ray.h"
#include <limits>
#include <array>
#include <cmath>

class AABB
{
public:
    Eigen::Vector3f low, high;//AABB的2个对角坐标点，以表示AABB

    AABB()
    {
        double minNum = std::numeric_limits<double>::lowest();
        double maxNum = std::numeric_limits<double>::max();

        low = Eigen::Vector3f(maxNum, maxNum, maxNum);
        high = Eigen::Vector3f(minNum, minNum, minNum);
    }
    AABB(Eigen::Vector3f P) : low(P), high(P) {}
    AABB(Eigen::Vector3f p1, Eigen::Vector3f p2) 
    {
        low = Eigen::Vector3f(fmin(p1.x(), p2.x()), fmin(p1.y(), p2.y()), fmin(p1.z(), p2.z()));
        high = Eigen::Vector3f(fmax(p1.x(), p2.x()), fmax(p1.y(), p2.y()), fmax(p1.z(), p2.z()));
    }

    Eigen::Vector3f Diagonal() const { return high - low; }
    int maxAxis() //返回最大的跨度值
    {
        Eigen::Vector3f d = Diagonal();
        if (d.x() > d.y() && d.x() > d.z()) return 0;
        else if (d.y() > d.z()) return 1;
        else return 2;
    }
    double SurfaceArea()
    {
        Eigen::Vector3f d = Diagonal();
        return 2 * (d.x() * d.y() + d.x() * d.z() + d.y() * d.z());
    }

    Eigen::Vector3f GetCenter() { return 0.5 * (low + high); }

    AABB Intersect(const AABB Box)
    {
        return AABB(Eigen::Vector3f(fmax(low.x(), Box.low.x()), fmax(low.y(), Box.low.y()), fmax(low.z(), Box.low.z())),
            Eigen::Vector3f(fmin(high.x(), Box.high.x()), fmin(high.y(), Box.high.y()), fmin(high.z(), Box.high.z())));
    }

    Eigen::Vector3f Offset(const Eigen::Vector3f& p) const
    {
        Eigen::Vector3f o = p - low;
        if (high.x() > high.x())
            o.x() /= high.x() - low.x();
        if (high.y() > high.y())
            o.y() /= high.y() - low.y();
        if (high.z() > high.z())
            o.z() /= high.z() - low.z();
        return o;
    }

    bool IsOverlap(const AABB& Box1, const AABB& Box2)
    {
        bool x = (Box1.low.x() <= Box2.high.x()) && (Box1.high.x() >= Box2.low.x());
        bool y = (Box1.low.y() <= Box2.high.y()) && (Box1.high.y() >= Box2.low.y());
        bool z = (Box1.low.z() <= Box2.high.z()) && (Box1.high.z() >= Box2.low.z());

        return x && y && z;
    }

    bool IsInside(const Eigen::Vector3f& P, const AABB Box)
    {
        return(P.x() >= Box.low.x() && P.x() <= Box.high.x() && P.y() >= Box.low.y() && P.y() <= Box.high.y() && P.z() >= Box.low.z() && P.z() <= Box.high.z());
    }

    inline const Eigen::Vector3f& operator[](int i) const
    {
        return (i == 0) ? low : high;
    }

    inline bool IntersectP(const Ray& ray, const Eigen::Vector3f& invDir, const std::array<int, 3>& IsNeg) const
    {
        const auto& origin = ray.origin;
        float tEnter = -std::numeric_limits<float>::infinity();
        float tExit = std::numeric_limits<float>::infinity();
        for (int i = 0; i < 3; i++)
        {
            float min = (low[i] - origin[i]) * invDir[i];
            float max = (high[i] - origin[i]) * invDir[i];
            if (!IsNeg[i])
            {
                std::swap(min, max);
            }
            tEnter = std::max(min, tEnter);
            tExit = std::min(max, tExit);
        }

        return tEnter <= tExit && tExit >= 0;
    }

};

//合并两个AABB，AABB与点合并

inline AABB Merge(const AABB& Box1, const AABB& Box2)
{
    return AABB(Eigen::Vector3f(fmin(Box1.low.x(), Box2.low.x()), fmin(Box1.low.y(), Box2.low.y()), fmin(Box1.low.z(), Box2.low.z())),
        Eigen::Vector3f(fmax(Box1.high.x(), Box2.high.x()), fmax(Box1.high.y(), Box2.high.y()), fmax(Box1.high.z(), Box2.high.z())));
}

inline AABB Merge(const AABB& Box, Eigen::Vector3f& P)
{
    return AABB(Eigen::Vector3f(fmin(Box.low.x(), P.x()), fmin(Box.low.y(), P.y()), fmin(Box.low.z(), P.z())),
        Eigen::Vector3f(fmax(Box.high.x(), P.x()), fmax(Box.high.y(), P.y()), fmax(Box.high.z(), P.z())));
}