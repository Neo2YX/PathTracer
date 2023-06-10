#pragma once
#include "Object.h"
#include "Material.h"
#include "AABB.h"
#include "Intersection.h"
#include <vector>
#include<string>
#include "Light.h"
#include <tuple>
#include "BVH.h"


//对三角形进行插值，得到其中一点的中心坐标
static std::tuple<float, float, float> interpolatePoint(Eigen::Vector3f v0, Eigen::Vector3f v1, Eigen::Vector3f v2, Eigen::Vector3f p)
{
    Vector3f ep0 = v0 - p, ep1 = v1 - p, ep2 = v2 - p;
    float area_p01 = 0.5 * ep0.cross(ep1).norm();
    float area_p12 = 0.5 * ep1.cross(ep2).norm();
    float area_p02 = 0.5 * ep2.cross(ep0).norm();

    float area = area_p01 + area_p12 + area_p02;
    float alpha = area_p12 / area;
    float beta = area_p02 / area;

    return std::make_tuple(alpha, beta, 1 - alpha - beta);
}


//检测光线与三角面片是否相交
static bool rayTriangleIntersect(const Vector3f& v0, const Vector3f& v1,
    const Vector3f& v2, const Vector3f& orig,
    const Vector3f& dir, float& tnear, float& u, float& v)
{
    Eigen::Vector3f e1 = v1 - v0;
    Eigen::Vector3f e2 = v2 - v0;
    Eigen::Vector3f S1 = dir.cross(e2);
    float det = S1.dot(e1);
    if (fabsf(det) < EPSILON || det < 0) return false;
    float det_inv = 1. / det;
    Eigen::Vector3f S = orig - v0;
    u = S1.dot(S) * det_inv;
    if (u < 0 || u > 1) return false;
    Eigen::Vector3f S2 = S.cross(e1);
    v = S2.dot(dir) * det_inv;
    if (v < 0 || v > 1) return false;
    tnear = S2.dot(e2) * det_inv;

    return true;
}



class Triangle : public Object
{
public:
    Eigen::Vector3f v0, v1, v2; // 顶点
    Eigen::Vector3f n0, n1, n2; // 法线
    Eigen::Vector2f uv0, uv1, uv2; //贴图
    float area;
    Material* mat;

    Triangle() : mat(nullptr) {}
    Triangle(Eigen::Vector3f _v0, Eigen::Vector3f _v1, Eigen::Vector3f _v2, Eigen::Vector3f _n0, Eigen::Vector3f _n1, Eigen::Vector3f _n2, Eigen::Vector2f _uv0, Eigen::Vector2f _uv1, Eigen::Vector2f _uv2, Material* _m = nullptr) : v0(_v0), v1(_v1), v2(_v2), uv0(_uv0), uv1(_uv1), uv2(_uv2), n0(_n0), n1(_n1), n2(_n2), mat(_m)
    {
        Eigen::Vector3f e1 = v1 - v0;
        Eigen::Vector3f e2 = v2 - v0;
        area = e1.cross(e2).norm() * 0.5;
    }
    //通过插值获取UV和法线
    Eigen::Vector3f GetNormalAt(Eigen::Vector3f v)
    {
        float alpha, beta, gamma;
        std::tie(alpha, beta, gamma) = interpolatePoint(v0, v1, v2, v);
        return n0 * alpha + n1 * beta + n2 * gamma;
    }
    Eigen::Vector2f GetUVAt(Eigen::Vector3f v)
    {
        float alpha, beta, gamma;
        std::tie(alpha, beta, gamma) = interpolatePoint(v0, v1, v2, v);
        return uv0 * alpha + uv1 * beta + uv2 * gamma;
    }

    bool intersect(const Ray& ray) override { return true; }
    bool intersect(const Ray& ray, float& tnear,
        uint32_t& index) const override {
        return false;
    }
    Intersection getIntersection(Ray ray, float tnear);
    Intersection getIntersection(Ray ray) override;
    void getSurfaceProperties(const Vector3f& P, const Vector3f& I,
        const uint32_t& index, const Vector2f& uv,
        Vector3f& N, Vector2f& st) const override
    {
        N = n0;
    }
    Vector3f evalDiffuseColor(const Vector2f&) const override;
    AABB getAABB() override { return Merge(AABB(v0, v1), v2); }
    void Sample(Intersection& pos, float& pdf) {
        float x = std::sqrt(RNG()), y = RNG();
        pos.coords = v0 * (1.0f - x) + v1 * (x * (1.0f - y)) + v2 * (x * y);
        pos.normal = n0;
        pos.emit = this->mat->emis;
        pos.m = this->mat;
        pdf = 1.0f / area;
    }
    float getArea() {
        return area;
    }
    bool hasEmit() {
        return mat->hasEmission();
    }

};

struct Mesh {
    std::vector<Object*> TriList;
    Material* Mat;
    std::string name;
};

class Model : public Object
{
public:
    std::vector<Eigen::Vector3f> vertices;
    std::vector<Eigen::Vector3f> normals;
    std::vector<Eigen::Vector2f> UVs;
    std::vector<Light> lights;
    std::map<std::string, Material> mats;


    std::vector<Mesh> meshes;
    std::vector<BVH*> bvhs;
    std::vector<float> areas;

    Model(){}
    Model(const std::string& filename, const std::string& filepath);

    bool load(const std::string& filename, const std::string& filepath);
    void deb_PrintInfo();

    bool intersect(const Ray& ray) override { return true; }
    bool intersect(const Ray& ray, float& tnear,
        uint32_t& index) const override;
    Intersection getIntersection(Ray ray) override
    {
        Intersection ret;
        for(auto bvh : bvhs)
        if (bvh) ret = bvh->Intersect(ray);
        return ret;
    }
    void getSurfaceProperties(const Vector3f& P, const Vector3f& I,
        const uint32_t& index, const Vector2f& uv,
        Vector3f& N, Vector2f& st) const override
    {
        Triangle* tri = (Triangle*)&meshes[index % 100].TriList[index / 100];
        N = tri->n0;
        st = (1 - uv.x() - uv.y()) * tri->uv0 + uv.x() * tri->uv1 + uv.y() * tri->uv2;
    }
    Vector3f evalDiffuseColor(const Vector2f& st) const override
    {
        float scale = 5;
        float pattern =
            (fmodf(st.x() * scale, 1) > 0.5) ^ (fmodf(st.y() * scale, 1) > 0.5);
        return Vector3f(0.815, 0.235, 0.031) * pattern + Vector3f(0.937, 0.937, 0.231) * (1 - pattern);
    }
    AABB getAABB() override { return AABB(); }
    float getArea() override
    {
        return 0;
    }
    void Sample(Intersection& pos, float& pdf) override
    {
        bvhs[0]->Sample(pos, pdf);
        
    }
    bool hasEmit() override
    {
        return true;
    }
};

