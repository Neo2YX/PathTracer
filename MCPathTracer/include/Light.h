#pragma once

#include "Eigen/Eigen"
#include "utils.h"

class Mesh;

class Light
{
public:
	Light(const std::string& _name, const Eigen::Vector3f& Int): name(_name), Intensity(Int), area(0) {}
    std::string name;
	Eigen::Vector3f Intensity;
    Mesh* pMesh = nullptr;
    float area;
};

class AreaLight : public Light
{
public:
    AreaLight(const std::string& _name, const Eigen::Vector3f& Int) : Light(_name, Int) { }

    
    void AddMesh(Mesh* m);
};

