#pragma once
#include "Eigen/Eigen"
#include "utils.h"
#include<string>
#include<cmath>
#include "Texture.h"

class Triangle;
class Mesh;
class Intersection;

using namespace Eigen;

class Material
{
public:
	//计算反射方向
	Vector3f reflect(const Vector3f& InRay, const Vector3f& normal);
	//计算折射方向
	Vector3f refract(const Vector3f& InRay, const Vector3f& normal, const float& ior);
	//计算菲涅尔方程
	void fresnel(const Vector3f& I, const Vector3f& N, const float& ior, float& kr) const;
	//将半球局部坐标转化为世界坐标
	Vector3f toWorld(const Vector3f& a, const Vector3f& N);
public:
	std::string name;
	Vector3f emis;
	//材质参数
	Vector3f kd, ks, tr;
	double Ns, Ni;
	//贴图
	Texture Tex;
	bool hasTex = false;

	

	inline Material(Vector3f e = Vector3f(0, 0, 0)) : emis(e) {}
	inline std::string getName() { return name; }

	inline Vector3f getColorAt(float u, float v);
	inline Vector3f getEmission() { return emis; }
	inline bool hasEmission() {return emis.norm() > EPSILON ? true : false; }

	//通过cos-weight采样一根光线(diffuse)
	Vector3f CosineWeightImportanceSample(const Vector3f& wi, const Vector3f& N);
	//对高光的重要性采样
	Vector3f specularWeightImportanceSample(const Vector3f& wi, const Vector3f& N, float Ns);
	//计算光线的pdf(cosine weight\ specular
	float pdf_cosine(const Vector3f& wi, const Vector3f& wo, const Vector3f& N);
	float pdf_specular(const Vector3f& wi, const Vector3f& wo, const Vector3f& N, float Ns);
	//计算光线的贡献
	Vector3f eval_diffuse(const Vector3f& wi, const Vector3f& wo, const Vector3f& N, Intersection& I);
	Vector3f eval_specular(const Vector3f& wi, const Vector3f& wo, const Vector3f& N);
	Vector3f eval_light(const Vector3f& wi, const Vector3f& wo, const Vector3f& N, Intersection& I);
};



