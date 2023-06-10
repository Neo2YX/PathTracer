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
	//���㷴�䷽��
	Vector3f reflect(const Vector3f& InRay, const Vector3f& normal);
	//�������䷽��
	Vector3f refract(const Vector3f& InRay, const Vector3f& normal, const float& ior);
	//�������������
	void fresnel(const Vector3f& I, const Vector3f& N, const float& ior, float& kr) const;
	//������ֲ�����ת��Ϊ��������
	Vector3f toWorld(const Vector3f& a, const Vector3f& N);
public:
	std::string name;
	Vector3f emis;
	//���ʲ���
	Vector3f kd, ks, tr;
	double Ns, Ni;
	//��ͼ
	Texture Tex;
	bool hasTex = false;

	

	inline Material(Vector3f e = Vector3f(0, 0, 0)) : emis(e) {}
	inline std::string getName() { return name; }

	inline Vector3f getColorAt(float u, float v);
	inline Vector3f getEmission() { return emis; }
	inline bool hasEmission() {return emis.norm() > EPSILON ? true : false; }

	//ͨ��cos-weight����һ������(diffuse)
	Vector3f CosineWeightImportanceSample(const Vector3f& wi, const Vector3f& N);
	//�Ը߹����Ҫ�Բ���
	Vector3f specularWeightImportanceSample(const Vector3f& wi, const Vector3f& N, float Ns);
	//������ߵ�pdf(cosine weight\ specular
	float pdf_cosine(const Vector3f& wi, const Vector3f& wo, const Vector3f& N);
	float pdf_specular(const Vector3f& wi, const Vector3f& wo, const Vector3f& N, float Ns);
	//������ߵĹ���
	Vector3f eval_diffuse(const Vector3f& wi, const Vector3f& wo, const Vector3f& N, Intersection& I);
	Vector3f eval_specular(const Vector3f& wi, const Vector3f& wo, const Vector3f& N);
	Vector3f eval_light(const Vector3f& wi, const Vector3f& wo, const Vector3f& N, Intersection& I);
};



