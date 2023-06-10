#pragma once
#include "Camera.h"
#include "Model.h"
#include <ctime>

static clock_t start, end;

class Scene
{
public:
	Camera* camera;

	std::vector<Object*> objects;
	std::vector<Light*> lights;
	Model* model;
	float lightArea;

	Scene(Camera* cam, Model* _model) : camera(cam), lightArea(0), model(_model) {}

	void AddMesh(Mesh* m) { objects.insert(objects.end(), m->TriList.begin(), m->TriList.end()); }
	void AddLight(Light* l) { lights.push_back(l); lightArea += l->area; }

	void init() { buildBVH(); }

	Intersection intersect(const Ray& ray) const { return bvh->Intersect(ray); }
	BVH* bvh;
	void buildBVH()
	{
		Print("开始构建BVH");
		start = clock();
		bvh = new BVH(objects);
		end = clock();
		double time = (double)(end - start) / CLOCKS_PER_SEC;
		Print("构建BVH完成，用时：" << time << "s");
	}
	Ray* GetRays(int w, int h, int sampleNum)
	{
		Ray* ret = new Ray[sampleNum];
		for (int i = 0; i < sampleNum; ++i)
		{
			ret[i] = camera->GetRay(w, h);
		}
		return ret;
	}
	bool isInShadow(Ray& ray) { return true; }
	void sampleLight(Intersection& Pos, float& pdf)
	{
		float sampleA = RNG() * lightArea;
		float areaSum = 0;
		bool flag = false;
		for (auto& l : lights)
		{
			areaSum += l->area;
			if (areaSum < sampleA) continue;
			else areaSum -= l->area;
			for (auto& t : l->pMesh->TriList)
			{
				areaSum += t->getArea();
				if (areaSum >= sampleA)
				{
					t->Sample(Pos, pdf);
					flag = true;
					break;
				}
			}
			if (flag) break;
		}
	}
	void sampleLight(Intersection& Pos, float& pdf, Light* l)
	{
		float sampleA = RNG() * l->area;
		float areaSum = 0;
		for (auto& t : l->pMesh->TriList)
		{
			areaSum += t->getArea();
			if (areaSum >= sampleA)
			{
				t->Sample(Pos, pdf);
				break;
			}
		}
	}
	Eigen::Vector3f reflect(const Eigen::Vector3f& I, Eigen::Vector3f N)
	{
		return I - 2 * I.dot(N) * N;
	}

	Eigen::Vector3f refract(const Vector3f& InRay, const Vector3f& normal, const float& ior)
	{
		float cosi = InRay.dot(normal);
		Vector3f n = normal;
		float eta_in = 1, eta_tr = ior;
		if (cosi < 0) cosi = -cosi;
		else {
			n = -normal;
			std::swap(eta_in, eta_tr);
		}
		float eta = eta_in / eta_tr;

		float k = 1 - eta * eta * (1 - cosi * cosi);
		return (k < 0 ? Vector3f(0, 0, 0) : ((eta * cosi - std::sqrtf(k)) * n + eta * InRay));
	}
};