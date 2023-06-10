#pragma once
#include "Scene.h"

struct sampleProb {
	float diffuseProb;
	float specularProb;
	float reflectProb;
	float refractProb;
};

enum class sampleEvent{
	DIFFUSE, SPECULAR, REFLECT, REFRACT
};

class Renderer
{
public:
	Scene* pScene;
	int pixelSampleNum;
	int intSampleNum;
	int maxRecursiveDepth;
	float RussianRoulette = 0.8;
	int renderTimes = 0;
	int MISSample = 3;
	unsigned char* frameBuffer;
	bool useMIS = true;

	Renderer(Scene* s);

	unsigned char* Render();
	
	Eigen::Vector3f trace(Ray& ray,  int currentDepth = 0);

	//计算采样概率
	void ComputeProb(Ray& ray, const Intersection& I, sampleProb& prob);

	Ray mcSample(Ray& ray,const Intersection& I, float& pdf);

	bool russianRoulette(float p, float& survivor);

	Vector3f directLight_MIS(Ray& ray, Intersection& intersection);

	
};

