#include "Renderer.h"
#include <omp.h>





//计算颜色值的亮度
float luminance(const Vector3f& RGB)
{
	return 0.212671f * RGB.x() + 0.715160f * RGB.y() + 0.072169f * RGB.z();
}

const Eigen::Vector3f operator*(const Eigen::Vector3f& a, const Eigen::Vector3f& b)
{
	return Vector3f(a.x() * b.x(), a.y() * b.y(), a.z() * b.z());
}

float power_heuristic(float aw, float a, float bw, float b)
{
	if (!aw && !bw) return 0;
	float x = a * a, y = b * b;
	return(aw * x) / (aw * x + bw * y);
}

Renderer::Renderer(Scene* s)
{
	pScene = s;
	pixelSampleNum = 5;
	intSampleNum = 100;
	maxRecursiveDepth = 5;
	int framesize = s->camera->width * s->camera->height * 4; //RGBA
	frameBuffer = new unsigned char[framesize];
	for (int ind = 0; ind < s->camera->width * s->camera->height; ind++) {
		frameBuffer[ind * 4] = 0;
		frameBuffer[ind * 4 + 1] = 0;
		frameBuffer[ind * 4 + 2] = 0;
		frameBuffer[ind * 4 + 3] = 255;
	}
}

bool Renderer::russianRoulette(float p, float& survivor)
{
	survivor = RNG();
	if (survivor > p) return true;
	else return false;
}

Ray Renderer::mcSample(Ray& ray, const Intersection& I, float& pdf)
{
	Material* m = I.m;
	Vector3f dir;
	sampleEvent event;
	sampleProb probs;
	ComputeProb(ray, I, probs);

	pdf = 0;
	float index = RNG();
	if (index <= probs.diffuseProb) event = sampleEvent::DIFFUSE;
	else if (index <= probs.diffuseProb + probs.specularProb) event = sampleEvent::SPECULAR;
	else if (index <= probs.diffuseProb + probs.specularProb + probs.reflectProb) event = sampleEvent::REFLECT;
	else event = sampleEvent::REFRACT;

	switch (event)
	{
	case sampleEvent::DIFFUSE:
		dir = m->CosineWeightImportanceSample(ray.direction, I.normal);
		pdf += probs.diffuseProb * m->pdf_cosine(ray.direction, dir, I.normal);
		if (probs.specularProb) pdf += probs.specularProb * m->pdf_specular(ray.direction, dir, I.normal, m->Ns);
		return Ray(I.coords, dir, SOURCE::DIFFUSE_REFLECT);
	case sampleEvent::SPECULAR:
		dir = m->specularWeightImportanceSample(ray.direction, I.normal, m->Ns);
		pdf += probs.specularProb * m->pdf_specular(ray.direction, dir, I.normal, m->Ns);
		if(probs.diffuseProb) pdf += probs.diffuseProb * m->pdf_cosine(ray.direction, dir, I.normal);
		return Ray(I.coords, dir, SOURCE::SPECULAR_REFLECT);
	case sampleEvent::REFLECT:
		dir = m->reflect(ray.direction, I.normal);
		pdf += probs.reflectProb;
		return Ray(I.coords, dir, SOURCE::REFLECT);
	case sampleEvent::REFRACT:
		//dir = m->refract(ray.direction, I.normal, m->Ni);
		dir = ray.direction;
		pdf += probs.refractProb;
		return Ray(I.coords, dir.normalized(), SOURCE::TRANSMISSON);
	}

	return Ray(I.coords, dir, SOURCE::NONE);
}

Eigen::Vector3f Renderer::trace(Ray& ray, int currentDepth)
{
	Intersection intersection;
	intersection = pScene->intersect(ray);
	
	if (!intersection.happened) return Eigen::Vector3f(0.f, 0.f, 0.f);
	else if (intersection.m->hasEmission() || currentDepth >= maxRecursiveDepth)
	{
		return intersection.m->emis;
	}
	else
	{	
		
		Eigen::Vector3f indirectLight = Vector3f(0.f,0.f,0.f), directLight = Vector3f(0.f, 0.f, 0.f);
		Eigen::Vector3f brdf;
		Ray newRay;
		float pdf;
		newRay = mcSample(ray, intersection, pdf);

		if (newRay.source != SOURCE::NONE && RNG() < RussianRoulette && abs(pdf)>EPSILON)
		{
			//计算间接光照
			indirectLight = trace(newRay, currentDepth + 1);
			if (indirectLight.norm() > EPSILON)
			{
				float cosTheta;
				brdf = Vector3f(0.f, 0.f, 0.f);
				switch (newRay.source)
				{
				case SOURCE::DIFFUSE_REFLECT:
					cosTheta = std::max(0.f, newRay.direction.dot(intersection.normal));
					brdf += intersection.m->eval_diffuse(-newRay.direction, -ray.direction, intersection.normal, intersection) * cosTheta;
					
					if (intersection.m->ks.norm() > EPSILON)
						brdf += intersection.m->eval_specular(-newRay.direction, -ray.direction, intersection.normal);
					indirectLight = indirectLight * brdf / pdf / RussianRoulette;
				
					break;
				case SOURCE::SPECULAR_REFLECT:

					cosTheta = std::max(0.f, newRay.direction.dot(intersection.normal));
					brdf += intersection.m->eval_specular(-newRay.direction, -ray.direction, intersection.normal);
					if (intersection.m->kd.norm() > EPSILON)
						brdf += intersection.m->eval_diffuse(-newRay.direction, -ray.direction, intersection.normal, intersection) * cosTheta;
					indirectLight = indirectLight * brdf / pdf / RussianRoulette;
					break;
				case SOURCE::TRANSMISSON:
					indirectLight = indirectLight * intersection.m->tr / RussianRoulette;
					break;
				case SOURCE::REFLECT:
					indirectLight = indirectLight / RussianRoulette;
				}
			}
			
		}
		
		//计算直接光照
		/*float light_pdf;
		Intersection lightPos;
		pScene->sampleLight(lightPos, light_pdf);
		Vector3f obj2light = lightPos.coords - intersection.coords;
		Vector3f lightRay = obj2light.normalized();

		auto t = pScene->intersect(Ray(intersection.coords, lightRay));
		if (std::fabsf(t.distance - obj2light.norm()) < EPSILON)
		{
			
			Vector3f light_eval(0.f, 0.f, 0.f);
			float r2 = obj2light.dot(obj2light);
			float cosA = std::max(0.f, intersection.normal.dot(lightRay));
			float light_pdf_sa = light_pdf * r2 / lightPos.normal.dot(-lightRay);
			if (intersection.m->kd.norm() > EPSILON)
				light_eval += intersection.m->eval_diffuse(-lightRay, -ray.direction, intersection.normal, intersection) * cosA;
			if (intersection.m->ks.norm() > EPSILON)
				light_eval += intersection.m->eval_specular(-lightRay, -ray.direction, intersection.normal);
			directLight = lightPos.emit * light_eval / light_pdf_sa;


			
		}*/
		
		if(abs(intersection.m->Ni - 1) < EPSILON)
			directLight = directLight_MIS(ray, intersection);
		return  directLight + indirectLight;

	}
}

unsigned char* Renderer::Render()
{
	++renderTimes;

	if (renderTimes > intSampleNum) return frameBuffer;

	int width = pScene->camera->width, height = pScene->camera->height;
	Camera* camera = pScene->camera;

#pragma omp parallel for schedule(dynamic,1)
	for (int y = 0; y < height; ++y)
	{
		for (int x = 0; x < width; ++x)
		{
			Vector3f Color = Vector3f(0.f,0.f,0.f);
			Ray* rays = pScene->GetRays(x, y, pixelSampleNum);
			
			for (int i = 0; i < pixelSampleNum; ++i)
			{
				Vector3f res = trace(rays[i]);
				if (res.x() < 0.f) res.x() = 0.f;
				if (res.y() < 0.f) res.y() = 0.f;
				if (res.z() < 0.f) res.z() = 0.f;

				if (res.x() > 1.f) res.x() = 1.f;
				if (res.y() > 1.f) res.y() = 1.f;
				if (res.z() > 1.f) res.z() = 1.f;
				//gamma校正
				res.x() = std::pow(res.x(), 1 / 2.2);
				res.y() = std::pow(res.y(), 1 / 2.2);
				res.z() = std::pow(res.z(), 1 / 2.2);

				Color += res;
			}

			Color = Color / pixelSampleNum;
			delete[] rays;
			
			int index = 4 * x + 4 * y * width;
			frameBuffer[index] = (unsigned char)(((float)frameBuffer[index] * (renderTimes - 1) + Color.x()*255) / renderTimes);
			frameBuffer[index+1] = (unsigned char)(((float)frameBuffer[index+1] * (renderTimes - 1) + Color.y()*255) / renderTimes);
			frameBuffer[index+2] = (unsigned char)(((float)frameBuffer[index+2] * (renderTimes - 1) + Color.z()*255) / renderTimes);
		}
	}
	return frameBuffer;
}


Vector3f Renderer::directLight_MIS(Ray& ray, Intersection& intersection)
{
	Vector3f brdf_eval, light_eval;
	Vector3f dir_lit(0.f,0.f,0.f);
	Vector3f brdf_rgb(0.f, 0.f, 0.f), lit_rgb(0.f, 0.f, 0.f);


	Vector3f ret(0.f, 0.f, 0.f);
	//直接光源采样
	float light_pdf;
	float lit_pdf_sum=0;
	int lit_pdf_cnt=0;
	Intersection lightPos;
	for (int i = 0; i < pScene->lights.size(); i++)
	{
		
		for (int j = 0; j < MISSample; j++) {
			light_eval = Vector3f(0.f, 0.f, 0.f);
			pScene->sampleLight(lightPos, light_pdf, pScene->lights[i]);
			light_pdf /= pScene->lights.size();
			Vector3f obj2light = lightPos.coords - intersection.coords;
			Vector3f lightRay = obj2light.normalized();
			float dist2 = obj2light.dot(obj2light);
			//检查光线是否遮挡
			auto t = pScene->intersect(Ray(intersection.coords, lightRay));
			if (std::abs(t.distance - obj2light.norm()) < EPSILON)
			{
				float light_pdf_sa = light_pdf * dist2 / t.normal.dot(-lightRay);
				lit_pdf_sum = (lit_pdf_sum * lit_pdf_cnt + light_pdf) / (lit_pdf_cnt + 1);
				lit_pdf_cnt++;
				float cosA = std::max(0.f, intersection.normal.dot(lightRay));
				if(intersection.m->kd.norm() > EPSILON)
				light_eval += intersection.m->eval_diffuse(-lightRay, -ray.direction, intersection.normal, intersection) * cosA;
				if(intersection.m->ks.norm() > EPSILON)
				light_eval += intersection.m->eval_specular(-lightRay, -ray.direction, intersection.normal);
				lit_rgb += lightPos.emit * light_eval / light_pdf_sa;
			}
			
		}
	}
	if(lit_pdf_cnt)
	lit_rgb /= lit_pdf_cnt;
	
	//brdf采样
	float brdf_pdf_sum = 0;
	int brdf_pdf_cnt = 0;
	float brdf_pdf = 0;

	for (int i = 0; i < MISSample; ++i)
	{
		brdf_eval = Vector3f(0.f, 0.f, 0.f);
		Ray newRay = mcSample(ray, intersection, brdf_pdf);
		auto isec = pScene->intersect(newRay);
		if (isec.happened && isec.m->hasEmission() && abs(brdf_pdf) > EPSILON )
		{
			float brdf_pdf_sa = brdf_pdf * isec.distance * isec.distance / isec.normal.dot(-newRay.direction);
			brdf_pdf_sum = (brdf_pdf_sum * brdf_pdf_cnt + brdf_pdf) / (brdf_pdf_cnt + 1);
			brdf_pdf_cnt++;
			float cosA = std::max(0.f, intersection.normal.dot(newRay.direction));
			if (intersection.m->kd.norm() > EPSILON) brdf_eval += intersection.m->eval_diffuse(-newRay.direction, -ray.direction, intersection.normal, intersection) * cosA;
			if (intersection.m->ks.norm() > EPSILON) brdf_eval += intersection.m->eval_specular(-newRay.direction, -ray.direction, intersection.normal);

			brdf_rgb += isec.emit * brdf_eval / brdf_pdf_sa;
		}
	}
	if (brdf_pdf_cnt)
		brdf_rgb /= brdf_pdf_cnt;
	
	float lit_w = power_heuristic(lit_pdf_cnt, lit_pdf_sum, brdf_pdf_cnt, brdf_pdf_sum);
	float brdf_w = power_heuristic(brdf_pdf_cnt, brdf_pdf_sum, lit_pdf_cnt, lit_pdf_sum);
	dir_lit += lit_rgb * lit_w;
	dir_lit += brdf_rgb * brdf_w;
	return dir_lit;
}


void Renderer::ComputeProb(Ray& ray, const Intersection& I, sampleProb& prob)
{
	Material* m = I.m;
	float albedoDiffuse = luminance(m->kd);
	float albedoSpecular = luminance(m->ks);
	if (abs(m->Ni - 1) < EPSILON)
	{
		float totalAlbedo = albedoDiffuse + albedoSpecular;
		prob.diffuseProb = albedoDiffuse / totalAlbedo;
		prob.specularProb = albedoSpecular / totalAlbedo;
		prob.reflectProb = 0.f;
		prob.refractProb = 0.f;
		return;
	}
	float kr = 0;
	m->fresnel(ray.direction, I.normal, m->Ni, kr);
	float albedoReflect = kr;
	float albedoRefract = 1 - kr;
	prob.diffuseProb = 0.f;
	prob.specularProb = 0.f;
	prob.reflectProb = albedoReflect;
	prob.refractProb = albedoRefract;
	
}

