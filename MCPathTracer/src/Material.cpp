#include "Material.h"
#include "Model.h"

float GetTexCoord(float a)
{
	if (a < 0) return GetTexCoord(100.f - a);
	return a - floor(a);
}

inline Vector3f Material::reflect(const Vector3f& InRay, const Vector3f& normal)
{
	return (InRay - 2 * InRay.dot(normal) * normal).normalized();
}

Vector3f Material::refract(const Vector3f& InRay, const Vector3f& normal, const float& ior)
{
	float cosi = -InRay.dot(normal);
	Vector3f n = normal;
	float eta_in = 1, eta_tr = ior;
	if (cosi < 0) {
		cosi = -cosi;
		n = -normal;
		std::swap(eta_in, eta_tr);
	}
	float eta = eta_in / eta_tr;
	float k = 1 - eta * eta * (1 - cosi * cosi);
	return (k < 0 ? Vector3f(0, 0, 0) : ((eta * cosi - std::sqrtf(k)) * n + eta * InRay));
}

void Material::fresnel(const Vector3f& InRay, const Vector3f& normal, const float& ior, float& kr) const
{
	float cosi = InRay.dot(normal);
	float etai = 1, etat = ior;
	if (cosi > 0) { std::swap(etai, etat); }

	float sint = etai / etat * sqrtf(std::max(0.f, 1 - cosi * cosi));

	if (sint >= 1) {
		kr = 1;
	}
	else {
		float cost = sqrtf(std::max(0.f, 1 - sint * sint));
		cosi = fabsf(cosi);
		float Rs = ((etat * cosi) - (etai * cost)) / ((etat * cosi) + (etai * cost));
		float Rp = ((etai * cosi) - (etat * cost)) / ((etai * cosi) + (etat * cost));
		kr = (Rs * Rs + Rp * Rp) / 2;
	}
}

inline Vector3f Material::toWorld(const Vector3f& a, const Vector3f& N)
{
	Vector3f B, C;
	if (std::fabs(N.x()) > std::fabs(N.y())) {
		float invLen = 1.0f / std::sqrt(N.x() * N.x() + N.z() * N.z());
		C = Vector3f(N.z() * invLen, 0.0f, -N.x() * invLen);
	}
	else {
		float invLen = 1.0f / std::sqrt(N.y() * N.y() + N.z() * N.z());
		C = Vector3f(0.0f, -N.z() * invLen, N.y() * invLen);
	}
	B = N.cross(C);
	return a.x() * B + a.y() * N + a.z() * C;
}


Vector3f Material::CosineWeightImportanceSample(const Vector3f& wi, const Vector3f& N)
{
	float ep1 = RNG();
	float ep2 = RNG();

	float theta = std::acosf(1 - 2 * ep1) * 0.5;
	float phi = 2 * PI * ep2;

	float x = std::sinf(theta) * std::cosf(phi);
	float z = std::sinf(theta) * std::sinf(phi);
	float y = cosf(theta);

	Vector3f dir = toWorld(Vector3f(x, y, z), N);

	return dir.normalized();
}

Vector3f Material::specularWeightImportanceSample(const Vector3f& wi, const Vector3f& N, float Ns)
{
	
	float ep1 = RNG();
	float ep2 = RNG();

	float theta = std::acosf(std::powf(ep1, 1.0 / (Ns + 1)));
	float phi = 2 * PI * ep2;

	float x = std::sinf(theta) * std::cosf(phi);
	float z = std::sinf(theta) * std::sinf(phi);
	float y = cosf(theta);

	Vector3f reflectDir = reflect(wi, N);
	Vector3f dir = toWorld(Vector3f(x, y, z), reflectDir);

	
	return dir.normalized();
}

float Material::pdf_cosine(const Vector3f& wi, const Vector3f& wo, const Vector3f& N)
{
	if (wo.dot(N) > 0) return wo.normalized().dot(N) / PI;
	else return 0;
}

float Material::pdf_specular(const Vector3f& wi, const Vector3f& wo, const Vector3f& N, float Ns)
{
	Vector3f reflectDir = reflect(wi, N).normalized();
	if (wo.dot(N) > 0) return (Ns + 1) * std::powf(wo.normalized().dot(reflectDir), Ns) / PI * 0.5;
	else return 0;
}

Vector3f Material::eval_diffuse(const Vector3f& wi, const Vector3f& wo, const Vector3f& N, Intersection& I)
{
	if (wo.dot(N) <= 0) return Vector3f(0.f,0.f,0.f);
	if (!hasTex) return kd / PI;

	Eigen::Vector2f UV = ((Triangle*)I.obj)->GetUVAt(I.coords);
	UV.x() = GetTexCoord(UV.x());
	UV.y() = GetTexCoord(UV.y());
	Vector3f TexColor = getColorAt(UV.x(), UV.y());
	return TexColor / PI;
}
Vector3f Material::eval_specular(const Vector3f& wi, const Vector3f& wo, const Vector3f& N)
{
	Eigen::Vector3f halfV = (-wi + wo).normalized();
	float cos = halfV.dot(N);
	return std::pow(cos, Ns) * (Ns + 2) / (2.0 * PI) * ks;
}

Vector3f Material::eval_light(const Vector3f& wi, const Vector3f& wo, const Vector3f& N, Intersection& I)
{
	Vector3f res(0.f,0.f,0.f);
	if (kd.norm() > EPSILON) {
		res += eval_diffuse(wi, wo, N, I);
	}
	if (ks.norm() > EPSILON)
	{
		res += eval_specular(wi, wo, N);
	}

	return res;
}

Vector3f Material::getColorAt(float _u, float _v)
{
	_u = std::max(0.f, std::min(1.f, _u));
	_v = std::max(0.f, std::min(1.f, _v));
	int u = (int)(_u*Tex.width), v = (int)(_v*Tex.height);
	return Vector3f((float)Tex.imag[u * Tex.nChannel + v * Tex.width * Tex.nChannel] / 255.f, (float)Tex.imag[u * Tex.nChannel + v * Tex.width * Tex.nChannel + 1] / 255.f, (float)Tex.imag[u * Tex.nChannel + v * Tex.width * Tex.nChannel + 2] / 255.f);
}