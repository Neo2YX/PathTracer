#pragma once
#include "Eigen/Eigen"
#include "Ray.h"
#include "utils.h"
class Camera
{
public:
	int width, height;
	float fov;

	Eigen::Vector3f position;
	Eigen::Vector3f lookAt;

	Eigen::Vector3f up;
	Eigen::Vector3f right;
	Eigen::Vector3f zAxis;


	Camera(int _width, int _height, float _fov, Eigen::Vector3f pos, Eigen::Vector3f look, Eigen::Vector3f _up) : width(_width), height(_height), fov(_fov), position(pos), lookAt(look), up(_up)
	{
		lookAt.normalize();
		right = lookAt.cross(up).normalized();
		up = right.cross(lookAt).normalized();
		zAxis = -lookAt.normalized();
	}

	inline Ray GetRay(int w, int h);
};

inline Ray Camera::GetRay(int w, int h)
{
	float x = RNG();
	float y = RNG();

	float halfH = std::tanf(fov * PI / 360.f);
	float aspect = (float)width / height;

	float rightVecScale = (2 * (w + x) / (float)width - 1) * aspect * halfH;
	float upVecScale = (1 - 2 * (h + y) / (float)height) * halfH;
	Eigen::Vector3f dir = rightVecScale * right + upVecScale * up + lookAt;
	dir.normalize();
	Ray* ret = new Ray(position, dir);
	return *ret;
}