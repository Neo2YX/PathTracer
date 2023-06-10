#pragma once
#include "AABB.h"
#include "Object.h"
#include <vector>
#include "utils.h"


struct BVHNode {
	AABB Box;
	BVHNode* Left;
	BVHNode* Right;
	Object* obj;
	float area;

public:
	int splitAxis = 0, firstPrimOffset = 0, nPrimitives = 0;
	BVHNode()
	{
		Box = AABB();
		Left = nullptr; Right = nullptr; obj = nullptr;
	}
};


class BVH
{
public:
	enum class SplitMethod { NAIVE, SAH };

	BVH(const std::vector<Object*>& p, int maxPrimsInNode =1, SplitMethod splitMethod = SplitMethod::NAIVE);
	AABB WorldBound() const { return root->Box; }
	~BVH();

	Intersection Intersect(const Ray& ray) const;
	Intersection getIntersection(BVHNode* node, const Ray& ray)const;
	bool IntersectP(const Ray& ray) const;
	BVHNode* root;

	BVHNode* recursiveBuild(std::vector<Object*>objects);

	const int maxPrimsInNode;
	const SplitMethod splitMethod;
	std::vector<Object*> primitives;

	void getSample(BVHNode* node, float p, Intersection& pos, float& pdf);
	void Sample(Intersection& pos, float& pdf);
};