#include "BVH.h"
#include <algorithm>
#include <assert.h>

BVH::BVH(const std::vector<Object*>& p, int maxPrimsInNode, SplitMethod splitMethod) : primitives(std::move(p)), maxPrimsInNode(maxPrimsInNode), splitMethod(splitMethod)
{
	if (primitives.empty()) return;
	root = recursiveBuild(primitives);
}

BVHNode* BVH::recursiveBuild(std::vector<Object*>objects)
{
	BVHNode* ret = new BVHNode();

	//合并所有的AABB
	AABB Box;
	for (int i = 0; i < objects.size(); ++i)
		Box = Merge(Box, objects[i]->getAABB());
	if (objects.size() == 1) {
		ret->Box = objects[0]->getAABB();
		ret->Left = nullptr;
		ret->Right = nullptr;
		ret->obj = objects[0];
		ret->area = objects[0]->getArea();
		return ret;
	}
	else if (objects.size() == 2) {
		ret->Left = recursiveBuild(std::vector<Object*>{objects[0]});
		ret->Right = recursiveBuild(std::vector<Object*>{objects[1]});

		ret->Box = Merge(ret->Left->Box, ret->Right->Box);
		ret->area = ret->Left->area + ret->Right->area;
		return ret;
	}
	else {
		AABB CenterBox;
		for (int i = 0; i < objects.size(); ++i)
			CenterBox = Merge(CenterBox, (objects[i]->getAABB()).GetCenter());
		int splitAxis = CenterBox.maxAxis();

		//对objects中所有的物体进行排序，并取中值划分新的层次
		switch (splitAxis)
		{
		case 0:
			std::sort(objects.begin(), objects.end(), [](auto f1, auto f2) {
				return f1->getAABB().GetCenter().x() < f2->getAABB().GetCenter().x();
				});
			break;
		case 1:
			std::sort(objects.begin(), objects.end(), [](auto f1, auto f2) {
				return f1->getAABB().GetCenter().y() < f2->getAABB().GetCenter().y();
				});
			break;
		case 2:
			std::sort(objects.begin(), objects.end(), [](auto f1, auto f2) {
				return f1->getAABB().GetCenter().z() < f2->getAABB().GetCenter().z();
				});
			break;
		}

		auto begin = objects.begin();
		auto end = objects.end();
		auto middle = objects.begin() + objects.size() / 2;

		auto leftSubObj = std::vector<Object*>(begin, middle);
		auto rightSubObj = std::vector<Object*>(middle, end);

		assert(objects.size() == (leftSubObj.size() + rightSubObj.size()));

		ret->Left = recursiveBuild(leftSubObj);
		ret->Right = recursiveBuild(rightSubObj);

		ret->Box = Merge(ret->Left->Box, ret->Right->Box);
		ret->area = ret->Left->area + ret->Right->area;
		return ret;
	}
}

Intersection BVH::Intersect(const Ray& ray) const
{
	Intersection I;
	if (!root) return I;
	I = getIntersection(root, ray);
	return I;
}

Intersection BVH::getIntersection(BVHNode* node, const Ray& ray)const
{
	Intersection I;
	std::array<int, 3> dirIsNeg;
	dirIsNeg[0] = int(ray.direction.x() >= 0);
	dirIsNeg[1] = int(ray.direction.y() >= 0);
	dirIsNeg[2] = int(ray.direction.z() >= 0);

	if (!node->Box.IntersectP(ray, ray.direction_inv, dirIsNeg)) return I;

	if (node->Left == nullptr && node->Right == nullptr)
	{
		I = node->obj->getIntersection(ray);
		return I;
	}

	auto hit1 = getIntersection(node->Left, ray);
	auto hit2 = getIntersection(node->Right, ray);

	return hit1.distance < hit2.distance ? hit1 : hit2;
}

void BVH::getSample(BVHNode* node, float p, Intersection& pos, float& pdf) {
	if (node->Left == nullptr || node->Right == nullptr) {
		node->obj->Sample(pos, pdf);
		pdf *= node->area;
		return;
	}
	if (p < node->Left->area) getSample(node->Left, p, pos, pdf);
	else getSample(node->Right, p - node->Left->area, pos, pdf);
}

void BVH::Sample(Intersection& pos, float& pdf) {
	float p = std::sqrt(RNG()) * root->area;
	getSample(root, p, pos, pdf);
	pdf /= root->area;
}