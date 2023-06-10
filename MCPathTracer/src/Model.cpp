#include "Model.h"
#include <fstream>
#include <ctime>

clock_t start, end;



Intersection Triangle::getIntersection(Ray ray)
{
	Intersection I;
	double u, v, t;
	if (ray.direction.dot(n0) >= 0) return I;
	Eigen::Vector3f e1 = v1 - v0;
	Eigen::Vector3f e2 = v2 - v0;
	Eigen::Vector3f S1 = ray.direction.cross(e2);
	double det = S1.dot(e1);
	if (fabs(det) < EPSILON || det < 0) return I;
	double det_inv = 1. / det;
	Eigen::Vector3f S = ray.origin - v0;
	u = S1.dot(S) * det_inv;
	if (u < 0 || u > 1) return I;
	Eigen::Vector3f S2 = S.cross(e1);
	v = S2.dot(ray.direction) * det_inv;
	if (v < 0 || v > 1) return I;
	t = S2.dot(e2) * det_inv;

	if (t <= 0) return I;
	I.happened = true;
	I.coords = ray(t);
	I.m = this->mat;
	I.normal = n0;
	I.distance = t;
	I.obj = this;
	I.emit = I.m->emis;

	return I;
}

static bool loadMtl(const std::string& filename, const std::string& filepath, std::map<std::string, Material>& MatTable)
{
	std::ifstream file(filepath+filename);
	if (!file.is_open()) {
		std::cout << "读取材质文件" << filename << "失败！" << std::endl;
		return false;
	}

	std::string MatName;
	Material Mat;

	std::string type;
	std::string TexPath;
	

	bool flag = false;
	while (file >> type)
	{
		if (type == "newmtl") {
			if (flag) {
				MatTable.insert(std::make_pair(MatName, Mat));
				Mat = Material();
			}
			else flag = true;

			file >> MatName;
			Mat.name = MatName;
		}
		else if (type == "Kd")
		{
			file >> Mat.kd.x() >> Mat.kd.y() >> Mat.kd.z();
		}
		else if (type == "Ks")
		{
			file >> Mat.ks.x() >> Mat.ks.y() >> Mat.ks.z();
		}
		else if (type == "Tr")
		{
			file >> Mat.tr.x() >> Mat.tr.y() >> Mat.tr.z();
		}
		else if (type == "Ns")
		{
			file >> Mat.Ns;
		}
		else if (type == "Ni")
		{
			file >> Mat.Ni;
		}
		else if (type == "Ke")
		{
			file >> Mat.emis.x() >> Mat.emis.y() >> Mat.emis.z();
		}
		else if (type == "map_Kd")
		{
			file >> TexPath;
			Mat.Tex = Texture(filepath + TexPath);
			Mat.hasTex = true;
		}
		else file.ignore((std::numeric_limits<std::streamsize>::max)(), '\n');
	}
	if(flag) MatTable.insert(std::make_pair(MatName, Mat));
	return true;
}

bool Model::load(const std::string& filename, const std::string& filepath)
{
	std::ifstream file(filepath + filename);

	if (!file.is_open())
	{
		std::cout << "读取obj文件" << filename << "失败！" << std::endl;
		return false;
	}

	bool ret = true;
	int verticesInd[3];
	int normalsInd[3];
	int uvsInd[3];
	std::string name, type;
	Eigen::Vector3f v;
	Eigen::Vector2f u;
	bool flag = false;
	Mesh mesh;
	Triangle* t;

	float all_area;

	while (file >> type)
	{
		if (type == "mtllib") {
			file >> name;
			if (!loadMtl(name, filepath, mats))
			{
				ret = false;
				std::cout << "读取mtl文件" << name << "失败！" << std::endl;
				break;
			}

		}
		else if (type == "v")
		{
			file >> v.x() >> v.y() >> v.z();
			vertices.push_back(v);
		}
		else if (type == "vn")
		{
			file >> v.x() >> v.y() >> v.z();
			normals.push_back(v.normalized());
		}
		else if (type == "vt")
		{
			file >> u.x() >> u.y();
			UVs.push_back(u);
		}
		else if (type == "g") {
			if (!flag) {
				flag = true;
				all_area = 0;
				continue;
			}
			meshes.push_back(mesh);
			mesh.TriList.clear();
			areas.push_back(all_area);
			all_area = 0;
		}
		else if (type == "usemtl")
		{
			file >> name;
			std::map<std::string, Material>::iterator it = mats.find(name);
			if (it != mats.end())
			{
				mesh.Mat = &(it->second);
			}
			else {
				std::cout << "加载" << name << "材质失败！" << std::endl;
				ret = false;
				break;
			}
		}
		else if (type == "f") //顶点坐标 / 贴图坐标 / 法线坐标
		{
			mesh.name = mesh.Mat->name;
			char ch;
			for (int i = 0; i < 3; ++i)
			{
				file >> verticesInd[i];
				--verticesInd[i];
				file >> ch;
				assert(ch == '/');
				file >> uvsInd[i];
				--uvsInd[i];
				file >> ch;
				assert(ch == '/');
				file >> normalsInd[i];
				--normalsInd[i];
			}
			t = new Triangle(vertices[verticesInd[0]], vertices[verticesInd[1]], vertices[verticesInd[2]], normals[normalsInd[0]], normals[normalsInd[1]], normals[normalsInd[2]], UVs[uvsInd[0]], UVs[uvsInd[1]], UVs[uvsInd[2]], mesh.Mat);
			mesh.TriList.push_back(t);
			all_area += t->area;
		}
		else if (type == "debug")
		{
			//debug
			std::string s_tmp;
			std::getline(file, s_tmp);
			Print("debug: " << s_tmp);
		}
		else {
			ret = true;
			std::cout << "读取obj文件中遇到不明字符！" << std::endl;
			break;
		}
	}
	if (flag)
	{
		meshes.push_back(mesh);
		mesh.TriList.clear();
		areas.push_back(all_area);
	}
	/*  取消在读入模型的时候构造bvh，而是在加载入场景后再进行构造
	if (meshes.size())
	{
		for (int i = 0; i < meshes.size(); ++i)
		{
			BVH* bvh = new BVH(meshes[i].TriList);
			bvhs.push_back(bvh);
		}
	}
	*/
	file.close();
	return ret;
}

Model::Model(const std::string& filename, const std::string& filepath)
{
	start = clock();
	Print("开始加载模型");
	if (load(filename, filepath)) {
		Print("模型" << filename << "加载成功");
		end = clock();
		double time = (double)(end - start) / CLOCKS_PER_SEC;
		Print("总共用时：" << time << "s");
		//debug
		deb_PrintInfo();
	}
	else {
		Print("模型" << filename << "加载失败");
	}
}

void Model::deb_PrintInfo()
{
	Print("num of vertex: " << vertices.size());
	Print("num of uvs: " << UVs.size());
	Print("num of normals: " << normals.size());
	Print("num of meshes: " << meshes.size());
	Print("num of Mats: " << mats.size());
	Print("Meshes name: ");
	for (int i = 0; i < meshes.size(); ++i)
	{
		Print(meshes[i].name);
	}
}


//Moller Trumbore Algorithm
Intersection Triangle::getIntersection(Ray ray, float tnear)
{
	Intersection ret;

	if (ray.direction.dot(n0) > 0) return ret;

	double t, b1, b2;
	Eigen::Vector3f e1 = v1 - v0;
	Eigen::Vector3f e2 = v2 - v0;
	Eigen::Vector3f S1 = ray.direction.cross(e2);
	float det = S1.dot(e1);
	if (fabsf(det) < EPSILON) return ret;
	float det_inv = 1. / det;
	Eigen::Vector3f S = ray.origin - v0;
	b1 = S1.dot(S) * det_inv;
	if (b1 < 0 || b1 > 1) return ret;
	Eigen::Vector3f S2 = S.cross(e1);
	b2 = S2.dot(ray.direction) * det_inv;
	if (b2 < 0 || b2 > 1) return ret;
	t = S2.dot(e2) * det_inv;

	if (t < tnear) return ret;

	ret.happened = true;
	ret.coords = ray(t);
	ret.m = mat;
	ret.distance = t;
	ret.normal = GetNormalAt(ret.coords);
	ret.obj = this;

	return ret;

}

Eigen::Vector3f Triangle::evalDiffuseColor(const Vector2f&) const
{
	return Eigen::Vector3f(0.5f, 0.5f, 0.5f);
}

bool Model::intersect(const Ray& ray, float& tnear,
	uint32_t& index) const
{
	bool ret = false;
	for (uint32_t i = 0; i < meshes.size(); ++i)
	{
		for (uint32_t j = 0; j < meshes[i].TriList.size(); ++j)
		{
			float t, u, v;
			Triangle* tri = (Triangle*)meshes[i].TriList[j];
			if (rayTriangleIntersect(tri->v0, tri->v1, tri->v2, ray.origin, ray.direction, t, u, v) && (t < tnear)) {
				tnear = t;
				index = i + j * 100; // 后两位为mesh序号，前面的则是mesh中的TriList序号
				ret |= true;
			}
		}
	}
	return ret;
}