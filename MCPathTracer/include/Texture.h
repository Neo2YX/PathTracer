#pragma once
#include <string>
#include "Eigen/Eigen"
#define STB_IMAGE_STATIC
#define STB_IMAGE_IMPLEMENTATION
#include "stb_image.h"

class Texture
{
public:
	unsigned char* imag = nullptr;
	int width;
	int height;
	int nChannel;

	Texture(){}
	Texture(const std::string& texPath)
	{
		imag = stbi_load(texPath.c_str(), &width, &height, &nChannel, 0);
		if (!imag) std::cout << "¶ÁÈ¡ÌùÍ¼" << texPath << "Ê§°Ü£¡" << std::endl;
		else { Print("¶ÁÈ¡ÌùÍ¼" << texPath << "³É¹¦£¡"); DebugPrint(); }
	}

	void DebugPrint()
	{
		Print("width * height: " << width << " * " << height);
		Print("nChannel: " << nChannel);
		Print(_msize(imag));
	}

};