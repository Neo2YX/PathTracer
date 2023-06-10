#include<iostream>
#include<random>
#include "Model.h"
#include "Renderer.h"
#include "Window.h"

#define STB_IMAGE_STATIC
#define STB_IMAGE_WRITE_IMPLEMENTATION
#include "stb_image_write.h"
void DrawCornellBox(int framecnt, bool isDebug = false)
{
	Model model("cornell-box.obj", "Models/CornellBox/");
	Camera camera(1024, 1024, 39.3077, Eigen::Vector3f(278, 273, -800), Eigen::Vector3f(0, 0, 1), Eigen::Vector3f(0.f, 1.f, 0.f));

	Scene scene(&camera, &model);
	//添加scene，light
	for (auto& m : model.meshes)
	{
		scene.AddMesh(&m);

		if (m.Mat->hasEmission())
		{
			Print("添加光源：" << m.Mat->name);
			Light* l = new AreaLight(m.name, m.Mat->emis);
			((AreaLight*)l)->AddMesh(&m);
			scene.AddLight(l);
		}
	}
	scene.init();
	Renderer* r = new Renderer(&scene);
	//render
	InitWindow(camera.width, camera.height);

	int frame_cnt = 0;
	while (myWindow->window_close_ == false)
	{
		frame_cnt++;
		start = clock();
		DrawWindow(r->Render());
		end = clock();

		Print("绘制connell-box第" << frame_cnt << "帧用时：" << (float)(end - start) / CLOCKS_PER_SEC << "s");
		
		if (frame_cnt == framecnt) {
			if (isDebug) __debugbreak();
			stbi_write_bmp("CornellBox_result.jpg", camera.width, camera.height, 4, r->frameBuffer);
			break;
		}
	}
	CloseWindow();

}

void DrawVeachMIS(int framecnt, bool isDebug = false)
{
	Model model("veach-mis.obj", "Models/veach-mis/");
	Camera camera(1280, 720, 20.1143, Eigen::Vector3f(28.2792, 5.2, 1.23612e-06), Eigen::Vector3f(-28.2792, -2.4, -1.23612e-06), Eigen::Vector3f(0.f, 1.f, 0.f));

	Scene scene(&camera, &model);
	//添加scene，light
	for (auto& m : model.meshes)
	{
		scene.AddMesh(&m);

		if (m.Mat->hasEmission())
		{
			Print("添加光源：" << m.Mat->name);
			Light* l = new AreaLight(m.name, m.Mat->emis);
			((AreaLight*)l)->AddMesh(&m);
			scene.AddLight(l);
		}
	}
	scene.init();
	Renderer* r = new Renderer(&scene);
	//render
	InitWindow(camera.width, camera.height);

	int frame_cnt = 0;
	while (myWindow->window_close_ == false)
	{
		frame_cnt++;
		start = clock();
		DrawWindow(r->Render());
		end = clock();

		Print("绘制veach-mis第" << frame_cnt << "帧用时：" << (float)(end - start) / CLOCKS_PER_SEC << "s");
		if (frame_cnt == framecnt) {
			if (isDebug) __debugbreak();
			stbi_write_bmp("VeachMIS_result.jpg", camera.width, camera.height, 4, r->frameBuffer);
			break;
		}
	}
	CloseWindow();

}

void DrawStairscase(int framecnt, bool isDebug = false)
{
	Model model("stairscase.obj", "Models/staircase/");
	Camera camera(1280, 720, 42.9957, Eigen::Vector3f(6.9118194580078125f, 1.6516278982162476f, 2.5541365146636963f), Eigen::Vector3f(-4.583800077438354f, -0.0000002284185791f, -2.21773192286491f), Eigen::Vector3f(0.f, 1.f, 0.f));

	Scene scene(&camera, &model);
	//添加scene，light
	for (auto& m : model.meshes)
	{
		scene.AddMesh(&m);

		if (m.Mat->hasEmission())
		{
			Print("添加光源：" << m.Mat->name);
			Light* l = new AreaLight(m.name, m.Mat->emis);
			((AreaLight*)l)->AddMesh(&m);
			scene.AddLight(l);
		}
	}
	scene.init();
	Renderer* r = new Renderer(&scene);
	//render
	InitWindow(camera.width, camera.height);

	int frame_cnt = 0;
	while (myWindow->window_close_ == false)
	{
		frame_cnt++;
		start = clock();
		DrawWindow(r->Render());
		end = clock();

		Print("绘制stairscase第" << frame_cnt << "帧用时：" << (float)(end - start) / CLOCKS_PER_SEC << "s");
		if (frame_cnt == framecnt) {
			if (isDebug) __debugbreak();
			stbi_write_bmp("Staircase_result.jpg", camera.width, camera.height, 4, r->frameBuffer);
			break;
		}
	}
	CloseWindow();

}



int main()
{
	//DrawCornellBox(100);
	//DrawVeachMIS(100);
	//DrawStairscase(100);
	
	


	return 0;
}