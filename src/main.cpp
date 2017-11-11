#include <stdio.h>
#include <string.h>
#include <malloc.h>

#include "utils.h"
#include "gl_helper.h"
#include "mobile_fusion.h"
#include "tsdf_model.h"
#include "frame_pool.h"
#include "stereo_match.h"
#include "pose_estimation.h"

#include "dataset_helper.h"

using namespace jhw_gl;
using namespace mf;

int main() {
	const GlobalCoeff G450(450, 375, 500.0, 500.0, 225.0, 175.0, 3.0);

	GlobalCoeff G = G450;
	GLInit(G.g_w, G.g_h);

	DatasetHelper DH;
	//DH.init((const char *)"../datasets/dataset0", G);
	DH.init((const char *)"res", G);
	/*
	Frame f; 
	MobileFusion mobile_fusion;
	mobile_fusion.init(G);
	
	for (int i = 0; i < 50; i+=2) {
		f = DH.readFrame(i);
		if (i < 20)
			mobile_fusion.addFrameAndPose(f);
		else {
			mobile_fusion.addFrame(f.getImage());
		}
	}
	DH.writeModel(mobile_fusion.getModelC(), mobile_fusion.getModelSW(), 10);

	return 0;
	*/
	Frame &f1 = DH.readFrame(2);
	Frame &f2 = DH.readFrame(6);
	//Frame &f3 = DH.readFrame(16);
	//Frame &f4 = DH.readFrame(18);
	//Frame &f5 = DH.readFrame(20);

	Model tsdf_model;
	tsdf_model.init(G.g_w, G.g_h);

	FramePool frames;
	frames.init(G, tsdf_model.getVertex());

	StereoMatch stereo_match;
	stereo_match.init(G.g_w, G.g_h, "ZNCC");

	MySolver solver;

	Frame rec1, rec2;
	frames.Rectification(f1, f2);
	frames.getRectPair(rec1, rec2);
	
	stereo_match.DisparityEstimation(rec1.getImage(), rec2.getImage(), 3);
	u32 *depth = stereo_match.LRCheckAndGetDepth(5, frames.getBaseline(), G.g_fx);
	u32 *delta1 = stereo_match.getDisparity1();
	u32 *delta2 = stereo_match.getDisparity2();

	DH.writeImage(stereo_match.u32DepthToGray(delta1, 8.f), "delta1.png");
	DH.writeImage(stereo_match.u32DepthToGray(delta2, 8.f), "delta2.png");
	DH.writeImage(stereo_match.u32DepthToGray(depth, 20.f), "depth.png");
	 
	tsdf_model.ModelUpdating(rec1.getImage(), depth, rec1.getPose());
/*
	u32 *i, *d, *y;
	tsdf_model.RayTracing(f5.getPose(), &i, &d, &y);
	u32 *ii = (u32*)malloc(sizeof(u32) * G.g_w * G.g_h);
	for (int x = 10; x < G.g_h; ++x) {
		for (int y = 0; y < G.g_w; ++y) {
			ii[x*G.g_w + y] = binterd_u32(i, y, x-9.5, G.g_w);
		}
	}
	DH.writeImage(ii, "ii.png");
	DH.writeImage(i, "iii.png");
	DH.writeImage(stereo_match.u32DepthToGray(d), "ddd.png");
	DH.writeModel(tsdf_model.getModelC(), tsdf_model.getModelSW(), 9);
*/
	return 0;
	/*
	stbi_write_png("res/depth.png", g_w, g_h, 4, stereo_match.u32DepthToGray(depth, 10.f), sizeof(u32) * g_w);

	tsdf_model.ModelUpdating(rec1.getImage(), depth, rec1.getPose());
	//tsdf_model.ModelUpdating(rec1.getImage(), depth, rec1.getPose());
	//tsdf_model.ModelUpdating(rec1.getImage(), depth, rec1.getPose());
	u32 *i, *d, *y;
	tsdf_model.RayTracing(rec1.getPose(), &i, &d, &y);
	stbi_write_png("res/image.png", g_w, g_h, 4, i, sizeof(u32) * g_w);

	frames.Rectification(f2, f3);
	frames.getRectPair(rec1, rec2);
	stereo_match.DisparityEstimation(rec1.getImage(), rec2.getImage());
	depth = stereo_match.LRCheckAndGetDepth(5, frames.getBaseline(), g_fx);
	tsdf_model.ModelUpdating(rec1.getImage(), depth, rec1.getPose());
	//tsdf_model.ModelUpdating(rec1.getImage(), depth, rec1.getPose());
	//tsdf_model.ModelUpdating(rec1.getImage(), depth, rec1.getPose());

	frames.Rectification(f3, f4);
	frames.getRectPair(rec1, rec2);
	stereo_match.DisparityEstimation(rec1.getImage(), rec2.getImage());
	depth = stereo_match.LRCheckAndGetDepth(5, frames.getBaseline(), g_fx);
	tsdf_model.ModelUpdating(rec1.getImage(), depth, rec1.getPose());
	//tsdf_model.ModelUpdating(rec1.getImage(), depth, rec1.getPose());
	//tsdf_model.ModelUpdating(rec1.getImage(), depth, rec1.getPose());

	frames.Rectification(f4, f5);
	frames.getRectPair(rec1, rec2);
	stereo_match.DisparityEstimation(rec1.getImage(), rec2.getImage());
	depth = stereo_match.LRCheckAndGetDepth(5, frames.getBaseline(), g_fx);
	tsdf_model.ModelUpdating(rec1.getImage(), depth, rec1.getPose());
	//tsdf_model.ModelUpdating(rec1.getImage(), depth, rec1.getPose());
	//tsdf_model.ModelUpdating(rec1.getImage(), depth, rec1.getPose());

	/*
	return 0;
	

	MobileFusion mobile_fusion;
	mobile_fusion.init(g_w, g_h, 3.75, Intrinsic);

	for (int i = 0; i < 100; i++) {
		printf("index: %d\n", i);

		R = Eigen::AngleAxisd(M_PI / 5 - i * 0.01, Eigen::Vector3d(0.0, 1.0, 0.0).normalized()).toRotationMatrix();
		pose = mf::CameraPose(Intrinsic, R, t);

		char name_tmp[128];
		sprintf(name_tmp, "../datasets/dataset0/im%02d.png", i);
		u32* image = (u32*)stbi_load(name_tmp, &xx, &yy, &nn, 4);
		Frame f(image, pose);
		//if (i < 522)
			mobile_fusion.addFrameAndPose(f);
		//else {
		//	mobile_fusion.addFrame(image);
		//}
		
	}

	stbi_write_png("E:/code/c/MobileFusion/small_resolution/QtGuiApplication/res/modelC8.png", 4096, 4096, 4, mobile_fusion.getModelC(), sizeof(u32) * 4096);
	stbi_write_png("E:/code/c/MobileFusion/small_resolution/QtGuiApplication/res/modelSW8.png", 4096, 4096, 4, mobile_fusion.getModelSW(), sizeof(u32) * 4096);

	mobile_fusion.destroy();
	*/
#if 0
	GLInit(320, 240);

	MobileFusion my_MF;

	my_MF.init();
	CameraPose	pose;
	CameraPose	last_pose(my_MF.CameraCoeff, 0.1, 0.78, 0.1, 0.0, 0, 8);
	u32*		I;
	u32			*I_M, *D_M, *Y_M;
	char		name_tmp[1024];
	char		dataset_name[128] = "dataset1";
	int			xx, yy, nn;


	for (int i = 0; i < 150; i++) {
		printf("index: %d\n", i);

		pose = CameraPose(my_MF.CameraCoeff, 0.1, 0.78 + i*0.01, 0.1, 0.0, 0, 8);

		sprintf(name_tmp, "../datasets/%s/im%02d.png", dataset_name, i);
		I = (u32*)stbi_load(name_tmp, &xx, &yy, &nn, 4);

		my_MF.addFrameAndPose(I, pose);
#if 0
		u32 *C = my_MF.getModelC();
		u32 *SW = my_MF.getModelSW();
		
		for (int ii = 0; ii < 4096; ++ii) {
			for (int jj = 0; jj < 4096; ++jj) {
				if (C[ii * 4096 + jj] != 0) {
					//printf("C\n");
					C[ii * 4096 + jj] |= 0xff000000;
				}
				if (SW[ii * 4096 + jj] != 132) {
					//printf("%0x SW\n", SW[ii * 4096 + jj]);
					SW[ii * 4096 + jj] |= 0xff000000;
				}
			}
		}
		

		stbi_write_png("res/C.png", 4096, 4096, 4, C, sizeof(u32) * 4096);
		stbi_write_png("res/SW.png", 4096, 4096, 4, SW, sizeof(u32) * 4096);
#endif

		my_MF.RayTracing(last_pose);
		
		if (my_MF.getDisplay(&I_M, &D_M, &Y_M)) {
			stbi_write_png("res/II.png", 320, 240, 4, I_M, sizeof(u32) * 320);
			stbi_write_png("res/DD.png", 320, 240, 4, D_M, sizeof(u32) * 320);
			stbi_write_png("res/YY.png", 320, 240, 4, Y_M, sizeof(u32) * 320);
		}
	}
	my_MF.destroy();

#endif
	return 0;
}
