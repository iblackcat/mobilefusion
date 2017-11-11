#include "dataset_helper.h"

#ifndef STB_IMAGE_IMPORT
#define STB_IMAGE_IMPORT
#define STB_IMAGE_IMPLEMENTATION 1
#define STB_IMAGE_WRITE_IMPLEMENTATION 1
#include "stb_image.h"
#include "stb_image_write.h"
#endif

namespace mf {

const GlobalCoeff G640(640, 480, 700.0, 700.0, 313.0, 256.0, 3.67);
const GlobalCoeff G320(320, 240, 500.0, 500.0, 150.0, 100.0, 3.0);

Frame DatasetHelper::readFrame(int index, int channels) {
	int w, h, n;
	char name_tmp[1024];
	sprintf(name_tmp, "%s/im%02d.png", path, index);
	u32* image = (u32*)stbi_load(name_tmp, &w, &h, &n, channels);

	if (!image || w != G.g_w || h != G.g_h){// || n != channels) {
		std::cerr << "Read image failed!" << std::endl;
		std::cerr << "Your choose : w = " << G.g_w << ",\th = " << G.g_h << ",\tchannels = " << channels << std::endl;
		std::cerr << "But read    : w = " << w << ",\th = " << h << ",\tchannels = " << n << std::endl;
	}

	sprintf(name_tmp, "%s/pose%02d.txt", path, index);
	FILE *fp = fopen(name_tmp, "r");

	Vector6d se3;
	fscanf(fp, "%lf %lf %lf %lf %lf %lf", &se3[0], &se3[1], &se3[2], &se3[3], &se3[4], &se3[5]);
	CameraPose pose(G.Intrinsic, se3);
	fclose(fp);

	Frame f(image, pose);
	return f;
}

bool DatasetHelper::writeImage(void *i, const char * file_name, int channels) {
	if (!i) {
		return false;
	}
	char name_tmp[256];
	sprintf(name_tmp, "res/%s", file_name);
	if (channels == 4)
		stbi_write_png(name_tmp, G.g_w, G.g_h, channels, i, sizeof(u32) * G.g_w);
	else if (channels == 1)
		stbi_write_png(name_tmp, G.g_w, G.g_h, channels, i, sizeof(u8) * G.g_w);
	return true;
}

bool DatasetHelper::writeModel(u32 *C, u32 *SW, int index) {
	if (!C || !SW) {
		return false;
	}
	char name_tmp[256];
	sprintf(name_tmp, "E:/code/c/MobileFusion/small_resolution/QtGuiApplication/res/modelC%d.png", index);
	stbi_write_png(name_tmp, 4096, 4096, 4, C, sizeof(u32) * 4096);
	sprintf(name_tmp, "E:/code/c/MobileFusion/small_resolution/QtGuiApplication/res/modelSW%d.png", index);
	stbi_write_png(name_tmp, 4096, 4096, 4, SW, sizeof(u32) * 4096);
	return true;
}

} // namespace mf
