#include "tsdf_model.h"

namespace mf {

inline double Model::
LocalAxis(double tmp) {
	return tmp * ModelSize / Size + ModelSize / 2;
}

inline double Model::
WorldAxis(double tmp) {
	return (tmp - ModelSize / 2)*Size / ModelSize;
}

inline Eigen::Vector3d Model::
WorldCoord(double x, double y, double z) {
	return Eigen::Vector3d((x - ModelSize / 2)*Size / ModelSize, (y - ModelSize / 2)*Size / ModelSize, (z - ModelSize / 2)*Size / ModelSize);
}

bool Model::
init(int w, int h, float size) {
	if (ModelC) {
		free(ModelC);
		ModelC = NULL;
	}
	ModelC = (u32*)malloc(sizeof(u32) * ModelTexSize * ModelTexSize);
	
	if (ModelSW) {
		free(ModelSW);
		ModelSW = NULL;
	}
	ModelSW = (u32*)malloc(sizeof(u32) * ModelTexSize * ModelTexSize);

	memset(ModelC, 0, sizeof(u32) * ModelTexSize * ModelTexSize);
	for (int i = 0; i < ModelTexSize; i++) {
		for (int j = 0; j < ModelTexSize; j++) {
			ModelSW[i*ModelTexSize + j] = u32(Mu + 128);
		}
	}

	m_axis[0] = Eigen::Vector3d( 1.0,  0.0,  0.0);
	m_axis[1] = Eigen::Vector3d(-1.0,  0.0,  0.0);
	m_axis[2] = Eigen::Vector3d( 0.0,  1.0,  0.0);
	m_axis[3] = Eigen::Vector3d( 0.0, -1.0,  0.0);
	m_axis[4] = Eigen::Vector3d( 0.0,  0.0,  1.0);
	m_axis[5] = Eigen::Vector3d( 0.0,  0.0, -1.0);

	m_vertex = (Eigen::Vector3d*)malloc(sizeof(Eigen::Vector3d) * 8);
	m_vertex[0] = WorldCoord(0.0, 0.0, 0.0);
	m_vertex[1] = WorldCoord(0.0, 0.0, ModelSize - 1);
	m_vertex[2] = WorldCoord(0.0, ModelSize - 1, 0.0);
	m_vertex[3] = WorldCoord(0.0, ModelSize - 1, ModelSize - 1);
	m_vertex[4] = WorldCoord(ModelSize - 1, 0.0, 0.0);
	m_vertex[5] = WorldCoord(ModelSize - 1, 0.0, ModelSize - 1);
	m_vertex[6] = WorldCoord(ModelSize - 1, ModelSize - 1, 0.0);
	m_vertex[7] = WorldCoord(ModelSize - 1, ModelSize - 1, ModelSize - 1);

	Size = size;
	m_image_width = w;
	m_image_height = h;
	return gl_init(w, h);
}

bool Model::
destroy() {
	if (ModelC) { free(ModelC); ModelC = NULL; }
	if (ModelSW) { free(ModelSW); ModelSW = NULL; }
	if (m_vertex) { free(m_vertex); m_vertex = NULL; }

	m_gl_updating.destroy();
	m_gl_raytracing.destroy();

	return true;
}

bool Model::
gl_init(int w, int h) {
	//model updating renderer
	if (!m_gl_updating.init(ModelTexSize, ModelTexSize))
		return false;
	if (!m_gl_updating.setShaderFile("shader/default.vert", "shader/model_updating.frag"))
		return false;
	m_gl_updating.CreateVertexBuffer();
	m_gl_updating_teximage = m_gl_updating.CreateTexture(&m_gl_updating_teximage.tex_id, w, h);
	m_gl_updating_texdepth = m_gl_updating.CreateTexture(&m_gl_updating_texdepth.tex_id, w, h);
	m_gl_updating_texmodel = m_gl_updating.CreateTexture(&m_gl_updating_texmodel.tex_id, ModelTexSize, ModelTexSize);

	//ray tracing renderer
	if (!m_gl_raytracing.init(w, h))
		return false;
	if (!m_gl_raytracing.setShaderFile("shader/default.vert", "shader/ray_tracing.frag"))
		return false;
	m_gl_raytracing.CreateVertexBuffer();
	m_gl_raytracing_texmodelC = m_gl_raytracing.CreateTexture(&m_gl_raytracing_texmodelC.tex_id, ModelTexSize, ModelTexSize);
	m_gl_raytracing_texmodelSW = m_gl_raytracing.CreateTexture(&m_gl_raytracing_texmodelSW.tex_id, ModelTexSize, ModelTexSize);
	return true;
}

bool Model::
ModelUpdating(u32 *I_i, u32 *D_i, CameraPose T_i) {
	if (!I_i || !D_i || !ModelC || !ModelSW) return false;

	m_gl_updating.useRenderer();

	float Q[16];
	for (int i = 0; i < 3; ++i) {
		for (int j = 0; j < 3; ++j) {
			Q[j * 4 + i] = static_cast<float>(T_i.Q(i, j));
		}
		Q[3 * 4 + i] = static_cast<float>(T_i.q[i]);
		Q[i * 4 + 3] = 0.0f;
	}
	Q[3 * 4 + 3] = 1.0;

	//C
	m_gl_updating.setUniform1("m_w",			m_image_width);
	m_gl_updating.setUniform1("m_h",			m_image_height);
	m_gl_updating.setUniform1("ModelSize",		static_cast<int>(ModelSize));
	m_gl_updating.setUniform1("ModelTexSize",	static_cast<int>(ModelTexSize));
	m_gl_updating.setUniform1("Mu",				static_cast<int>(Mu));
	m_gl_updating.setUniform1("size",			Size);
	m_gl_updating.setUniform1("isC_flag",		1);
	m_gl_updating.setUniform4v("Q",				Q);
	m_gl_updating.setTexSub2D("tex_image",		m_gl_updating_teximage, 0, GL_TEXTURE0, I_i);
	m_gl_updating.setTexSub2D("tex_depth",		m_gl_updating_texdepth, 1, GL_TEXTURE1, D_i);
	m_gl_updating.setTexSub2D("model",			m_gl_updating_texmodel, 2, GL_TEXTURE2, ModelC);
	free(ModelC); ModelC = NULL;
	ModelC = m_gl_updating.RenderScence<u32>();

	//SW
	m_gl_updating.setUniform1("m_w",			m_image_width);
	m_gl_updating.setUniform1("m_h",			m_image_height);
	m_gl_updating.setUniform1("ModelSize",		static_cast<int>(ModelSize));
	m_gl_updating.setUniform1("ModelTexSize",	static_cast<int>(ModelTexSize));
	m_gl_updating.setUniform1("Mu",				static_cast<int>(Mu));
	m_gl_updating.setUniform1("size",			Size);
	m_gl_updating.setUniform1("isC_flag",		0);
	m_gl_updating.setUniform4v("Q",				Q);
	m_gl_updating.setTexSub2D("tex_image",		m_gl_updating_teximage, 0, GL_TEXTURE0, I_i);
	m_gl_updating.setTexSub2D("tex_depth",		m_gl_updating_texdepth, 1, GL_TEXTURE1, D_i);
	m_gl_updating.setTexSub2D("model",			m_gl_updating_texmodel, 2, GL_TEXTURE2, ModelSW);
	free(ModelSW); ModelSW = NULL;
	ModelSW = m_gl_updating.RenderScence<u32>();

	return true;
}

bool Model::
RayTracing(CameraPose T_v, u32 **I_M, u32 **D_M, u32 **Y_M) {
	m_gl_raytracing.useRenderer();

	u32 *I = NULL, *D = NULL, *Y = NULL;
	//find nearest face
	double tmp = 0.0;
	int axis_tmp = 0;
	for (int i = 0; i < 6; ++i) {
		if (T_v.R.col(2).dot(m_axis[i])> tmp) {
			tmp = T_v.R.col(2).dot(m_axis[i]);
			axis_tmp = i;
		}
	}

	Eigen::Matrix3d invQ = T_v.Q.inverse();
	float iQ[9] = { 0.0 };
	for (int i = 0; i < 3; ++i) {
		for (int j = 0; j < 3; ++j) {
			iQ[i * 3 + j] = invQ(i, j);
		}
	}
#ifdef _DEBUG
	printf("axis: %d %lf\n", axis_tmp, tmp);
	printf("iQ:\n");
	for (int i = 0; i < 3; ++i) {
		for (int j = 0; j < 3; ++j) {
			printf("%f ", iQ[i * 3 + j]);
		}
		printf("\n");
	}
	printf("\n");
#endif
	//I_M flag=0
	m_gl_raytracing.setUniform1("m_w",		m_image_width);
	m_gl_raytracing.setUniform1("m_h",		m_image_height);
	m_gl_raytracing.setUniform1("ModelSize",	static_cast<int>(ModelSize));
	m_gl_raytracing.setUniform1("tex_size",	static_cast<int>(ModelTexSize));
	m_gl_raytracing.setUniform1("size",		Size);
	m_gl_raytracing.setUniform1("flag",		0);
	m_gl_raytracing.setUniform1("Mu",			static_cast<int>(Mu));
	m_gl_raytracing.setUniform1("Axis",		axis_tmp);
	m_gl_raytracing.setUniform3v("invQ",		iQ);
	m_gl_raytracing.setUniform3("q", static_cast<float>(T_v.q[0]), static_cast<float>(T_v.q[1]), static_cast<float>(T_v.q[2]));
	m_gl_raytracing.setTexSub2D("modelC",		m_gl_raytracing_texmodelC,  0, GL_TEXTURE0, ModelC);
	m_gl_raytracing.setTexSub2D("modelSW",	m_gl_raytracing_texmodelSW, 1, GL_TEXTURE1, ModelSW);
	
	I = m_gl_raytracing.RenderScence<u32>();

	//Y_M flag=1
	m_gl_raytracing.setUniform1("m_w",		m_image_width);
	m_gl_raytracing.setUniform1("m_h",		m_image_height);
	m_gl_raytracing.setUniform1("ModelSize",	static_cast<int>(ModelSize));
	m_gl_raytracing.setUniform1("tex_size",	static_cast<int>(ModelTexSize));
	m_gl_raytracing.setUniform1("size",		Size);
	m_gl_raytracing.setUniform1("flag",		1);
	m_gl_raytracing.setUniform1("Mu",			static_cast<int>(Mu));
	m_gl_raytracing.setUniform1("Axis",		axis_tmp);
	m_gl_raytracing.setUniform3v("invQ",		iQ);
	m_gl_raytracing.setUniform3("q", static_cast<float>(T_v.q[0]), static_cast<float>(T_v.q[1]), static_cast<float>(T_v.q[2]));
	m_gl_raytracing.setTexSub2D("modelC",		m_gl_raytracing_texmodelC,  0, GL_TEXTURE0, ModelC);
	m_gl_raytracing.setTexSub2D("modelSW",	m_gl_raytracing_texmodelSW, 1, GL_TEXTURE1, ModelSW);
	
	Y = m_gl_raytracing.RenderScence<u32>();

	//D_M flag=2
	m_gl_raytracing.setUniform1("m_w",		m_image_width);
	m_gl_raytracing.setUniform1("m_h",		m_image_height);
	m_gl_raytracing.setUniform1("ModelSize",	static_cast<int>(ModelSize));
	m_gl_raytracing.setUniform1("tex_size",	static_cast<int>(ModelTexSize));
	m_gl_raytracing.setUniform1("size",		Size);
	m_gl_raytracing.setUniform1("flag",		2);
	m_gl_raytracing.setUniform1("Mu",			static_cast<int>(Mu));
	m_gl_raytracing.setUniform1("Axis",		axis_tmp);
	m_gl_raytracing.setUniform3v("invQ",		iQ);
	m_gl_raytracing.setUniform3("q", static_cast<float>(T_v.q[0]), static_cast<float>(T_v.q[1]), static_cast<float>(T_v.q[2]));
	m_gl_raytracing.setTexSub2D("modelC",		m_gl_raytracing_texmodelC,  0, GL_TEXTURE0, ModelC);
	m_gl_raytracing.setTexSub2D("modelSW",	m_gl_raytracing_texmodelSW, 1, GL_TEXTURE1, ModelSW);
	
	D = m_gl_raytracing.RenderScence<u32>();

	*I_M = I;
	*D_M = D;
	*Y_M = Y;

	return true;
}

/*
u32 RGBAinter(u32 value, double f, u32 value2, double f2) {
	u32 r = u32(f * (value & 0xff) + f2 * (value2 & 0xff) + 0.0001f);
	u32 g = u32(f * ((value >> 8) & 0xff) + f2 * ((value2 >> 8) & 0xff) + 0.0001f);
	u32 b = u32(f * ((value >> 16) & 0xff) + f2 * ((value2 >> 16) & 0xff) + 0.0001f);
	u32 a = u32(f * ((value >> 24) & 0xff) + f2 * ((value2 >> 24) & 0xff) + 0.0001f);

	return ((a & 0xff) << 24) | ((b & 0xff) << 16) | ((g & 0xff) << 8) | (r & 0xff);
}

bool Model::
RayTracing_cpu(CameraPose T_v, u32 **I_M, u32 **D_M, u32 **Y_M) {
	u32 *I = (u32*)malloc(sizeof(u32) * m_image_width * m_image_height);
	u32 *Y = (u32*)malloc(sizeof(u32) * m_image_width * m_image_height);
	u32 *D = (u32*)malloc(sizeof(u32) * m_image_width * m_image_height);


	float tmp_a = 0.0;
	int axis_tmp = 0;
	for (int i = 0; i < 6; i++) {
		if (T_v.rot[2] * m_axis[i] > tmp_a) {
			tmp_a = T_v.rot[2] * m_axis[i];
			axis_tmp = i;
		}
	}
	printf("!! Axis = %d\n", axis_tmp);
	Matrix33d invQ = T_v.Q.inv();


	float tmp = ModelSize;
	u32 C = 0, SW = 0, last_C = 0, last_SW = 0;
	double depth = 0.0, last_depth;

	for (int i = 0; i < m_image_height; i++) {
		for (int j = 0; j < m_image_width; j++) {
			float xx, yy, weight;
			float weight_tmp = 0.0;

			int start, end, step;
			if (axis_tmp % 2 == 0) { start = 0; end = ModelSize; step = 1; }
			else { start = ModelSize - 1; end = -1; step = -1; }

			double x, y, z, xm, ym, zm, test;
			for (int k = start; k != end; k += step) {
				if (axis_tmp / 2 == 0) { x = WorldAxis(float(k)); test = x; }
				else if (axis_tmp / 2 == 1) { y = WorldAxis(float(k)); test = y; }
				else { z = WorldAxis(float(k)); test = z; }

				zm = (test + invQ[axis_tmp / 2] * T_v.q) / (invQ[axis_tmp / 2] * double3(j, 480 - i - 1, 1));
				xm = j*zm;
				ym = (480 - i - 1) * zm;

				x = (invQ[0] * double3(xm, ym, zm)) - (invQ[0] * T_v.q); x = LocalAxis(x);
				y = (invQ[1] * double3(xm, ym, zm)) - (invQ[1] * T_v.q); y = ModelSize - LocalAxis(y) - 1;
				z = (invQ[2] * double3(xm, ym, zm)) - (invQ[2] * T_v.q); z = LocalAxis(z);

				float s_tmp = ModelSize;
				if (x < 0 || x > ModelSize - 1 || y < 0 || y > ModelSize - 1 || z < 0 || z > ModelSize - 1) s_tmp = ModelSize;
				else {
					float tmp_z = z - floor(z);
					xx = (int(z) % 16) * 256 + x;
					yy = (int(z) / 16) * 256 + y;
					float xx1 = (tmp_z < 1e-6) ? 0.0f : (int(z + 1) % 16) * 256 + x;
					float yy1 = (tmp_z < 1e-6) ? 0.0f : (int(z + 1) / 16) * 256 + y;
					last_SW = SW;
					last_C = C;

					//SW = (1.0 - tmp_z) * binter(ModelSW, xx, yy, 4096) + (tmp_z) * binter(ModelSW, xx1, yy1, 4096);
					SW = RGBAinter(binter(ModelSW, xx, yy, 4096), (1.0 - tmp_z), binter(ModelSW, xx1, yy1, 4096), tmp_z);
					//C  = (1.0 - tmp_z) * binter(ModelC,  xx, yy, 4096) + (tmp_z) * binter(ModelC,  xx1, yy1, 4096);
					C = RGBAinter(binter(ModelC, xx, yy, 4096), (1.0 - tmp_z), binter(ModelC, xx1, yy1, 4096), tmp_z);

					//xx = floor(xx);
					//yy = floor(yy);

					//SW = ModelSW[int(yy) * 4096 + int(xx)];
					//C  = ModelC [int(yy) * 4096 + int(xx)];

					s_tmp = float(SW & 0xff) - 128.f;
					weight = float((SW >> 16) & 0xff);

					//float3 p = T_v.rot * WorldCoord(x, ModelSize - y - 1, z) + T_v.translation;
					last_depth = depth;
					depth = zm;

					if (i == 203 && j == 302 && ((SW >> 24) & 0xff) + 1e-6 >= 1.0) {
						printf("x,y,z: %f %f %f\n", x, y, z);
						u32 testC = binter(ModelC, xx, yy, 4096);
						printf("testC1: %d %d %d %d\n", (testC) & 0xff,
							(testC >> 8) & 0xff,
							(testC >> 16) & 0xff,
							(testC >> 24) & 0xff);
						testC = binter(ModelC, xx1, yy1, 4096);
						printf("testC2: %d %d %d %d\n", (testC) & 0xff,
							(testC >> 8) & 0xff,
							(testC >> 16) & 0xff,
							(testC >> 24) & 0xff);

						printf("C: %d %d %d %d\n", (C) & 0xff,
							(C >> 8) & 0xff,
							(C >> 16) & 0xff,
							(C >> 24) & 0xff);
						printf("%f * :\n", (1.0 - tmp_z));
						print_inter(ModelC, xx, yy, 4096);
						printf("%f * :\n", tmp_z);
						print_inter(ModelC, xx1, yy1, 4096);
						printf("\n");
						printf("SW: %d %d %d %d\n", (SW) & 0xff,
							(SW >> 8) & 0xff,
							(SW >> 16) & 0xff,
							(SW >> 24) & 0xff);
						printf("%f * :\n", (1.0 - tmp_z));
						print_inter(ModelSW, xx, yy, 4096);
						printf("%f * :\n", tmp_z);
						print_inter(ModelSW, xx1, yy1, 4096);
						printf("\n");

						if (((SW >> 24) & 0xff) + 1e-6 >= 1.0) printf("!!!!!!!!!\n");
						printf("tmp : %f\ns_tmp : %f\n\n", tmp, s_tmp);

					}


					if (((SW >> 24) & 0xff) + 1e-6 < 1.0) {
						s_tmp = ModelSize;
					}
				}
				if (tmp > 0.0f && s_tmp <= 0.0f) {// && tmp < Mu && s_tmp > -Mu) {
					if (i == 203 && j == 302) {
						printf("--------- SW: %d %d %d %d\n", (SW >> 24) & 0xff,
							(SW >> 16) & 0xff,
							(SW >> 8) & 0xff,
							(SW) & 0xff);
						printf("--------- last_SW: %d %d %d %d\n", (last_SW >> 24) & 0xff,
							(last_SW >> 16) & 0xff,
							(last_SW >> 8) & 0xff,
							(last_SW) & 0xff);
					}
					//I
					if (tmp != ModelSize && s_tmp != 0.0f) {
						I[(i*g_w + j) * 4] = (((C) & 0xff)*(-tmp) + ((last_C) & 0xff)*(s_tmp)) / (s_tmp - tmp);
						I[(i*g_w + j) * 4 + 1] = (((C >> 8) & 0xff)*(-tmp) + ((last_C >> 8) & 0xff)*(s_tmp)) / (s_tmp - tmp);
						I[(i*g_w + j) * 4 + 2] = (((C >> 16) & 0xff)*(-tmp) + ((last_C >> 16) & 0xff)*(s_tmp)) / (s_tmp - tmp);
						I[(i*g_w + j) * 4 + 3] = 255;
					}
					else {
						I[(i*g_w + j) * 4] = ((C) & 0xff);
						I[(i*g_w + j) * 4 + 1] = ((C >> 8) & 0xff);
						I[(i*g_w + j) * 4 + 2] = ((C >> 16) & 0xff);
						I[(i*g_w + j) * 4 + 3] = 255;
					}

					//Y
					if (tmp != ModelSize && s_tmp != 0.0f) {
						Y[(i*g_w + j) * 4] = (((SW >> 16) & 0xff)*(-tmp) + ((last_SW >> 16) & 0xff)*(s_tmp)) / (s_tmp - tmp);
						Y[(i*g_w + j) * 4 + 1] = (((SW >> 16) & 0xff)*(-tmp) + ((last_SW >> 16) & 0xff)*(s_tmp)) / (s_tmp - tmp);
						Y[(i*g_w + j) * 4 + 2] = (((SW >> 16) & 0xff)*(-tmp) + ((last_SW >> 16) & 0xff)*(s_tmp)) / (s_tmp - tmp);
						Y[(i*g_w + j) * 4 + 3] = 255;
					}
					else {
						Y[(i*g_w + j) * 4] = (SW >> 16) & 0xff;
						Y[(i*g_w + j) * 4 + 1] = (SW >> 16) & 0xff;
						Y[(i*g_w + j) * 4 + 2] = (SW >> 16) & 0xff;
						Y[(i*g_w + j) * 4 + 3] = 255;
					}

					//D
					D[i*g_w + j] = 0;

					break;
				}
				tmp = s_tmp;
			}
		}
	}
	scanf("%*c");
	*I_M = I;
	*Y_M = Y;
	*D_M = D;

	return true;
}

*/

} // namespace mf
