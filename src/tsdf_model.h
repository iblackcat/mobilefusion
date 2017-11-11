#ifndef TSDF_MODEL_H
#define TSDF_MODEL_H

#include "utils.h"
#include "gl_helper.h"

namespace mf {

class Model{
public:
	inline double LocalAxis(double tmp);
	inline double WorldAxis(double tmp);
	inline Eigen::Vector3d WorldCoord(double x, double y, double z);
	inline Eigen::Vector3d* getVertex() { return m_vertex; }

	bool init(int w, int h, float size = 4.0);
	bool destroy();

	bool gl_init(int w, int h);

	bool ModelUpdating(u32 *I_i, u32 *D_i, CameraPose T_i);
	bool RayTracing(CameraPose T_v, u32 **I_M, u32 **D_M, u32 **Y_M);
	//bool RayTracing_cpu(CameraPose T_v, u32 **I_M, u32 **D_M, u32 **Y_M);

	//test
	u32 *getModelC() { return ModelC; }
	u32 *getModelSW() { return ModelSW; }

private:
	const int				ModelSize = 256; //in voxel
	const int				ModelTexSize = 4096; //in voxel
	const int				Mu = 4; //in voxel

	float					Size = 4.0; //in millimeter
	int						m_image_width;
	int						m_image_height;
	u32*					ModelC;
	u32*					ModelSW;
	Eigen::Vector3d			m_axis[6];
	Eigen::Vector3d*		m_vertex = NULL;

	jhw_gl::MyGLRenderer	m_gl_updating;
	jhw_gl::GLTex2d			m_gl_updating_teximage;
	jhw_gl::GLTex2d			m_gl_updating_texdepth;
	jhw_gl::GLTex2d			m_gl_updating_texmodel;

	jhw_gl::MyGLRenderer	m_gl_raytracing;
	jhw_gl::GLTex2d			m_gl_raytracing_texmodelC;
	jhw_gl::GLTex2d			m_gl_raytracing_texmodelSW;
};

}

#endif //TSDF_MODEL_H
