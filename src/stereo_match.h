#ifndef STEREO_MATCH_H
#define STEREO_MATCH_H

#include "utils.h"
#include "gl_helper.h"

namespace mf {

class StereoMatch {
public:
	/*
	Method choice:
	1. "SSD" : Sum of Square Differences (default).
	2. "ZNCC" : Zero mean Normalized Cross Correlation.
	*/
	bool	init					(int w, int h, const char *method_name = "SSD");
	bool	destroy					();

	void	DisparityEstimation		(u32 *image1, u32 *image2, const int radius = 2);
	u32*	LRCheckAndGetDepth		(int max_diff, float baseline, float fx);

	bool	gl_init					(int w, int h, const char *method_name);

	inline 
	u32*	getDisparity1			() { u32* tmp = Delta1; Delta1 = NULL; return tmp; }
	inline 
	u32*	getDisparity2			() { u32* tmp = Delta2; Delta2 = NULL; return tmp; }

	//use this function to visualize depth value
	u32*	u32DepthToGray(u32* depth, float scale = 1.0);

private:
	//DISALLOW_COPY_AND_ASSIGN(StereoMatch);

	int						m_width;
	int						m_height;

	u32*					Delta1;
	u32*					Delta2;

	jhw_gl::MyGLRenderer	m_gl_disparity;
	jhw_gl::GLTex2d			m_gl_disparity_tex1;
	jhw_gl::GLTex2d			m_gl_disparity_tex2;

	jhw_gl::MyGLRenderer	m_gl_depth;
	jhw_gl::GLTex2d			m_gl_depth_tex1;
	jhw_gl::GLTex2d			m_gl_depth_tex2;

	jhw_gl::MyGLRenderer	m_gl_depth2gray;
	jhw_gl::GLTex2d			m_gl_d2g_tex;
};

}//namespace mf

#endif //STEREO_MATCH_H
