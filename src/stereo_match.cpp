#include "stereo_match.h"

namespace mf {

bool StereoMatch::
init(int w, int h, const char* method_name) {
	m_width = w;
	m_height = h;

	return gl_init(w, h, method_name);
}

bool StereoMatch::
destroy() {
	m_gl_disparity.destroy();
	m_gl_depth.destroy();
	m_gl_depth2gray.destroy();

	if (Delta1) { free(Delta1); Delta1 = NULL; }
	if (Delta2) { free(Delta2); Delta2 = NULL; }
	return true;
}

bool StereoMatch::
gl_init(int w, int h, const char* method_name) {
	//disparity estimation renderer
	char stereo_match_method[256];
	sprintf(stereo_match_method, "shader/stereo_matching_%s.frag", method_name);

	if (!m_gl_disparity.init(w, h))
		return false;
	if (!m_gl_disparity.setShaderFile("shader/default.vert", stereo_match_method))
		return false;
	m_gl_disparity.CreateVertexBuffer();
	m_gl_disparity_tex1 = m_gl_disparity.CreateTexture(&m_gl_disparity_tex1.tex_id, w, h);
	m_gl_disparity_tex2 = m_gl_disparity.CreateTexture(&m_gl_disparity_tex2.tex_id, w, h);
	
	//LR-check and depth renderer
	if (!m_gl_depth.init(w, h))
		return false;
	if (!m_gl_depth.setShaderFile("shader/default.vert", "shader/lrcheck_and_triangulation.frag"))
		return false;
	m_gl_depth.CreateVertexBuffer();
	m_gl_depth_tex1 = m_gl_depth.CreateTexture(&m_gl_depth_tex1.tex_id, w, h);
	m_gl_depth_tex2 = m_gl_depth.CreateTexture(&m_gl_depth_tex2.tex_id, w, h);
	
	//depth image to gray image renderer
	if (!m_gl_depth2gray.init(w, h))
		return false;
	if (!m_gl_depth2gray.setShaderFile("shader/default.vert", "shader/depth_rgba2gray.frag"))
		return false;
	m_gl_depth2gray.CreateVertexBuffer();
	m_gl_d2g_tex = m_gl_depth2gray.CreateTexture(&m_gl_d2g_tex.tex_id, w, h);
	return true;
}

void StereoMatch::
DisparityEstimation(u32 *image1, u32 *image2, const int radius) {
	if (Delta1) { free(Delta1); Delta1 = NULL; }
	if (Delta2) { free(Delta2); Delta2 = NULL; }

	m_gl_disparity.useRenderer();
	m_gl_disparity.setUniform1("m_w", m_width);
	m_gl_disparity.setUniform1("m_h", m_height);
	m_gl_disparity.setUniform1("step", 1);
	m_gl_disparity.setUniform1("radius", radius);
	m_gl_disparity.setTexSub2D("tex" , m_gl_disparity_tex1, 0, GL_TEXTURE0, image1);
	m_gl_disparity.setTexSub2D("tex2", m_gl_disparity_tex2, 1, GL_TEXTURE1, image2);
	Delta1 = m_gl_disparity.RenderScence<u32>();

	m_gl_disparity.setUniform1("m_w", m_width);
	m_gl_disparity.setUniform1("m_h", m_height);
	m_gl_disparity.setUniform1("step", -1);
	m_gl_disparity.setUniform1("radius", radius);
	m_gl_disparity.setTexSub2D("tex" , m_gl_disparity_tex1, 0, GL_TEXTURE0, image2);
	m_gl_disparity.setTexSub2D("tex2", m_gl_disparity_tex2, 1, GL_TEXTURE1, image1);
	Delta2 = m_gl_disparity.RenderScence<u32>();
}

u32* StereoMatch::
LRCheckAndGetDepth(int max_diff, float baseline, float fx) {
	if (!Delta1 || !Delta2) return NULL;

	m_gl_depth.useRenderer();

	m_gl_depth.setUniform1("m_w",		m_width);
	m_gl_depth.setUniform1("max_diff",	max_diff);
	m_gl_depth.setUniform1("baseline",	baseline);
	m_gl_depth.setUniform1("fx",		fx);

	m_gl_depth.setTexSub2D("tex" , m_gl_depth_tex1, 0, GL_TEXTURE0, Delta1);
	m_gl_depth.setTexSub2D("tex2", m_gl_depth_tex2, 1, GL_TEXTURE1, Delta2);

	u32 *Depth = m_gl_depth.RenderScence<u32>();
	return Depth;
}

u32* StereoMatch::
u32DepthToGray(u32* depth, float scale) {
	if (!depth) return NULL;

	m_gl_depth2gray.useRenderer();
	m_gl_depth2gray.setUniform1("scale", scale);
	m_gl_depth2gray.setTexSub2D("tex", m_gl_d2g_tex, 0, GL_TEXTURE0, depth);
	u32 *gray = m_gl_depth2gray.RenderScence<u32>();

	return gray;
}

} // namespace mf
