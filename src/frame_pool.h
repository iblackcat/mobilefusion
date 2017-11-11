#ifndef FRAME_POOL_h
#define FRAME_POOL_h

#include "utils.h"
#include "gl_helper.h"

namespace mf {

class Frame;
class FramePool;


class FramePool {
public:
	//call this init when only use Rectification
	bool init(int w, int h);
	//call this init when use keyframe Selection
	bool init(GlobalCoeff _g, Eigen::Vector3d *v);
	//bool init(int w, int h, double fx, double fy, double focalLength, Eigen::Vector3d *v);
	
	bool destroy();

	//Don't free frame, it will free by FramePool.						///setVertex before keyframe select
	bool addFrame(Frame new_frame);												///step 0. add new frame
	bool getRectPair(Frame &rec1, Frame &rec2);	///step 1. get frame pair

	bool gl_init(int w, int h);

	double FramepairScore(Frame frame1, Frame frame2);
	//Don't free frame1 or frame2 before 'getRectPair'.
	void Rectification(Frame &frame1, Frame &frame2);

	inline int lastframe_id();
	inline float getBaseline() { return baseline; }

private:
	//DISALLOW_COPY_AND_ASSIGN(FramePool);

	const int				pool_size = 40;
	const int				d_max = 64;

	GlobalCoeff				G;
	int						m_width, m_height;  //camera resolution
	int						head, rear;
	double					m_fx, m_fy, m_focalLength;
	//u32**					I;
	//CameraPose*				T;
	Frame*					frames;

	CameraPose				rect_pose1;
	CameraPose				rect_pose2;
	Eigen::Matrix3d			rect_tran1;
	Eigen::Matrix3d			rect_tran2;

	u32*					Image_cache1;
	u32*					Image_cache2;
	int						keyframe_id;
	float					baseline;
	Eigen::Vector3d*		tsdf_vertex;

	jhw_gl::MyGLRenderer	m_gl_rectify;
	jhw_gl::GLTex2d			m_gl_tex;
};



} //namespace mf

#endif //FRAME_POOL_h
