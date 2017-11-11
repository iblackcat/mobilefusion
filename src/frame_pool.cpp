#include "frame_pool.h"

namespace mf {

bool FramePool::
init(int w, int h) {
	m_width = w;
	m_height = h;
	head = 0;
	rear = 0;
	//I = (u32**)malloc(sizeof(u32*)*pool_size);
	//T = (CameraPose*)malloc(sizeof(CameraPose)*pool_size);
	frames = new Frame[pool_size];

	//memset(I, 0, sizeof(u32*)*pool_size);

	Image_cache1 = NULL;
	Image_cache2 = NULL;
	tsdf_vertex	 = NULL;

	m_fx = -1.0;
	m_fy = -1.0;
	m_focalLength = -1.0;

	return gl_init(w, h);
}
/*
bool FramePool::
init(int w, int h, double fx, double fy, double focalLength, Eigen::Vector3d *v) {
	if (!init(w, h)) return false;
	if (fx <= 0 || fy <= 0 || focalLength <= 0) return false;
	if (!v) return false;
	m_fx = fx;
	m_fy = fy;
	m_focalLength = focalLength;
	tsdf_vertex = (Eigen::Vector3d*)malloc(sizeof(Eigen::Vector3d) * 8);
	for (int i = 0; i < 8; i++) {
		tsdf_vertex[i] = v[i];
	}
	return true;
}
*/
bool FramePool::
init(GlobalCoeff _g, Eigen::Vector3d *v) {
	if (!init(_g.g_w, _g.g_h)) return false;
	if (!v) return false;

	G = _g;
	//?!
	m_fx = _g.g_fx;
	m_fy = _g.g_fy;
	m_focalLength = _g.g_focallength;
	tsdf_vertex = (Eigen::Vector3d*)malloc(sizeof(Eigen::Vector3d) * 8);
	for (int i = 0; i < 8; i++) {
		tsdf_vertex[i] = v[i];
	}
	return true;
}

bool FramePool::
destroy() {
	for (int i = 0; i < rear; ++i) 
		frames[i].destroy();
	delete[] frames;

	m_gl_rectify.destroy();
	return true;
}

bool FramePool::
gl_init(int w, int h) {
	if (!m_gl_rectify.init(w, h)) 
		return false;
	if (!m_gl_rectify.setShaderFile("shader/default.vert", "shader/Rectification.frag"))
		return false;
	m_gl_rectify.CreateVertexBuffer();
	m_gl_tex = m_gl_rectify.CreateTexture(&m_gl_tex.tex_id, w, h);
	return true;
}

inline int FramePool::
lastframe_id() {
	if (!head && !rear) return -1;
	else return (head > 0) ? (head - 1) : (pool_size - 1);
}


double FramePool::
FramepairScore(Frame frame1, Frame frame2) {
	double delta_max = 0.0;
	Rectification(frame1, frame2);
	for (int i = 0; i < 8; ++i) {
		double delta = fabs((rect_pose1.Q * tsdf_vertex[i] + rect_pose1.q)[0]
						  - (rect_pose2.Q * tsdf_vertex[i] + rect_pose2.q)[0]);
		if (delta > delta_max) delta_max = delta;
	}

	double S1k = (m_width * m_focalLength / m_fx - baseline) * m_height * m_focalLength / m_fy;
	double S2k = -fabs(1.0 - delta_max / d_max);

	return S1k + S2k;
}

bool FramePool::
addFrame(Frame new_frame) {
	//keyframe selection
	if ((head || rear) && tsdf_vertex && m_fx > 0 && m_fy > 0 && m_focalLength > 0) {
		double max_score = FramepairScore(new_frame, frames[0]);
		int keyframe = 0;
		for (int k = 1; k < rear; ++k) {
			double score = FramepairScore(new_frame, frames[k]);
			if (score > max_score) {
				max_score = score;
				keyframe = k;
			}
		}
		Rectification(new_frame, frames[keyframe]);
		printf("keyframe: %d\n", keyframe);
	}

	frames[head].destroy();
	frames[head] = new_frame;
	++head;

	if (head >= 40) head = 0;
	if (rear <  40) ++rear;

	//first add
	if (head == 1 && rear == 1) 
		return false;
	if (!tsdf_vertex || m_fx <= 0 || m_fy <= 0 || m_focalLength <= 0) {
#ifdef _DEBUG
		printf("Error: Please 'setVertex' before keyframe selection!\n");
#endif
		return false;
	}
	else return true;
}

bool FramePool::
getRectPair(Frame &rec1, Frame &rec2) {
	if (!Image_cache1 || !Image_cache2) return false;

	m_gl_rectify.useRenderer();

	u32* rect_pair1 = NULL;
	u32* rect_pair2 = NULL;

	float trans1[16] = { 0 }, trans2[16] = { 0 };
	for (int i = 0; i < 4; ++i) {
		for (int j = 0; j < 4; ++j) {
			if (i == 3 && j == 3) { trans1[i * 4 + j] = 1.0f; trans2[i * 4 + j] = 1.0f; }
			else if (i == 3 || j == 3) { trans1[i * 4 + j] = 0.0f; trans2[i * 4 + j] = 0.0f; }
			else { trans1[i * 4 + j] = (float)rect_tran1(j, i); trans2[i * 4 + j] = (float)rect_tran2(j, i); }
		}
	}

	m_gl_rectify.setUniform1("m_w", m_width);
	m_gl_rectify.setUniform1("m_h", m_height);
	m_gl_rectify.setUniform4v("Trans", trans1);
	m_gl_rectify.setTexSub2D("tex", m_gl_tex, 0, GL_TEXTURE0, Image_cache1);
	rect_pair1 = m_gl_rectify.RenderScence<u32>();

	m_gl_rectify.setUniform1("m_w", m_width);
	m_gl_rectify.setUniform1("m_h", m_height);
	m_gl_rectify.setUniform4v("Trans", trans2);
	m_gl_rectify.setTexSub2D("tex", m_gl_tex, 0, GL_TEXTURE0, Image_cache2);
	rect_pair2 = m_gl_rectify.RenderScence<u32>();

	// ?!
	if (rec1.getImage()) { free(rec1.getImage()); }
	if (rec2.getImage()) { free(rec2.getImage()); }
	rec1 = Frame(rect_pair1, rect_pose1);
	rec2 = Frame(rect_pair2, rect_pose2);

	Image_cache1 = NULL;
	Image_cache2 = NULL;
	return true;
}

void FramePool::
Rectification(Frame &frame1, Frame &frame2) {
	//pose rectification
	CameraPose P1 = frame1.getPose();
	CameraPose P2 = frame2.getPose();

	Eigen::Matrix3d A = (P1.intrinsics + P2.intrinsics) / 2;
	Eigen::Vector3d v1 = P1.center - P2.center;
	Eigen::Vector3d v2 = P1.R.row(2).cross(v1);//v2 = P1.rot[2].cross(v1);
	Eigen::Vector3d v3 = v1.cross(v2);
	Eigen::Matrix3d R;// (v1.norm(), v2.norm(), v3.norm());
	R.row(0) = v1.normalized();
	R.row(1) = v2.normalized();
	R.row(2) = v3.normalized();

	rect_pose1.R = R;
	rect_pose1.intrinsics = A;
	rect_pose1.t = -(R * P1.center);
	rect_pose1.center = P1.center;
	rect_pose1.Q = A * R;
	rect_pose1.q = A * rect_pose1.t;
	//rect_pose1.rotation = rect_pose1.getRotation(R);

	rect_pose2.R = R;
	rect_pose2.intrinsics = A;
	rect_pose2.t = -(R * P2.center);
	rect_pose2.center = P2.center;
	rect_pose2.Q = A * R;
	rect_pose2.q = A * rect_pose2.t;
	//rect_pose2.rotation = rect_pose2.getRotation(R);

	rect_tran1 = P1.Q * rect_pose1.Q.inverse(); // rect_pose1 to P1
	rect_tran2 = P2.Q * rect_pose2.Q.inverse(); // rect_pose2 to P2

	baseline = static_cast<float>((rect_pose1.center - rect_pose2.center).norm());
	
	Image_cache1 = frame1.getImage();
	Image_cache2 = frame2.getImage();
}

}//namespace mf