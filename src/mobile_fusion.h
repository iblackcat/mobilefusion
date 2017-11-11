#ifndef MOBILEFUSION_H
#define MOBILEFUSION_H

#include "utils.h"
#include "pose_estimation.h"
#include "tsdf_model.h"
#include "frame_pool.h"
#include "stereo_match.h"

namespace mf {

class MobileFusion {
public:
	//MobileFusion();
	
	bool cameraCalib();
	bool setCameraIntrinsic(double fx, double fy, double cx, double cy);
	//bool changeResolution(int w, int h);
	//bool init(int w, int h, float focalLengh, Eigen::Matrix3d Coeff);	
	bool init(GlobalCoeff _g);									///step 0. initiate

	bool addFrame(u32 *new_frame);								///step 1. add a live frame
	bool addFrameAndPose(Frame newframe);
	bool RayTracing(CameraPose pose);

	bool getDisplay(u32 **I_M, u32 **D_M, u32 **Y_M);			///step 2. get display images

	bool destroy();												///step n. destroy

	//test
	u32 *getModelC() { return TSDFModel.getModelC(); }
	u32 *getModelSW() { return TSDFModel.getModelSW(); }

private:
	const int				init_frame_num = 5;   //use feature-based pose estimation
	const int				stereo_match_max_diff = 5;
	
	GlobalCoeff				G;
	/*
	int						m_w;
	int						m_h;
	double					m_fx;
	double					m_fy;
	double					m_cx;
	double					m_cy;
	double					m_focalLength;
	*/
	u32*					DispCacheI;
	u32*					DispCacheD;
	u32*					DispCacheY;

	int						frame_id;
	PoseEstimation			PoseGuess;
	Model					TSDFModel;
	FramePool				Frames;
	StereoMatch				DenseStereoMatch;

	CameraPose				last_pose;

public:
	//Eigen::Matrix3d			m_camera_intrinsic;
};

} //namespace mf

#endif //MOBILEFUSION_H
