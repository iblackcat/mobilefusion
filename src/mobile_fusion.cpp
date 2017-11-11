#include "mobile_fusion.h"

namespace mf {
/*
MobileFusion::
MobileFusion()
   :m_fx(500.0),
	m_fy(500.0),
	m_cx(150.0),
	m_cy(150.0),
	m_focalLength(3.75),
	CameraCoeff(m_fx, 0.0, m_cx, 0.0, m_fy, m_cy, 0.0, 0.0, 1.0),
	DispCacheI(NULL),
	DispCacheD(NULL),
	DispCacheY(NULL),
	frame_id(0.0),
	last_pose()
{
}
bool MobileFusion::
init(int w, int h, float focalLenght, Eigen::Matrix3d Coeff) {
	m_w = w;
	m_h = h;
	m_focalLength = focalLenght;
	m_camera_intrinsic = Coeff;
	m_fx = Coeff(0, 0);
	m_fy = Coeff(1, 1);
	m_cx = Coeff(0, 2);
	m_cy = Coeff(1, 2);

	PoseGuess.init(m_w, m_h, Coeff);
	TSDFModel.init(m_w, m_h);
	Frames.init(m_w, m_h, m_fx, m_fy, m_focalLength, TSDFModel.getVertex());
	DenseStereoMatch.init(m_w, m_h, "ZNCC");

	frame_id = 0;

	return true;
}
*/

bool MobileFusion::
init(GlobalCoeff _g) {
	G = _g;

	PoseGuess.init(_g);
	TSDFModel.init(G.g_w, G.g_h);
	Frames.init(_g, TSDFModel.getVertex());
	DenseStereoMatch.init(G.g_w, G.g_h, "ZNCC");

	frame_id = 0;
	return true;
}

bool MobileFusion::
destroy() {
	if (DispCacheI) { free(DispCacheI); DispCacheI = NULL; }
	if (DispCacheD) { free(DispCacheD); DispCacheD = NULL; }
	if (DispCacheY) { free(DispCacheY); DispCacheY = NULL; }

	TSDFModel.destroy();
	Frames.destroy();
	DenseStereoMatch.destroy();
	return true;
}

/*
bool MobileFusion::
setCameraIntrinsic(double fx, double fy, double cx, double cy) {
	m_fx = fx;
	m_fy = fy;
	m_cx = cx;
	m_cy = cy;

	m_camera_intrinsic.setIdentity();
	m_camera_intrinsic(0, 0) = m_fx;
	m_camera_intrinsic(1, 1) = m_fy;
	m_camera_intrinsic(0, 2) = m_cx;
	m_camera_intrinsic(1, 2) = m_cy;
	return true;
}
*/
#include "stb_image.h"
#include "stb_image_write.h"

//TODO
bool MobileFusion::
addFrame(u32 *new_image) {
	CameraPose new_pose;
	if (frame_id <= init_frame_num) {
		//use feature-based pose estimation

	}
	else {
		//use dense pose estimation
		///step 0. guess pose by IMU
		//update last_pose

		///step 1. Ray-tracing
		TSDFModel.RayTracing(last_pose, &DispCacheI, &DispCacheD, &DispCacheY);
		//stbi_write_png("res/Iold.png", m_w, m_h, 4, DispCacheI, sizeof(u32) * m_w);
		//stbi_write_png("res/IoldD.png", m_w, m_h, 4, DenseStereoMatch.u32DepthToGray(DispCacheD, 3.0), sizeof(u32) * m_w);
		//stbi_write_png("res/IoldY.png", m_w, m_h, 4, DispCacheY, sizeof(u32) * m_w);
		//stbi_write_png("res/Inew.png", m_w, m_h, 4, new_image, sizeof(u32) * m_w);

		///step 2. pose estimation
		new_pose = PoseGuess.DenseImageAlignment(last_pose, DispCacheI, DispCacheD, DispCacheY, new_image);
	}
	//printf("\n");
	//last_pose.getRotation(last_pose.rot).print("last_pose");
	//new_pose.getRotation(new_pose.rot).print("new_pose");
	//printf("\n");
	last_pose = new_pose;
	///step 3. add Frame and Pose
	return addFrameAndPose(Frame(new_image, new_pose));
}

bool MobileFusion::
addFrameAndPose(Frame newframe) {
	//TODO: get key-frame id
	++frame_id;
	last_pose = newframe.getPose();

	if (Frames.addFrame(newframe)) {
		Frame rec1, rec2;
		u32 *depth;
		Frames.getRectPair(rec1, rec2);

		///step 4. get Depth
		DenseStereoMatch.DisparityEstimation(rec1.getImage(), rec2.getImage(), 3);
		depth = DenseStereoMatch.LRCheckAndGetDepth(stereo_match_max_diff, Frames.getBaseline(), G.g_fx);
	
		u32 *dd = DenseStereoMatch.u32DepthToGray(depth, 3.0);
		//stbi_write_png("res/depth.png", m_w, m_h, 4, dd, sizeof(u32) * m_w);
		//stbi_write_png("res/1.png", m_w, m_h, 4, DenseStereoMatch.u32DepthToGray(DenseStereoMatch.getDisparity1()), sizeof(u32) * m_w);
		//stbi_write_png("res/2.png", m_w, m_h, 4, DenseStereoMatch.u32DepthToGray(DenseStereoMatch.getDisparity2()), sizeof(u32) * m_w);
		//free(dd); dd = NULL;

		///step 5. model update
		TSDFModel.ModelUpdating(rec1.getImage(), depth, rec1.getPose());

		rec1.destroy(); 
		rec2.destroy();
		return true;
	}
	else return false;
}

bool MobileFusion::
RayTracing(CameraPose pose) {
	return TSDFModel.RayTracing(pose, &DispCacheI, &DispCacheD, &DispCacheY);
}

bool MobileFusion::
getDisplay(u32 **I_M, u32 **D_M, u32 **Y_M) {
	if (!DispCacheI || !DispCacheD || !DispCacheY) return false;

	*I_M = DispCacheI;
	*D_M = DispCacheD;
	*Y_M = DispCacheY;

	DispCacheI = NULL;
	DispCacheD = NULL;
	DispCacheY = NULL;
	return true;
}

} //namespace mf