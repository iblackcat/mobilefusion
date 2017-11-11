#ifndef POSE_ESTIMATION_H
#define POSE_ESTIMATION_H

#include "utils.h"
#include <LMSolver.h>
#include <eigen/Eigen>

namespace mf {

typedef double real;
typedef Eigen::Matrix<real, -1, 1> DVec;
typedef Eigen::Matrix<real, -1, -1> DMat;

class MySolver : public CDenseNewtonSolver {
public:
	bool init(GlobalCoeff _g, real eps = 1e-4);
	void setImages(u32 *Ii, u32 *I, u32 *D, u32 *Y);
	real Optimize(DVec& xStart, int nMaxIter, bool showInfo = false);
	void CalcJacobiFunc(DVec& x, DMat& jac);

	inline bool getCoordInCamera2(int i, int j, CameraPose Tvl, Vector6d update_se3, Eigen::Vector3d &q, Eigen::Vector3d &u) {
		double d = getDepth(D_m[i*m_w + j]);
		//if (d != 0.0)
		//	std::cout << "d = " << d << std::endl;
		
		Eigen::Vector3d P;
		P.setZero(); q.setZero();
		if (d == 0 || rgba2gray(I_m[i*m_w + j]) == 0)
			return false;

		P = m_camera_intrinsic.inverse() * Eigen::Vector3d(j*d, (m_h - i - 1)*d, d);
		P = Tvl.SE3_Rt * P; // P in camera2
		Eigen::Vector4d q4 = (Sophus::SE3d::hat(update_se3)) * Eigen::Vector4d(P[0], P[1], P[2], 1.0);
		q = Eigen::Vector3d(q4[0], q4[1], q4[2]);
		u = Tvl.intrinsics * (P + q);
		u[0] /= P[2]; u[1] /= P[2];

		//std::cout << "P = " << P.transpose() << std::endl;
		//std::cout << "q = " << q4.transpose() << std::endl;
		//std::cout << "u :" << u << std::endl;
		//std::cout << "update.hat = " << std::endl << Sophus::SE3d::hat(update_se3) << std::endl;
		return true;
	}

	inline static real getDepth(u32 depth) {
		int a = (depth >> 24);
		int b = (depth >> 16) & 0xff;
		int g = (depth >> 8) & 0xff;
		int r = (depth) & 0xff;

		return (real(r) * 256.0 * 256.0 + real(g) * 256.0 + real(b)) / (256.0 * 256.0);
	}
protected:
	virtual real CalcEnergyFunc(const DVec& x, DVec& fx);

private:
	GlobalCoeff G;
	int m_w, m_h;
	u32	*I_m, *D_m, *Y_m, *I_i;
	Eigen::Matrix3d m_camera_intrinsic;
};


class PoseEstimation {
public:
	bool init(GlobalCoeff _g);
	CameraPose DenseImageAlignment(CameraPose pose, u32 *I, u32 *D, u32 *Y, u32 *new_I);

private:
	GlobalCoeff G;
	int m_w, m_h;
	Eigen::Matrix3d m_camera_intrinsic;
	MySolver m_NewtonSolver;
};

} //namespace mf

#endif //POSE_ESTIMATION_H
