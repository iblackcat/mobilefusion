#include "pose_estimation.h"

namespace mf {

bool MySolver::init(GlobalCoeff _g, real eps) {
	G = _g;
	m_w = G.g_w;
	m_h = G.g_h;
	m_camera_intrinsic = G.Intrinsic;

	M = (m_w - 20) * (m_h - 20); //ºöÂÔ±ßÔµ
	N = 6;
	m_eps = eps;
	return true;
}

void MySolver::setImages(u32 *Ii, u32 *I, u32 *D, u32 *Y) {
	I_i = Ii;
	I_m = I;
	D_m = D;
	Y_m = Y;
}

real MySolver::Optimize(DVec& xStart, int nMaxIter, bool showInfo) {
	DMat jac(M, N);
	DMat JacTJac(N, N);
	DVec fx(M), fx1(M), h(N), g(N);
	Eigen::LDLT<DMat> solver;
	real f_energy = 0;

	DVec yx(M);
	DMat JacYx(M, N);
	for (int i = 0; i < M; i++) {
		yx[i] = double(Y_m[i]&0xff) / 5.0;
	}

	//Gauss-Newton Optimization
	for (int iter = 0; iter<nMaxIter; iter++)
	{
		CalcJacobiFunc(xStart, jac);	//J
		for (int i = 0; i < M; i++) {
			for (int j = 0; j < N; j++) {
				JacYx(i, j) = jac(i, j) * yx[i];
			}
		}

		JacTJac = jac.transpose() * jac;
		//JacTJac = JacYx.transpose() * jac;

		f_energy = CalcEnergyFunc(xStart, fx);	//f

												//solve: J'J h =  - J' f(x)
		g = jac.transpose() * (-fx);
		//g = JacYx.transpose() * (-fx);

		solver.compute(JacTJac);
		h = solver.solve(g);
		real normv = xStart.norm();

		if (iter < 5) {
			xStart = xStart + h;
			for (int i = 0; i < N; i++)
			printf("xStart[%d]: %.9lf\n", i, xStart[i]);
		}
		else {
			for (real alpha = 1; alpha > 1e-15; alpha *= 0.5)
			{
				DVec x = xStart + h;
				if (useBound)
				{
					x = x.cwiseMin(x_upper);
					x = x.cwiseMax(x_lower);
				}
				real f1_energy = CalcEnergyFunc(x, fx1);	//f
															//if (f1_energy > f_energy)
				if (fx1.dot(fx1) > fx.dot(fx))
					h = h * 0.5;
				else
				{
					xStart = x;
					break;
				}
			}
		}
		real normh = h.norm();

		if (showInfo)
			printf("lalala Gauss-Newton: %d -- %f, energy: %f\n", iter, normh / (normv + m_eps), fx.dot(fx));

		//if (normh / (normv + m_eps) < real(1e-5)) break;

		if (normh < (normv + real(1e-6)) * real(1e-6))
			break;
	}

	return f_energy;
}

void MySolver::CalcJacobiFunc(DVec& x, DMat& jac)
{
	const static real delta = real(m_eps);
	
	DMat jacobian_uv_ksai(2, 6);
	DMat jacobian_pixel_uv(1, 2);

	CameraPose Tvl(m_camera_intrinsic, x);
	Vector6d update_se3; //¸üÐÂÁ¿
	update_se3.setZero();
	update_se3(0, 0) = m_eps;
	update_se3(1, 0) = m_eps;
	update_se3(2, 0) = m_eps;
	update_se3(3, 0) = m_eps;
	update_se3(4, 0) = m_eps;
	update_se3(5, 0) = m_eps;

	int su = 0;
	int index = -1;
	for (int i = 10; i < m_h - 10; ++i) {
		for (int j = 10; j < m_w - 10; ++j) {
			++index;
			Eigen::Vector3d q, u;
			if (!getCoordInCamera2(i, j, Tvl, update_se3, q, u)) continue;

			//double xx = u[0] / u[2], yy = m_h - (u[1] / u[2]) - 1;
			double xx = u[0], yy = m_h - u[1] - 1;

			//std::cout << "index : " << index << std::endl;
			//printf("xx : %lf , yy : %lf\n", xx, yy);

			if (xx > 0 && xx < m_w - 1 && yy > 0 && yy < m_h - 1) {
				jacobian_pixel_uv(0, 0) = (rgba2gray(binterd_u32(I_i, xx + 1, yy, m_w)) 
										- rgba2gray(binterd_u32(I_i, xx - 1, yy, m_w))) / 2;
				jacobian_pixel_uv(0, 1) = (rgba2gray(binterd_u32(I_i, xx, yy + 1, m_w))
										- rgba2gray(binterd_u32(I_i, xx, yy - 1, m_w))) / 2;
			}
			else continue;

			su++;
			//printf("sum: %d - xx : %lf , yy : %lf\n", su, xx, yy);
			//printf("i = %d, j = %d\n", i, j);

			double X = q[0], Y = q[1], Z = q[2];
			jacobian_uv_ksai(0, 0) = G.g_fx / Z;
			jacobian_uv_ksai(1, 0) = 0;
			jacobian_uv_ksai(0, 1) = 0;
			jacobian_uv_ksai(1, 1) = G.g_fy / Z;
			jacobian_uv_ksai(0, 2) = -(G.g_fx * X) / (Z * Z);
			jacobian_uv_ksai(1, 2) = -(G.g_fy * Y) / (Z * Z);
			jacobian_uv_ksai(0, 3) = -(G.g_fx * X * Y) / (Z * Z);
			jacobian_uv_ksai(1, 3) = -G.g_fy - (G.g_fy * Y * Y) / (Z * Z);
			jacobian_uv_ksai(0, 4) =  G.g_fx + (G.g_fx * X * X) / (Z * Z);
			jacobian_uv_ksai(1, 4) = -(G.g_fy * X * Y) / (Z * Z);
			jacobian_uv_ksai(0, 5) = -(G.g_fx * Y) / Z;
			jacobian_uv_ksai(1, 5) =  (G.g_fy * X) / Z;

			jac.row(index) = jacobian_pixel_uv * jacobian_uv_ksai;
			//std::cout << "jac(" << index << ") = " << jac.row(index) << std::endl;
		}
	}
	/*
	const static real delta = real(m_eps);
	DMat jacobian_uv_ksai(2,6);
	DMat jacobian_pixel_uv(1,2);

	CameraPose Tvl(m_camera_intrinsic, AxisAngle(x[0], x[1], x[2]), double3(x[3], x[4], x[5]));

	double fx_ = m_camera_intrinsic[0][0];
	double fy_ = m_camera_intrinsic[1][1];
	double X, Y, invZ, invZ_2;
	double3 point;

	for (int i = 10; i < m_h - 10; ++i) {
		for (int j = 10; j < m_w - 10; ++j) {
			double3 point_in_m = 
			point = Tvl.rot * double3()
		}
	}

	jacobian_uv_ksai(0, 0) = -x*y*invz_2 *fx_;
	jacobian_uv_ksai(0, 1) = (1 + (x*x*invz_2)) *fx_;
	jacobian_uv_ksai(0, 2) = -y*invz *fx_;
	jacobian_uv_ksai(0, 3) = invz *fx_;
	jacobian_uv_ksai(0, 4) = 0;
	jacobian_uv_ksai(0, 5) = -x*invz_2 *fx_;

	jacobian_uv_ksai(1, 0) = -(1 + y*y*invz_2) *fy_;
	jacobian_uv_ksai(1, 1) = x*y*invz_2 *fy_;
	jacobian_uv_ksai(1, 2) = x*invz *fy_;
	jacobian_uv_ksai(1, 3) = 0;
	jacobian_uv_ksai(1, 4) = invz *fy_;
	jacobian_uv_ksai(1, 5) = -y*invz_2 *fy_;

	DVec fx(M), fxp(M), fxn(M);
	CalcEnergyFunc(x, fx);

	for (int j = 0; j<N; j++)
	{
		real d = real(1e-4)*x[j]; // force evaluation
		d = fabs(d);
		if (d<delta) d = delta;

		x[j] += d;
		CalcEnergyFunc(x, fxp);
		x[j] -= 2 * d;
		CalcEnergyFunc(x, fxn);
		x[j] += d;

		d = real(1.0) / (2 * d);
		for (int i = 0; i<M; i++)
		{
			jac(i, j) = (fxp[i] - fxn[i])*d;
		}

	}//end for j
	*/
}

real MySolver::CalcEnergyFunc(const DVec& x, DVec& fx)
{
	fx.setZero();

	CameraPose Tvl(m_camera_intrinsic, x);
	Eigen::Matrix3d A = m_camera_intrinsic;
	Eigen::Matrix3d invA = m_camera_intrinsic.inverse();

	int index = -1;
	for (int i = 10; i < m_h - 10; i++) {
		for (int j = 10; j < m_w - 10; j++) {
			index++;
			//?!
			double d = getDepth(D_m[i*m_w + j]);
			if (d == 0 || rgba2gray(I_m[i*m_w + j]) == 0)
				continue;
			Eigen::Vector3d m_M(j*d, (m_h - i - 1)*d, d);
			Eigen::Vector3d m_i = A*Tvl.R * invA * m_M + A*Tvl.t;

			double xx = m_i[0] / m_i[2];
			double yy = m_h - m_i[1] / m_i[2] - 1;

			double tmp_f;
			if (xx >= 0 && xx < m_w && yy >= 0 && yy < m_h) {
				tmp_f = rgba2gray(binterd_u32(I_i, xx, yy, m_w));
				if (tmp_f == 0) continue;
			}
			else continue;

			fx[index] = (tmp_f - rgba2gray(I_m[i*m_w + j]));
			//printf("%lf\n", fx[index]);
		}
	}
	return fx.dot(fx);


	//DVec yx(M);
	//yx.setZero();

	//CameraPose Tvl;
	/*
	//Tvl = CameraPose(m_camera_intrinsic, x[0], x[1], x[2], x[3], x[4], x[5]);
	Tvl = CameraPose(m_camera_intrinsic, AxisAngle(x[0], x[1], x[2]), double3(x[3], x[4], x[5]));
	Eigen::Matrix3d A = m_camera_intrinsic;
	Eigen::Matrix3d invA = m_camera_intrinsic.inv();
	//printf("%d %d\n", fx.rows(), fx.cols());

	int index = -1;
	for (int i = 0; i < m_h; i++) {
		for (int j = 0; j < m_w; j++) {
			index++;
			double d = getDepth(D_m[i*m_w + j]);
			if (d == 0 || rgba2gray(I_m[i*m_w + j]) == 0)
				continue;
			double3 m_M(j*d, (m_h - i - 1)*d, d);
			double3 m_i = A*Tvl.rot * invA * m_M + A*Tvl.translation;

			double xx = m_i[0] / m_i[2];
			double yy = m_h - m_i[1] / m_i[2] - 1;

			double tmp_f;
			if (xx >= 0 && xx < m_w && yy >= 0 && yy < m_h) {
				tmp_f = rgba2gray(binterd_u32(I_i, xx, yy, m_w));
				if (tmp_f == 0) continue;
			}
			else continue;

			fx[index] = (tmp_f - rgba2gray(I_m[i*m_w + j]));
			//printf("%lf\n", fx[index]);
		}
	}
	*/
	return fx.dot(fx);
}


bool PoseEstimation::init(GlobalCoeff _g) {
	G = _g;
	m_NewtonSolver.init(_g);
	return true;
}

CameraPose PoseEstimation::DenseImageAlignment(CameraPose pose, u32 *I, u32 *D, u32 *Y, u32 *new_I) {
	if (!I || !D || !Y || !new_I) return CameraPose();
	
	m_NewtonSolver.setImages(new_I, I, D, Y);
	DVec xstart(6);
	for (int i = 0; i < 6; ++i) {
		xstart[i] = 0.0;
	}
	m_NewtonSolver.Optimize(xstart, 20, true);

	printf("endx: ");
	for (int i = 0; i < 6; ++i) {
		printf("%lf ", xstart[i]);
	}
	printf("\n");

	Vector6d se3 = xstart;
	CameraPose Tvl(m_camera_intrinsic, se3);
	//CameraPose Tvl(m_camera_intrinsic, xstart[0], xstart[1], xstart[2], xstart[3], xstart[4], xstart[5]);
	CameraPose Tv;
	Tv.intrinsics = m_camera_intrinsic;
	Tv.R = Tvl.R * pose.R;
	Tv.t = Tvl.R * pose.t + Tvl.t;
	Tv.refreshByARt();

	/*
	Tv.A = m_camera_intrinsic;
	Tv.rot = Tvl.rot * pose.rot;
	Tv.translation = Tvl.rot * pose.translation + Tvl.translation;
	Tv.refreshByARt();
	*/
	m_NewtonSolver.setImages(NULL, NULL, NULL, NULL);

	return Tv;
}


} //namespace mf
