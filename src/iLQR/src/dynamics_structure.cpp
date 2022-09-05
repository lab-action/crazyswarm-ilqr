#include <iostream>

#include <unsupported/Eigen/AdolcForward>
#include <adolc/adolc.h>
#include <Eigen/Dense>

#include "dynamics_structure.h"

namespace Unicycle_Dynamics
{
	unicycle_state_tensor dynamics(const unicycle_state_tensor &x, const unicycle_input_tensor &u)
	{
		unicycle_state_tensor x_dot;
		adouble px, py, theta, v, omega, a;

		x_dot[0] = x[3] * cos(x[2]);
		x_dot[1] = x[3] * sin(x[2]);
		x_dot[2] = u[0];
		x_dot[3] = u[1];

		return x_dot;
	};
	unicycle2_state_tensor dynamics_2(const unicycle2_state_tensor &x, const unicycle2_input_tensor &u)
	{
		unicycle2_state_tensor x_dot;

		for (int i = 0; i < 2; ++i)
			x_dot.segment(4 * i, 4) = dynamics(x.segment(4 * i, 4), u.segment(2 * i, 2));

		return x_dot;
	};
	unicycle4_state_tensor dynamics_4(const unicycle4_state_tensor &x, const unicycle4_input_tensor &u)
	{
		unicycle4_state_tensor x_dot;

		for (int i = 0; i < 4; ++i)
			x_dot.segment(4 * i, 4) = dynamics(x.segment(4 * i, 4), u.segment(2 * i, 2));

		return x_dot;
	};

	unicycle3_state_tensor dynamics_3(const unicycle3_state_tensor &x, const unicycle3_input_tensor &u)
	{
		unicycle3_state_tensor x_dot;
		for (int i = 0; i < 3; ++i)
			x_dot.segment(4 * i, 4) = dynamics(x.segment(4 * i, 4), u.segment(2 * i, 2));

		return x_dot;
	};

	unicycle6_state_tensor dynamics_6(const unicycle6_state_tensor &x, const unicycle6_input_tensor &u)
	{
		unicycle6_state_tensor x_dot;
		for (int i = 0; i < 6; ++i)
			x_dot.segment(4 * i, 4) = dynamics(x.segment(4 * i, 4), u.segment(2 * i, 2));

		return x_dot;
	}
}

namespace Drone_Dynamics
{

	drone_state_tensor dynamics(const drone_state_tensor &X, const drone_input_tensor &u)
	{
		drone_state_tensor X_dot;
		adouble x, y, z, phi, theta, psi, x_d, y_d, z_d, phi_d, theta_d, psi_d, w1, w2, w3, w4, thrust;
		x = X[0];
		y = X[1];
		z = X[2];
		phi = X[3];
		theta = X[4];
		psi = X[5];
		x_d = X[6];
		y_d = X[7];
		z_d = X[8];
		phi_d = X[9];
		theta_d = X[10];
		psi_d = X[11];

		w1 = u[0];
		w2 = u[1];
		w3 = u[2];
		w4 = u[3];

		Eigen::Matrix<adouble, 3, 3> Rsb, mat, mat_inv, I, I_inv;

		Rsb << cos(theta) * cos(psi), cos(theta) * sin(psi), -sin(theta),
			-cos(phi) * sin(psi) + sin(theta) * cos(psi) * sin(phi), cos(psi) * cos(phi) + sin(theta) * sin(psi) * sin(phi), sin(phi) * cos(theta),
			sin(phi) * sin(psi) + cos(phi) * sin(theta) * cos(psi), -sin(phi) * cos(psi) + cos(phi) * sin(theta) * sin(psi), cos(theta) * cos(phi);

		mat << 1, 0, -sin(theta),
			0, cos(phi), sin(phi) * cos(theta),
			0, -sin(phi), cos(theta) * cos(phi);

		mat_inv << 1, sin(phi) * tan(theta), cos(phi) * tan(theta),
			0, cos(phi), -sin(phi),
			0, sin(phi) / cos(theta), cos(phi) / cos(theta);

		I << I_xx, 0, 0,
			0, I_yy, 0,
			0, 0, I_zz;

		I_inv << 1 / I_xx, 0, 0,
			0, 1 / I_yy, 0,
			0, 0, 1 / I_zz;

		thrust = C_T * (w1 * w1 + w2 * w2 + w3 * w3 + w4 * w4);

		Eigen::Matrix<adouble, 3, 1> angle_dots, Omega_in, v_dots, gravity_term, thrust_term, pos_dots, Omega_b, L_b, Moments, angle_ddots, temp;
		Eigen::Matrix<adouble, 3, 3> skew1, skew2;
		pos_dots << x_d, y_d, z_d;
		gravity_term << 0, 0, g;
		thrust_term << 0, 0, thrust / mass;
		angle_dots << phi_d, theta_d, psi_d;
		Omega_in = Rsb * (mat * angle_dots);
		//	temp=Omega_in.cross(pos_dots);
		skew1 << 0, -Omega_in[2], Omega_in[1],
			Omega_in[2], 0, -Omega_in[0],
			-Omega_in[1], Omega_in[0], 0;
		v_dots = Rsb * thrust_term - gravity_term - skew1 * pos_dots; // Omega_in.cross(pos_dots);

		X_dot[0] = x_d;
		X_dot[1] = y_d;
		X_dot[2] = z_d;
		X_dot[3] = phi_d;
		X_dot[4] = theta_d;
		X_dot[5] = psi_d;
		X_dot[6] = v_dots[0];
		X_dot[7] = v_dots[1];
		X_dot[8] = v_dots[2];

		Omega_b = mat * angle_dots;

		L_b = I * Omega_b;

		Moments << d * C_T / sqrt(2) * (-w1 * w1 - w2 * w2 + w3 * w3 + w4 * w4),
			d * C_T / sqrt(2) * (-w1 * w1 + w2 * w2 + w3 * w3 - w4 * w4),
			C_D * (-w1 * w1 + w2 * w2 - w3 * w3 + w4 * w4);

		skew2 << 0, -Omega_b[2], Omega_b[1],
			Omega_b[2], 0, -Omega_b[0],
			-Omega_b[1], Omega_b[0], 0;

		angle_ddots = mat_inv * (I_inv * (Moments - skew2 * L_b)); // mat_inv*(I_inv*(Moments-Omega_b.cross(L_b))); //Euler's eqn.

		X_dot[9] = angle_ddots[0];
		X_dot[10] = angle_ddots[1];
		X_dot[11] = angle_ddots[2];

		return X_dot;
	};

	drone2_state_tensor dynamics_2(const drone2_state_tensor &X, const drone2_input_tensor &u)
	{
		drone2_state_tensor X_dot;
		drone_state_tensor X_dot1, X_dot2;
		X_dot1 = dynamics(X.segment(0, 12), u.segment(0, 4));
		X_dot2 = dynamics(X.segment(12, 12), u.segment(4, 4));
		X_dot << X_dot1, X_dot2;
		return X_dot;
	}

}

namespace Single_Integrator_3D
{
	state_tensor dynamics(const state_tensor &x, const input_tensor &u)
	{
		return u;
	};

	state_tensor2 dynamics2(const state_tensor2 &x, const input_tensor2 &u)
	{
		return u;
	}
}

namespace Double_Integrator_3D
{
	state_tensor dynamics(const state_tensor &x, const input_tensor &u)
	{
		state_tensor x_dot;
		x_dot[0] = x[3];
		x_dot[1] = x[4];
		x_dot[2] = x[5];
		x_dot[3] = u[0] / mb;
		x_dot[4] = u[1] / mb;
		x_dot[5] = u[2] / mb;

		return x_dot;
	};
}

namespace Drone_First_Order_Dynamics
{
	typedef Eigen::Matrix<adouble, 6, 1> drone_state_tensor;
	typedef Eigen::Matrix<adouble, 6, 1> drone_input_tensor;

	typedef Eigen::Matrix<adouble, 6 * 2, 1> drone2_state_tensor;
	typedef Eigen::Matrix<adouble, 6 * 2, 1> drone2_input_tensor;

	drone_state_tensor dynamics(const drone_state_tensor &X, const drone_input_tensor &input)
	{
		adouble x, y, z, phi, theta, psi, u, v, w, p, q, r, x_d, y_d, z_d, phi_d, theta_d, psi_d;
		drone_state_tensor x_dot;
		u = input[0];
		v = input[1];
		w = input[2];
		p = input[3];
		q = input[4];
		r = input[5];
		Eigen::Matrix<adouble, 3, 3> Rsb, mat, mat_inv;
		Eigen::Matrix<adouble, 3, 1> v_b, w_b, v_i, euler_d;

		v_b << u, v, w;
		w_b << p, q, r;

		Rsb << cos(theta) * cos(psi), cos(theta) * sin(psi), -sin(theta),
			-cos(phi) * sin(psi) + sin(theta) * cos(psi) * sin(phi), cos(psi) * cos(phi) + sin(theta) * sin(psi) * sin(phi), sin(phi) * cos(theta),
			sin(phi) * sin(psi) + cos(phi) * sin(theta) * cos(psi), -sin(phi) * cos(psi) + cos(phi) * sin(theta) * sin(psi), cos(theta) * cos(phi);

		mat_inv << 1, sin(phi) * tan(theta), cos(phi) * tan(theta),
			0, cos(phi), -sin(phi),
			0, sin(phi) / cos(theta), cos(phi) / cos(theta);

		v_i = Rsb * v_b;
		euler_d = mat_inv * w_b;

		x_d = v_i[0];
		y_d = v_i[1];
		z_d = v_i[2];

		phi_d = euler_d[0];
		theta_d = euler_d[1];
		psi_d = euler_d[2];

		x_dot << x_d, y_d, z_d, phi_d, theta_d, psi_d;

		return x_dot;
	}

	drone2_state_tensor dynamics_2(const drone2_state_tensor &x, const drone2_input_tensor &u)
	{
		drone2_state_tensor X_dot;
		drone_state_tensor X_dot1, X_dot2;
		X_dot1 = dynamics(x.segment(0, 6), u.segment(0, 6));
		X_dot2 = dynamics(x.segment(6, 6), u.segment(6, 6));
		X_dot << X_dot1, X_dot2;
		return X_dot;
	}
}
