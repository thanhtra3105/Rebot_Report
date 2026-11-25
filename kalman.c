/*
 * kalman.c
 *
 * Consolidated Kalman filters: 1D (attitude), 3D (GPS axis), 4D (altitude).
 * See kalman.h for the state definitions.
 */

#include "kalman.h"
#include <math.h>
#include <string.h>

#define KF_EPSILON 1.0e-6f

/* Return early if a CMSIS-DSP call fails. */
#define MAT_CALL(expr)                                   \
	do {                                                 \
		arm_status status_ = (expr);                     \
		if (status_ != ARM_MATH_SUCCESS) return status_; \
	} while (0)

/* ======================================================================
 *  1D Kalman filter (scalar)
 *
 *  Predict : angle += dt * gyro        P += dt^2 * gyro_var
 *  Update  : K = P / (P + accel_var)
 *            angle += K * (accel_angle - angle)
 *            P = (1 - K) * P
 * ====================================================================== */

void Kalman1D_Init(Kalman1D_t *kf, float init_angle, float init_uncertainty,
		float gyro_std, float accel_std) {
	kf->angle = init_angle;
	kf->uncertainty = init_uncertainty;
	kf->gyro_var = gyro_std * gyro_std;
	kf->accel_var = accel_std * accel_std;
}

float Kalman1D_Update(Kalman1D_t *kf, float gyro_rate, float accel_angle,
		float dt) {
	/* Predict: integrate the gyro, grow the uncertainty */
	kf->angle += dt * gyro_rate;
	kf->uncertainty += dt * dt * kf->gyro_var;

	/* Update: correct with the accelerometer angle */
	float gain = kf->uncertainty / (kf->uncertainty + kf->accel_var);
	kf->angle += gain * (accel_angle - kf->angle);
	kf->uncertainty *= (1.0f - gain);

	return kf->angle;
}

/* ======================================================================
 *  Shared matrix-based Kalman core (CMSIS-DSP)
 * ====================================================================== */

static void Core_Init(KalmanCore_t *kf, uint8_t n) {
	memset(kf, 0, sizeof(*kf));
	kf->n = n;
}

/* Force P to be symmetric and keep the diagonal positive. */
static void Core_Symmetrize(float32_t *P, uint16_t n) {
	for (uint16_t i = 0; i < n; ++i) {
		for (uint16_t j = i + 1; j < n; ++j) {
			float32_t s = 0.5f * (P[i * n + j] + P[j * n + i]);
			P[i * n + j] = s;
			P[j * n + i] = s;
		}
		if (P[i * n + i] < KF_EPSILON)
			P[i * n + i] = KF_EPSILON;
	}
}

/* Closed-form inverse for the small S matrices (1x1 or 2x2) so that the
 * input is never modified and no workspace is needed. */
static arm_status Inverse_Small(const float32_t *S, float32_t *Sinv, uint16_t m) {
	if (m == 1U) {
		if (!(S[0] > KF_EPSILON))
			return ARM_MATH_SINGULAR;
		Sinv[0] = 1.0f / S[0];
		return ARM_MATH_SUCCESS;
	}
	if (m == 2U) {
		float32_t det = S[0] * S[3] - S[1] * S[2];
		if (!(det > KF_EPSILON))
			return ARM_MATH_SINGULAR;
		Sinv[0] = S[3] / det;
		Sinv[1] = -S[1] / det;
		Sinv[2] = -S[2] / det;
		Sinv[3] = S[0] / det;
		return ARM_MATH_SUCCESS;
	}
	return ARM_MATH_ARGUMENT_ERROR;
}

/*
 * Predict step:
 *     x = F*x + B*u
 *     P = F*P*F^T + Q
 * F: n x n, B: n x 1, Q: n x n (all row-major)
 */
static arm_status Core_Predict(KalmanCore_t *kf, const float32_t *F_d,
		const float32_t *B_d, float32_t u, const float32_t *Q_d) {
	const uint16_t n = kf->n;
	float32_t Ft_d[KF_MAX_N * KF_MAX_N], FP_d[KF_MAX_N * KF_MAX_N];
	float32_t Pp_d[KF_MAX_N * KF_MAX_N];
	float32_t FX_d[KF_MAX_N], BU_d[KF_MAX_N], Xp_d[KF_MAX_N];

	arm_matrix_instance_f32 F, B, Q, X, P, Ft, FP, Pp, FX, BU, Xp;
	arm_mat_init_f32(&F, n, n, (float32_t*) F_d);
	arm_mat_init_f32(&B, n, 1, (float32_t*) B_d);
	arm_mat_init_f32(&Q, n, n, (float32_t*) Q_d);
	arm_mat_init_f32(&X, n, 1, kf->x);
	arm_mat_init_f32(&P, n, n, kf->P);
	arm_mat_init_f32(&Ft, n, n, Ft_d);
	arm_mat_init_f32(&FP, n, n, FP_d);
	arm_mat_init_f32(&Pp, n, n, Pp_d);
	arm_mat_init_f32(&FX, n, 1, FX_d);
	arm_mat_init_f32(&BU, n, 1, BU_d);
	arm_mat_init_f32(&Xp, n, 1, Xp_d);

	/* State prediction: x = F*x + B*u */
	MAT_CALL(arm_mat_mult_f32(&F, &X, &FX));
	MAT_CALL(arm_mat_scale_f32(&B, u, &BU));
	MAT_CALL(arm_mat_add_f32(&FX, &BU, &Xp));

	/* Covariance prediction: P = F*P*F^T + Q */
	MAT_CALL(arm_mat_trans_f32(&F, &Ft));
	MAT_CALL(arm_mat_mult_f32(&F, &P, &FP));
	MAT_CALL(arm_mat_mult_f32(&FP, &Ft, &Pp));
	MAT_CALL(arm_mat_add_f32(&Pp, &Q, &Pp));

	memcpy(kf->x, Xp_d, sizeof(float32_t) * n);
	memcpy(kf->P, Pp_d, sizeof(float32_t) * n * n);
	Core_Symmetrize(kf->P, n);
	return ARM_MATH_SUCCESS;
}

/*
 * Update step with an m-dimensional measurement z = H*x + v, v ~ N(0, R):
 *     y = z - H*x                      (innovation)
 *     S = H*P*H^T + R
 *     K = P*H^T * S^-1                 (Kalman gain)
 *     x = x + K*y
 *     P = (I-KH) P (I-KH)^T + K R K^T  (Joseph form)
 * H: m x n, R: m x m, z: m x 1 (all row-major)
 */
static arm_status Core_Update(KalmanCore_t *kf, uint16_t m,
		const float32_t *z_d, const float32_t *H_d, const float32_t *R_d) {
	const uint16_t n = kf->n;

	float32_t Ht_d[KF_MAX_N * KF_MAX_M], PHt_d[KF_MAX_N * KF_MAX_M];
	float32_t S_d[KF_MAX_M * KF_MAX_M], Sinv_d[KF_MAX_M * KF_MAX_M];
	float32_t K_d[KF_MAX_N * KF_MAX_M], Kt_d[KF_MAX_M * KF_MAX_N];
	float32_t KR_d[KF_MAX_N * KF_MAX_M];
	float32_t HX_d[KF_MAX_M], y_d[KF_MAX_M], Ky_d[KF_MAX_N], Xn_d[KF_MAX_N];
	float32_t I_d[KF_MAX_N * KF_MAX_N], KH_d[KF_MAX_N * KF_MAX_N];
	float32_t A_d[KF_MAX_N * KF_MAX_N], At_d[KF_MAX_N * KF_MAX_N];
	float32_t AP_d[KF_MAX_N * KF_MAX_N], APAt_d[KF_MAX_N * KF_MAX_N];
	float32_t KRKt_d[KF_MAX_N * KF_MAX_N], Pn_d[KF_MAX_N * KF_MAX_N];

	arm_matrix_instance_f32 H, R, Z, X, P, Ht, PHt, S, Sinv, K, Kt, KR;
	arm_matrix_instance_f32 HX, y, Ky, Xn, I, KH, A, At, AP, APAt, KRKt, Pn;

	arm_mat_init_f32(&H, m, n, (float32_t*) H_d);
	arm_mat_init_f32(&R, m, m, (float32_t*) R_d);
	arm_mat_init_f32(&Z, m, 1, (float32_t*) z_d);
	arm_mat_init_f32(&X, n, 1, kf->x);
	arm_mat_init_f32(&P, n, n, kf->P);
	arm_mat_init_f32(&Ht, n, m, Ht_d);
	arm_mat_init_f32(&PHt, n, m, PHt_d);
	arm_mat_init_f32(&S, m, m, S_d);
	arm_mat_init_f32(&Sinv, m, m, Sinv_d);
	arm_mat_init_f32(&K, n, m, K_d);
	arm_mat_init_f32(&Kt, m, n, Kt_d);
	arm_mat_init_f32(&KR, n, m, KR_d);
	arm_mat_init_f32(&HX, m, 1, HX_d);
	arm_mat_init_f32(&y, m, 1, y_d);
	arm_mat_init_f32(&Ky, n, 1, Ky_d);
	arm_mat_init_f32(&Xn, n, 1, Xn_d);
	arm_mat_init_f32(&I, n, n, I_d);
	arm_mat_init_f32(&KH, n, n, KH_d);
	arm_mat_init_f32(&A, n, n, A_d);
	arm_mat_init_f32(&At, n, n, At_d);
	arm_mat_init_f32(&AP, n, n, AP_d);
	arm_mat_init_f32(&APAt, n, n, APAt_d);
	arm_mat_init_f32(&KRKt, n, n, KRKt_d);
	arm_mat_init_f32(&Pn, n, n, Pn_d);

	/* S = H*P*H^T + R */
	MAT_CALL(arm_mat_trans_f32(&H, &Ht));
	MAT_CALL(arm_mat_mult_f32(&P, &Ht, &PHt));
	MAT_CALL(arm_mat_mult_f32(&H, &PHt, &S));
	MAT_CALL(arm_mat_add_f32(&S, &R, &S));

	/* K = P*H^T * S^-1 */
	MAT_CALL(Inverse_Small(S_d, Sinv_d, m));
	MAT_CALL(arm_mat_mult_f32(&PHt, &Sinv, &K));

	/* Innovation y = z - H*x */
	MAT_CALL(arm_mat_mult_f32(&H, &X, &HX));
	MAT_CALL(arm_mat_sub_f32(&Z, &HX, &y));
	kf->last_innovation = y_d[0];
	kf->last_innovation_var = S_d[0];

	/* State correction: x = x + K*y */
	MAT_CALL(arm_mat_mult_f32(&K, &y, &Ky));
	MAT_CALL(arm_mat_add_f32(&X, &Ky, &Xn));

	/* Joseph form: A = I - K*H ; P = A*P*A^T + K*R*K^T */
	memset(I_d, 0, sizeof(I_d));
	for (uint16_t i = 0; i < n; ++i)
		I_d[i * n + i] = 1.0f;

	MAT_CALL(arm_mat_mult_f32(&K, &H, &KH));
	MAT_CALL(arm_mat_sub_f32(&I, &KH, &A));
	MAT_CALL(arm_mat_trans_f32(&A, &At));
	MAT_CALL(arm_mat_mult_f32(&A, &P, &AP));
	MAT_CALL(arm_mat_mult_f32(&AP, &At, &APAt));

	MAT_CALL(arm_mat_mult_f32(&K, &R, &KR));
	MAT_CALL(arm_mat_trans_f32(&K, &Kt));
	MAT_CALL(arm_mat_mult_f32(&KR, &Kt, &KRKt));
	MAT_CALL(arm_mat_add_f32(&APAt, &KRKt, &Pn));

	memcpy(kf->x, Xn_d, sizeof(float32_t) * n);
	memcpy(kf->P, Pn_d, sizeof(float32_t) * n * n);
	Core_Symmetrize(kf->P, n);
	return ARM_MATH_SUCCESS;
}

/* ======================================================================
 *  3D Kalman filter: [pos, vel, accel_bias] per horizontal axis
 *
 *  Kinematics with bias-corrected acceleration a = u - bias:
 *      pos' = pos + vel*dt + 0.5*a*dt^2
 *      vel' = vel + a*dt
 *      bias' = bias
 *  => F = [1 dt -0.5dt^2; 0 1 -dt; 0 0 1],  B = [0.5dt^2; dt; 0]
 * ====================================================================== */

arm_status Kalman3D_Init(Kalman3D_t *kf, float init_pos, float init_vel,
		float q_accel, float q_bias) {
	if (kf == NULL)
		return ARM_MATH_ARGUMENT_ERROR;

	Core_Init(&kf->core, 3);
	kf->core.x[0] = init_pos;
	kf->core.x[1] = init_vel;
	kf->core.x[2] = 0.0f;

	/* Initial covariance: identity */
	kf->core.P[0] = 1.0f;
	kf->core.P[4] = 1.0f;
	kf->core.P[8] = 1.0f;

	kf->q_accel = q_accel;
	kf->q_bias = q_bias;
	return ARM_MATH_SUCCESS;
}

arm_status Kalman3D_Predict(Kalman3D_t *kf, float accel_earth, float dt) {
	if (kf == NULL || !(dt > 0.0f) || !isfinite(dt) || !isfinite(accel_earth))
		return ARM_MATH_ARGUMENT_ERROR;

	const float32_t dt2 = dt * dt;
	const float32_t dt3 = dt2 * dt;

	const float32_t F[9] = {
		1.0f, dt,   -0.5f * dt2,
		0.0f, 1.0f, -dt,
		0.0f, 0.0f, 1.0f
	};
	const float32_t B[3] = { 0.5f * dt2, dt, 0.0f };

	/* Continuous white-noise acceleration model:
	 *   Q = q_accel * [dt^3/3 dt^2/2; dt^2/2 dt]  (pos/vel block)
	 *   plus a slow random walk on the bias. */
	float32_t Q[9] = { 0 };
	Q[0] = (1.0f / 3.0f) * dt3 * kf->q_accel;
	Q[1] = 0.5f * dt2 * kf->q_accel;
	Q[3] = Q[1];
	Q[4] = dt * kf->q_accel;
	Q[8] = kf->q_bias * dt;

	return Core_Predict(&kf->core, F, B, accel_earth, Q);
}

arm_status Kalman3D_Update(Kalman3D_t *kf, float gps_pos, float gps_vel,
		float r_pos, float r_vel) {
	if (kf == NULL || !isfinite(gps_pos) || !isfinite(gps_vel)
			|| !(r_pos > 0.0f) || !(r_vel > 0.0f))
		return ARM_MATH_ARGUMENT_ERROR;

	/* GPS observes position and velocity directly */
	const float32_t z[2] = { gps_pos, gps_vel };
	const float32_t H[6] = {
		1.0f, 0.0f, 0.0f,
		0.0f, 1.0f, 0.0f
	};
	const float32_t R[4] = { r_pos, 0.0f, 0.0f, r_vel };

	return Core_Update(&kf->core, 2, z, H, R);
}

/* ======================================================================
 *  4D Kalman filter: [altitude, velocity, accel_bias, baro_bias]
 *
 *      alt' = alt + vel*dt + 0.5*(u - acc_bias)*dt^2
 *      vel' = vel + (u - acc_bias)*dt
 *  => F = [1 dt -0.5dt^2 0; 0 1 -dt 0; 0 0 1 0; 0 0 0 1],
 *     B = [0.5dt^2; dt; 0; 0]
 * ====================================================================== */

#define KF4_SIGMA_ACCEL      10.0f   /* accel noise std-dev (cm/s^2)        */
#define KF4_Q_ACCEL_BIAS     1.0e-5f /* accel bias drifts very slowly       */
#define KF4_Q_BARO_BIAS      0.01f   /* baro bias random walk               */
#define KF4_TRANSITION_ALT   100.0f  /* below this, baro suffers ground
                                        effect from the propellers (cm)    */

arm_status Kalman4D_Init(Kalman4D_t *kf, float initial_alt_cm) {
	if (kf == NULL)
		return ARM_MATH_ARGUMENT_ERROR;

	Core_Init(&kf->core, 4);
	kf->core.x[0] = initial_alt_cm;

	/* Initial covariance (diagonal) */
	kf->core.P[0] = 10.0f;  /* altitude     */
	kf->core.P[5] = 10.0f;  /* velocity     */
	kf->core.P[10] = 1.0f;  /* accel bias   */
	kf->core.P[15] = 5.0f;  /* baro bias    */
	return ARM_MATH_SUCCESS;
}

arm_status Kalman4D_Predict(Kalman4D_t *kf, float acc_z_cm_s2, float dt) {
	if (kf == NULL || !(dt > 0.0f) || !isfinite(dt) || !isfinite(acc_z_cm_s2))
		return ARM_MATH_ARGUMENT_ERROR;

	const float32_t dt2 = dt * dt;
	const float32_t dt3 = dt2 * dt;
	const float32_t dt4 = dt2 * dt2;

	const float32_t F[16] = {
		1.0f, dt,   -0.5f * dt2, 0.0f,
		0.0f, 1.0f, -dt,         0.0f,
		0.0f, 0.0f, 1.0f,        0.0f,
		0.0f, 0.0f, 0.0f,        1.0f
	};
	const float32_t B[4] = { 0.5f * dt2, dt, 0.0f, 0.0f };

	/* Q: discrete white-noise acceleration on [alt, vel] + bias random walks */
	const float32_t var_acc = KF4_SIGMA_ACCEL * KF4_SIGMA_ACCEL;
	float32_t Q[16] = { 0 };
	Q[0] = 0.25f * dt4 * var_acc;
	Q[1] = 0.5f * dt3 * var_acc;
	Q[4] = Q[1];
	Q[5] = dt2 * var_acc;
	Q[10] = KF4_Q_ACCEL_BIAS * dt;
	Q[15] = KF4_Q_BARO_BIAS * dt;

	return Core_Predict(&kf->core, F, B, acc_z_cm_s2, Q);
}

arm_status Kalman4D_UpdateMeasure(Kalman4D_t *kf, float measurement_cm,
		float sigma_cm, const float H[4]) {
	if (kf == NULL || H == NULL || !isfinite(measurement_cm)
			|| !(sigma_cm > 0.0f) || !isfinite(sigma_cm))
		return ARM_MATH_ARGUMENT_ERROR;

	const float32_t z[1] = { measurement_cm };
	const float32_t R[1] = { sigma_cm * sigma_cm };

	return Core_Update(&kf->core, 1, z, H, R);
}

/* Barometer noise grows near the ground (propeller ground effect). */
float Kalman4D_GetSigmaBaro(float est_alt_cm) {
	float sigma = 20.0f; /* nominal baro noise at altitude (cm) */

	if (est_alt_cm < KF4_TRANSITION_ALT) {
		float factor = (KF4_TRANSITION_ALT - est_alt_cm) / KF4_TRANSITION_ALT;
		if (factor < 0.0f)
			factor = 0.0f;
		sigma += factor * 200.0f; /* up to ~220 cm on the ground */
	}
	return sigma;
}

/* Rangefinder is trusted only at low altitude and rejected when it jumps
 * without matching vertical acceleration (terrain change, not real motion). */
float Kalman4D_GetSigmaRange(float range_alt_cm, float est_alt_cm,
		float acc_z_cm_s2) {
	float sigma = 2.0f; /* very accurate at short range (cm) */

	/* Too high for the rangefinder: effectively ignore it */
	if (range_alt_cm > 100.0f || est_alt_cm > 200.0f)
		return 10000.0f;

	/* Large range jump with small acceleration => terrain, inflate R */
	float diff = fabsf(range_alt_cm - est_alt_cm);
	if (diff > 20.0f && fabsf(acc_z_cm_s2) < 200.0f)
		sigma += diff * 10.0f;

	return sigma;
}

arm_status Kalman4D_UpdateBaro(Kalman4D_t *kf, float baro_alt_cm) {
	static const float H_baro[4] = { 1.0f, 0.0f, 0.0f, 1.0f }; /* alt + baro_bias */

	if (kf == NULL)
		return ARM_MATH_ARGUMENT_ERROR;

	float sigma = Kalman4D_GetSigmaBaro(kf->core.x[0]);
	return Kalman4D_UpdateMeasure(kf, baro_alt_cm, sigma, H_baro);
}

arm_status Kalman4D_UpdateRange(Kalman4D_t *kf, float range_alt_cm,
		float acc_z_cm_s2) {
	static const float H_range[4] = { 1.0f, 0.0f, 0.0f, 0.0f }; /* true altitude */

	if (kf == NULL)
		return ARM_MATH_ARGUMENT_ERROR;

	float sigma = Kalman4D_GetSigmaRange(range_alt_cm, kf->core.x[0],
			acc_z_cm_s2);
	return Kalman4D_UpdateMeasure(kf, range_alt_cm, sigma, H_range);
}
