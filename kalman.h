/*
 * kalman.h
 *
 * Consolidated Kalman filter module for the UAV flight controller (STM32H562).
 *
 *   - Kalman1D : scalar filter, fuses gyro rate + accelerometer angle
 *                (roll / pitch attitude).
 *   - Kalman3D : 3-state filter [pos, vel, accel_bias] for ONE horizontal
 *                axis, fuses IMU acceleration (predict) with GPS position and
 *                velocity (update). Use one instance for North, one for East.
 *   - Kalman4D : 4-state altitude filter [alt, vel, accel_bias, baro_bias],
 *                fuses IMU vertical acceleration (predict) with barometer and
 *                rangefinder (updates).
 *
 * The 3D and 4D filters share one matrix-based core built on CMSIS-DSP
 * (arm_mat_* functions). The covariance update uses the Joseph form.
 */

#ifndef INC_KALMAN_H_
#define INC_KALMAN_H_

#include <stdint.h>
#include "arm_math.h" /* CMSIS-DSP: arm_matrix_instance_f32, arm_mat_*_f32 */

#ifdef __cplusplus
extern "C" {
#endif

/* ======================================================================
 *  1D Kalman filter  (attitude angle: gyro + accelerometer)
 * ====================================================================== */

#define KALMAN1D_DEFAULT_UNCERTAINTY 4.0f /* initial variance (deg^2)     */
#define KALMAN1D_DEFAULT_GYRO_STD    4.0f /* gyro noise std-dev (deg/s)   */
#define KALMAN1D_DEFAULT_ACCEL_STD   3.0f /* accel angle noise std (deg)  */

typedef struct {
	float angle;       /* estimated angle (deg)                        */
	float uncertainty; /* estimate variance P (deg^2)                  */
	float gyro_var;    /* gyro process variance  = gyro_std^2          */
	float accel_var;   /* accel measurement variance = accel_std^2     */
} Kalman1D_t;

void Kalman1D_Init(Kalman1D_t *kf, float init_angle, float init_uncertainty,
		float gyro_std, float accel_std);

/* One full predict + update step. Returns the new angle estimate.
 * gyro_rate  : angular rate (deg/s)
 * accel_angle: angle computed from the accelerometer (deg)
 * dt         : time step (s)                                           */
float Kalman1D_Update(Kalman1D_t *kf, float gyro_rate, float accel_angle,
		float dt);

/* ======================================================================
 *  Shared matrix-based core (used by Kalman3D and Kalman4D)
 * ====================================================================== */

#define KF_MAX_N 4U /* max number of states       */
#define KF_MAX_M 2U /* max measurement dimension  */

typedef struct {
	uint8_t n;                          /* number of states            */
	float32_t x[KF_MAX_N];              /* state vector                */
	float32_t P[KF_MAX_N * KF_MAX_N];   /* covariance, row-major       */
	float32_t last_innovation;          /* diagnostics (1st component) */
	float32_t last_innovation_var;      /* S[0][0] of last update      */
} KalmanCore_t;

/* ======================================================================
 *  3D Kalman filter  (one horizontal axis: GPS + IMU)
 *
 *  State : x = [pos (m), vel (m/s), accel_bias (m/s^2)]
 *  Input : u = accel in earth frame (gravity removed)
 *  Meas. : z = [gps_pos, gps_vel]
 * ====================================================================== */

typedef struct {
	KalmanCore_t core;
	float32_t q_accel; /* accel noise spectral density (drone vibration) */
	float32_t q_bias;  /* bias random-walk rate (keep very small)        */
} Kalman3D_t;

arm_status Kalman3D_Init(Kalman3D_t *kf, float init_pos, float init_vel,
		float q_accel, float q_bias);

/* Predict step - call at the IMU rate (e.g. 100-500 Hz). */
arm_status Kalman3D_Predict(Kalman3D_t *kf, float accel_earth, float dt);

/* Update step - call when a new GPS fix arrives (e.g. 5-10 Hz).
 * r_pos / r_vel are the measurement VARIANCES (m^2, (m/s)^2).           */
arm_status Kalman3D_Update(Kalman3D_t *kf, float gps_pos, float gps_vel,
		float r_pos, float r_vel);

static inline float Kalman3D_GetPos(const Kalman3D_t *kf)  { return kf->core.x[0]; }
static inline float Kalman3D_GetVel(const Kalman3D_t *kf)  { return kf->core.x[1]; }
static inline float Kalman3D_GetBias(const Kalman3D_t *kf) { return kf->core.x[2]; }

/* ======================================================================
 *  4D Kalman filter  (altitude: IMU + barometer + rangefinder)
 *
 *  State : x = [altitude (cm), velocity (cm/s), accel_bias (cm/s^2),
 *               baro_bias (cm)]
 *  Input : u = vertical acceleration (cm/s^2, gravity removed)
 *  Meas. : baro       z = altitude + baro_bias   -> H = [1 0 0 1]
 *          rangefinder z = altitude              -> H = [1 0 0 0]
 * ====================================================================== */

typedef struct {
	KalmanCore_t core;
} Kalman4D_t;

arm_status Kalman4D_Init(Kalman4D_t *kf, float initial_alt_cm);
arm_status Kalman4D_Predict(Kalman4D_t *kf, float acc_z_cm_s2, float dt);

/* Generic scalar update with an arbitrary measurement row H (1x4). */
arm_status Kalman4D_UpdateMeasure(Kalman4D_t *kf, float measurement_cm,
		float sigma_cm, const float H[4]);

/* Sensor-specific wrappers (sigma is chosen adaptively, see .c file). */
arm_status Kalman4D_UpdateBaro(Kalman4D_t *kf, float baro_alt_cm);
arm_status Kalman4D_UpdateRange(Kalman4D_t *kf, float range_alt_cm,
		float acc_z_cm_s2);

float Kalman4D_GetSigmaBaro(float est_alt_cm);
float Kalman4D_GetSigmaRange(float range_alt_cm, float est_alt_cm,
		float acc_z_cm_s2);

static inline float Kalman4D_GetAltitude(const Kalman4D_t *kf)  { return kf->core.x[0]; }
static inline float Kalman4D_GetVelocity(const Kalman4D_t *kf)  { return kf->core.x[1]; }
static inline float Kalman4D_GetAccBias(const Kalman4D_t *kf)   { return kf->core.x[2]; }
static inline float Kalman4D_GetBaroBias(const Kalman4D_t *kf)  { return kf->core.x[3]; }

#ifdef __cplusplus
}
#endif

#endif /* INC_KALMAN_H_ */
