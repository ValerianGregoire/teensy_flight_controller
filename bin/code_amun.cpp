#include <iostream>
#include <Eigen/Dense>
#include <algorithm>
#include <cmath>
#include <chrono>

/* ================= CONSTANTS ================= */

constexpr float g = 9.81f;

constexpr float dt_pos = 0.02f;   // 50 Hz
constexpr float dt_vel = 0.02f;   // 50 Hz
constexpr float dt_ang = 0.004f;  // 250 Hz
constexpr float dt_rate = 0.001f; // 1000 Hz

constexpr float acc_limit = 5.0f;
constexpr float angle_limit = 0.5f;
constexpr float rate_limit = 0.75f;
constexpr float M_limit = 0.17f;
constexpr float T_hover = 11.0f;

/* ================= PID ================= */

struct PID
{
  float Kp, Ki, Kd;
  float integral = 0.0f;
  float prev_error = 0.0f;
  float Imax;

  PID(float kp, float ki, float kd, float imax)
      : Kp(kp), Ki(ki), Kd(kd), Imax(imax) {}

  float update(float error, float dt, bool saturated)
  {
    if (!saturated)
    {
      integral += error * dt;
      integral = std::clamp(integral, -Imax, Imax);
    }

    float derivative = (error - prev_error) / dt;
    prev_error = error;

    return Kp * error + Ki * integral + Kd * derivative;
  }
};

/* ================= MAIN ================= */

int main()
{

  /* ===== Control Allocation ===== */

  Eigen::Matrix4f M;
  M << 1, 1, -1, 1,
      1, -1, -1, -1,
      1, -1, 1, 1,
      1, 1, 1, -1;

  Eigen::Matrix4f M_pinv =
      (M.transpose() * M).inverse() * M.transpose();

  Eigen::Vector4f Thrust;

  /* ===== States (normally from EKF) ===== */

  float x = 0, y = 0, z = 0;
  float vx = 0, vy = 0, vz = 0;
  float roll = 0, pitch = 0, yaw = 0;
  float p = 0, q = 0, r = 0;

  /* ===== Setpoints ===== */

  float x_d = 0, y_d = 0, z_d = 0;
  float yaw_d = 0.0f;

  /* ===== PID Controllers ===== */

  PID pid_x(1.0f, 0.0f, 0.0f, 2.0f);
  PID pid_y(1.0f, 0.0f, 0.0f, 2.0f);
  PID pid_z(1.0f, 0.0f, 0.0f, 2.0f);

  PID pid_vx(1.0f, 0.0f, 0.0f, 3.0f);
  PID pid_vy(1.0f, 0.0f, 0.0f, 3.0f);
  PID pid_vz(1.0f, 0.0f, 0.0f, 3.0f);

  PID pid_roll(6.0f, 0.0f, 0.1f, 0.5f);
  PID pid_pitch(6.0f, 0.0f, 0.1f, 0.5f);
  PID pid_yaw(2.0f, 0.0f, 0.0f, 0.3f);

  PID pid_p(0.2f, 0.02f, 0.001f, 0.2f);
  PID pid_q(0.2f, 0.02f, 0.001f, 0.2f);
  PID pid_r(0.1f, 0.01f, 0.000f, 0.1f);

  /* ===== Timing ===== */

  using clock = std::chrono::steady_clock;
  auto last_time = clock::now();

  float t_pos = 0, t_vel = 0, t_ang = 0;

  float vx_d = 0, vy_d = 0, vz_d = 0;
  float ax_d = 0, ay_d = 0, az_d = 0;
  float roll_d = 0, pitch_d = 0;
  float p_d = 0, q_d = 0, r_d = 0;

  /* ===== Main Loop ===== */

  while (true)
  {

    auto now = clock::now();
    std::chrono::duration<float> dt = now - last_time;
    last_time = now;

    float dT = dt.count();
    if (dT <= 0.0f)
      continue;

    t_pos += dT;
    t_vel += dT;
    t_ang += dT;

    /* ===== Position Loop ===== */
    if (t_pos >= dt_pos)
    {
      vx_d = pid_x.update(x_d - x, dt_pos, false);
      vy_d = pid_y.update(y_d - y, dt_pos, false);
      vz_d = pid_z.update(z_d - z, dt_pos, false);
      t_pos = 0.0f;
    }

    /* ===== Velocity Loop ===== */
    if (t_vel >= dt_vel)
    {
      ax_d = pid_vx.update(vx_d - vx, dt_vel, false);
      ay_d = pid_vy.update(vy_d - vy, dt_vel, false);
      az_d = pid_vz.update(vz_d - vz, dt_vel, false);

      ax_d = std::clamp(ax_d, -acc_limit, acc_limit);
      ay_d = std::clamp(ay_d, -acc_limit, acc_limit);
      az_d = std::clamp(az_d, -acc_limit, acc_limit);

      t_vel = 0.0f;
    }

    /* ===== Attitude Setpoint ===== */

    float T = std::sqrt(ax_d * ax_d + ay_d * ay_d + (g + az_d) * (g + az_d));

    roll_d = std::clamp(std::asin(-ay_d / T), -angle_limit, angle_limit);
    pitch_d = std::clamp(std::atan2(ax_d, g + az_d), -angle_limit, angle_limit);

    /* ===== Angle Loop ===== */
    if (t_ang >= dt_ang)
    {
      p_d = pid_roll.update(roll_d - roll, dt_ang, false);
      q_d = pid_pitch.update(pitch_d - pitch, dt_ang, false);
      r_d = pid_yaw.update(yaw_d - yaw, dt_ang, false);

      p_d = std::clamp(p_d, -rate_limit, rate_limit);
      q_d = std::clamp(q_d, -rate_limit, rate_limit);
      r_d = std::clamp(r_d, -rate_limit, rate_limit);

      t_ang = 0.0f;
    }

    /* ===== Rate Loop ===== */

    float Mx_cmd = pid_p.update(p_d - p, dt_rate, false);
    float My_cmd = pid_q.update(q_d - q, dt_rate, false);
    float Mz_cmd = pid_r.update(r_d - r, dt_rate, false);

    bool sat_x = std::abs(Mx_cmd) >= M_limit;
    bool sat_y = std::abs(My_cmd) >= M_limit;
    bool sat_z = std::abs(Mz_cmd) >= M_limit;

    float Mx = std::clamp(Mx_cmd, -M_limit, M_limit);
    float My = std::clamp(My_cmd, -M_limit, M_limit);
    float Mz = std::clamp(Mz_cmd, -M_limit, M_limit);

    pid_p.update(p_d - p, dt_rate, sat_x);
    pid_q.update(q_d - q, dt_rate, sat_y);
    pid_r.update(r_d - r, dt_rate, sat_z);

    /* ===== Normalisation ===== */

    float Tn = T / T_hover;
    float Mxn = Mx / M_limit;
    float Myn = My / M_limit;
    float Mzn = Mz / M_limit;

    /* ===== Allocation ===== */

    Eigen::Vector4f cmd(Tn, Mxn, Myn, Mzn);
    Thrust = M_pinv * cmd;

    for (int i = 0; i < 4; i++)
      Thrust(i) = std::clamp(Thrust(i), 0.0f, 1.0f);
  }
}
