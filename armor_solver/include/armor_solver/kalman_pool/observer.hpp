#ifndef OBSERVER
#define OBSERVER

// std
#include <functional>
// Eigen
#include <Eigen/Dense>
// ceres
#include <ceres/jet.h>

namespace ObserverFunction
{
  template <int X_N, int Z_N>
  struct PredictState
  {
    template <class T>
    void operator()(const T x0[X_N], T x1[X_N])
    {
      dt = get_dt();
      for (int i = 0; i < X_N; i++)
      {
        x1[i] = x0[i];
      }

      // v_xyz
      // linear velocity
      x1[0] += x0[3] * dt + 0.5 * x0[6] * dt * dt;
      x1[1] += x0[4] * dt + 0.5 * x0[7] * dt * dt;
      x1[2] += x0[5] * dt + 0.5 * x0[8] * dt * dt;

            //a_xyz
      // acceleration
      x1[3] += x0[6] * dt;
      x1[4] += x0[7] * dt;
      x1[5] += x0[8] * dt;

      // v_yaw
      // angular velocity
      x1[6] += x0[7] * dt;
    }

    double dt = 0;
    std::function<double()> get_dt;
  };

  template <int X_N, int Z_N>
  struct MeasureStateSingle
  {
    template <class T>
    void operator()(const T x[X_N], T z[Z_N])
    {
      z[0] = x[0] - ceres::cos(x[9]) * x[12];
      z[1] = x[1] - ceres::sin(x[9]) * x[12];
      z[2] = x[2] + x[12];
      z[3] = x[9];
  };

  template <int X_N, int Z_N>
  struct MeasureStateDouble
  {
    template <class T>
    void operator()(const T x[X_N], T z[Z_N])
    {
      z[0] = x[0];
      z[1] = x[1];
      z[2] = x[2];
      z[3] = x[9]; //yaw
      z[4] = x[11]; //r
      z[5] = x[12]; //dz
    }
  };
}

class Observer
{
public:
  struct Noise
  {
    double q_x, q_y, q_z;
    double r_x, r_y, r_z;
    double q_yaw;
    double r_yaw;
    double qs_x, qs_y, qs_z, qs_yaw, qs_r, qs_dz;
    double q_ro, r_ro;
    double rd_x, rd_y, rd_z, rd_r, rd_dz;
  };
  
  static constexpr int X_A = 13; //xc, yc, zc, vxc, vyc, vzc, axc, ayc, azc, yaw, v_yaw, r, d_z;
  static constexpr int Z_A = 4; //xa, ya, za, yaw

  static constexpr int X_M = 13; //xc, yc, zc, vxc, vyc, vzc, axc, ayc, azc, yaw, v_yaw, r, d_z;
  static constexpr int Z_M = 6; //xc, yc, zc, yaw, r, dz

  static constexpr int X_R = 3; //yaw, v_yaw, r
  static constexpr int Z_R = 1; //yaw

  static constexpr int X_Y = 2; //yaw, v_yaw
  static constexpr int Z_Y = 1; //yaw

  static constexpr int X_T = 9; //xc, yc, zc, vxc, vyc, vzc, axc, ayc, azc
  static constexpr int Z_T = 3; //xc, yc, zc


  using StateXX = Eigen::Matrix<double, X_A, X_A>;
  using StateX1 = Eigen::Matrix<double, X_A, 1>;
  using StateXZ = Eigen::Matrix<double, X_A, Z_A>;
  using StateZX = Eigen::Matrix<double, Z_A, X_A>;
  using StateZZ = Eigen::Matrix<double, Z_A, Z_A>;
  using StateZ1 = Eigen::Matrix<double, Z_A, 1>;
  using StateMX = Eigen::Matrix<double, Z_M, X_A>;
  using StateXM = Eigen::Matrix<double, X_A, Z_M>;
  using StateMM = Eigen::Matrix<double, Z_M, Z_M>;
  using StateM1 = Eigen::Matrix<double, Z_M, 1>;
  using PredictState = ObserverFunction::PredictState<X_A, X_A>;
  using MeasureStateSingle = ObserverFunction::MeasureStateSingle<X_A, Z_A>;
  using MeasureStateDouble = ObserverFunction::MeasureStateDouble<X_M, Z_M>;

  using YawXX = Eigen::Matrix<double, X_Y, X_Y>;
  using YawXZ = Eigen::Matrix<double, X_Y, Z_Y>;
  using YawZX = Eigen::Matrix<double, Z_Y, X_Y>;
  using YawX1 = Eigen::Matrix<double, X_Y, 1>;
  using YawZZ = Eigen::Matrix<double, Z_Y, Z_Y>;
  using YawZ1 = Eigen::Matrix<double, Z_Y, 1>;

  using TranslateXX = Eigen::Matrix<double, X_T, X_T>;
  using TranslateX1 = Eigen::Matrix<double, X_T, 1>;
  using TranslateXZ = Eigen::Matrix<double, X_T, Z_T>;
  using TranslateZX = Eigen::Matrix<double, Z_T, X_T>;
  using TranslateZZ = Eigen::Matrix<double, Z_T, Z_T>;
  using TranslateZ1 = Eigen::Matrix<double, Z_T, 1>;

#define Yaw_pri x_pri_.block(X_T, 0, X_Y, 1)
#define Yaw_post x_post_.block(X_T, 0, X_Y, 1)
#define Translation_pri x_pri_.block(0, 0, X_T, 1)
#define Translation_post x_post_.block(0, 0, X_T, 1)

  explicit Observer(const std::function<double()>& _get_dt);

  void init();
  // Set the initial state
  void setState(const StateX1& x0) noexcept { x_pri_ = x_post_ = x0; }
  void setNoise(const Noise& noise);

  //yaw的预测
  StateX1 predictYaw() noexcept;
  StateX1 updateYaw(const YawZ1& z) noexcept;

  //平移的预测
  StateX1 predictTranslate() noexcept;
  StateX1 updateTranslate(const TranslateZ1& z,double yaw_in_camera) noexcept;

  //全部状态量的预测
  StateX1 predictState() noexcept;
  StateX1 predictTranslate(double dt_) noexcept;
  StateX1 updateStateSingle(const StateZ1& z) noexcept;
  StateX1 updateStateDouble(const StateM1& z) noexcept;

  enum class TrackMode { Fit,Shoot };

private:
  double dt{};

  //旋转
  std::function<YawXX()> calcAy;
  YawXX Ay;
  YawZX Hy;
  YawXZ Ky;
  YawXX Py;
  YawXX Qy;
  YawZZ Ry;

  //平移
  std::function<TranslateXX()> calcAt;
  TranslateXX At;
  TranslateZX Ht;
  TranslateXZ Kt;
  TranslateXX Pt;
  TranslateXX Qt;
  TranslateZZ Rt;

  StateXX Att;
  std::function<StateXX(double dt_)> calcAtt;
  //全部状态
  PredictState predict_state;
  MeasureStateSingle measure_state_single;
  MeasureStateDouble measure_state_double;
  StateXX Fs;
  StateZX Hss; //单观测
  StateMX Hsd; //双观测
  StateXX Qs;
  StateZZ Rss;
  StateMM Rsd;
  StateXZ Kss;
  StateXM Ksd;
  StateXX Ps_pri;
  StateXX Ps_post;

  // 状态
  StateX1 x_pri_;
  StateX1 x_post_;

  //噪声计算
  //yaw
  std::function<YawZZ(const YawZ1& z)> calcRy;
  std::function<YawXX()> calcQy;
  double a_yaw{};
  double a_yaw_switch_count = 0;
  TrackMode yaw_track_mode = TrackMode::Fit;

  //平移
  std::function<TranslateZZ(const TranslateZ1& z, double)> calcRt;
  std::function<TranslateXX()> calcQt;

  //全状态
  std::function<StateZZ(const StateZ1& z)> calcRss;
  std::function<StateMM(const StateM1& z)> calcRsd;
  std::function<StateXX()> calcQs;

  std::function<double()> get_dt;
};


#endif  // OBSERVER
