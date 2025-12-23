//
// Created by lbw on 25-6-13.
//
#include <armor_solver/kalman_pool/observer.hpp>
#include <utility>

#include "armor_solver/armor_solver_node.h"

Observer::Observer(const std::function<double()>& _get_dt)
{
    get_dt = _get_dt;
    dt = get_dt();
    a_yaw = 0;
    init();
}

void Observer::init()
{
    calcAy = [this]()
    {
        dt = get_dt();
        Ay << 1, dt,
            0, 1;
        return Ay;
    };
    Ay = calcAy();
    Hy << 1, 0;
    Py.setIdentity();
    Qy.setIdentity();
    Ry.setIdentity();

    calcAt = [this]()
    {
        dt = get_dt();
        At << 1, 0, 0, dt, 0, 0,
            0, 1, 0, 0, dt, 0,
            0, 0, 1, 0, 0, dt,
            0, 0, 0, 1, 0, 0,
            0, 0, 0, 0, 1, 0,
            0, 0, 0, 0, 0, 1;
        return At;
    };
    At = calcAt();
    Ht << 1, 0, 0, 0, 0, 0,
        0, 1, 0, 0, 0, 0,
        0, 0, 1, 0, 0, 0;
    Pt.setIdentity();
    Qt.setIdentity();
    Rt.setIdentity();

    predict_state.get_dt = [this]() { return dt = get_dt(); };
    Ps_post.setIdentity();
    Ps_pri.setIdentity();
}

void Observer::setNoise(const Noise& noise)
{
    calcQy = [this,noise]()
    {
        double yaw = noise.q_yaw;
        Qy << yaw * dt * dt / 2.0, 0,
            0, yaw * dt;
        return Qy;
    };
    calcRy = [this,noise](const YawZ1& z, double yaw_in_camera)
    {
        double a_yaw_ = std::abs(a_yaw);
        double coff = 1;
        if (yaw_track_mode == TrackMode::Shoot)
        {
            if (a_yaw_ > 0.01)
            {
                a_yaw_switch_count++;
                if (a_yaw_switch_count >= 5)
                {
                    yaw_track_mode = TrackMode::Fit;
                }
            }
            else
            {
                a_yaw_switch_count = std::max(0.0, a_yaw_switch_count - 0.1);
            }
            coff = 1.0;
        }
        else if (yaw_track_mode == TrackMode::Fit)
        {
            if (a_yaw_ < 0.01)
            {
                a_yaw_switch_count++;
                if (a_yaw_switch_count >= 5)
                {
                    yaw_track_mode = TrackMode::Shoot;
                }
            }
            else
            {
                a_yaw_switch_count = std::max(0.0, a_yaw_switch_count - 1);
            }
            coff = 0.01;
        }


        Ry << coff * noise.r_yaw;
        return Ry;
    };

    calcQt = [this,noise]()
    {
        double vx = noise.q_x * dt, vy = noise.q_y * dt, vz = noise.q_z * dt;
        double x = vx * dt / 2.0, y = vy * dt / 2.0, z = vz * dt / 2.0;
        // x y z vx vy vz
        Qt << x * x, 0, 0, x * vx, 0, 0, //x
            0, y * y, 0, 0, y * vy, 0, //y
            0, 0, z * z, 0, 0, z * vz, //z
            vx * x, 0, 0, vx * vx, 0, 0, //vx
            0, vy * y, 0, 0, vy * vy, 0, //vy
            0, 0, vz * z, 0, 0, vz * vz; //vz
        return Qt;
    };
    calcRt = [this,noise](const TranslateZ1& measure, double yaw_in_camera)
    {
        double x = noise.r_x, y = noise.r_y, z = noise.r_z;
        double v_yaw = x_post_(7);
        // double mu_v_yaw = std::abs(v_yaw) > 5.0 ? 100 : 1;
        double mu_yaw = yaw_in_camera * yaw_in_camera + 1;
        double mu_distance = x * x + y * y;
        double mu = mu_yaw * mu_distance;
        Rt << x * mu, 0, 0,
            0, y * mu, 0,
            0, 0, z * mu;
        return Rt;
    };

    calcQs = [this,noise]()
    {
        double vx = noise.qs_x * dt, vy = noise.qs_y * dt, vz = noise.qs_z * dt;
        double x = vx * dt / 2.0, y = vy * dt / 2.0, z = vz * dt / 2.0;
        double vyaw = noise.qs_yaw, yaw = vyaw * dt;
        double r = noise.qs_r, dz = noise.qs_dz;
        double ro = noise.q_ro;
        double x2 = x * x, y2 = y * y, z2 = z * z;
        double xvx = x * vx, yvy = y * vy, zvz = z * vz;
        double vx2 = vx * vx, vy2 = vy * vy, vz2 = vz * vz;
        double yaw2 = yaw * yaw, yawv = yaw * vyaw, vyaw2 = vyaw * vyaw;
        double xr = ro * x * r, yr = ro * y * r, zr = ro * z * r;
        double vxr = ro * vx * r, vyr = ro * vy * r, vzr = ro * vz * r;
        double dz2 = dz * dz, r2 = r * r;

        Qs <<
            //x y  z   vx  vy vz yaw w r dz
            x2, 0, 0, xvx, 0, 0, 0, 0, xr, 0, //x
            0, y2, 0, 0, yvy, 0, 0, 0, yr, 0, //y
            0, 0, z2, 0, 0, zvz, 0, 0, zr, 0, //z
            xvx, 0, 0, vx2, 0, 0, 0, 0, vxr, 0, //vx
            0, yvy, 0, 0, vy2, 0, 0, 0, vyr, 0, //vy
            0, 0, zvz, 0, 0, vz2, 0, 0, vzr, 0, //vz
            0, 0, 0, 0, 0, 0, yaw2, yawv, 0, 0, //yaw
            0, 0, 0, 0, 0, 0, yawv, vyaw2, 0, 0, //v_yaw
            xr, yr, zr, vxr, vyr, vzr, 0, 0, r2, 0, //r
            0, 0, 0, 0, 0, 0, 0, 0, 0, dz2; //dz
        return Qs;
    };
    calcRss = [this,noise](const StateZ1& measure)
    {
        double x = noise.r_x, y = noise.r_y, z = noise.r_z, yaw = noise.r_yaw;
        Rss <<
            x * measure(0), 0, 0, 0,
            0, y * measure(1), 0, 0,
            0, 0, z * measure(2), 0,
            0, 0, 0, yaw * measure(3);
        return Rss;
    };
    calcRsd = [this,noise](const StateM1& measure)
    {
        double x = noise.rd_x, y = noise.rd_y, z = noise.rd_z;
        double yaw = noise.r_yaw;
        double r = noise.rd_r, dz = noise.rd_dz;
        double ro = noise.r_ro;
        double xr = ro * x * r, yr = ro * y * r, zr = ro * z * r;
        Rsd <<
            x, 0, 0, 0, xr, 0,
            0, y, 0, 0, yr, 0,
            0, 0, z, 0, zr, 0,
            0, 0, 0, yaw, 0, 0,
            xr, yr, zr, 0, r, 0,
            0, 0, 0, 0, 0, dz;
        return Rsd;
    };
}

Observer::StateX1 Observer::predictYaw() noexcept
{
    Ay = calcAy();
    Qy = calcQy();
    x_pri_ = x_post_;
    YawX1 x = Yaw_pri;
    x = Ay * x;
    Py = Ay * Py * Ay.transpose() + Qy;
    StateX1 output = x_pri_;
    output.block(X_T, 0, X_Y, 1) = x;
    Yaw_pri = x;
    return output;
}

Observer::StateX1 Observer::updateYaw(const YawZ1& z, double yaw_in_camera) noexcept
{
    YawX1 x = Yaw_pri;
    double old_v_yaw = x(1);
    Ky = Py * Hy.transpose() * (Hy * Py * Hy.transpose() + Ry).inverse();
    x = x + Ky * (z - Hy * x);
    Py = (Eigen::Matrix<double, X_Y, X_Y>::Identity() - Ky * Hy) * Py;
    Ry = calcRy(z, yaw_in_camera);
    Yaw_pri = x;
    a_yaw = x(1) - old_v_yaw;
    x_post_ = x_pri_;
    return x_pri_;
}

Observer::StateX1 Observer::predictTranslate() noexcept
{
    At = calcAt();
    Qt = calcQt();
    x_pri_ = x_post_;
    TranslateX1 x = Translation_pri;
    x = At * x;
    Pt = At * Pt * At.transpose() + Qt;
    StateX1 output = x_pri_;
    output.block(0, 0, X_T, 1) = x;
    Translation_pri = x;
    return output;
}

Observer::StateX1 Observer::updateTranslate(const TranslateZ1& z, double yaw_in_camera) noexcept
{
    TranslateX1 x = Translation_pri;
    Kt = Pt * Ht.transpose() * (Ht * Pt * Ht.transpose() + Rt).inverse();
    x = x + Kt * (z - Ht * x);
    Pt = (Eigen::Matrix<double, X_T, X_T>::Identity() - Kt * Ht) * Pt;
    Rt = calcRt(z, yaw_in_camera);
    Translation_pri = x;
    x_post_ = x_pri_;
    global_node::Visualization->debug_user.debug15 = Pt.trace();
    return x_pri_;
}

Observer::StateX1 Observer::predictState() noexcept
{
    ceres::Jet<double, X_A> x_e_jet[X_A];
    for (int i = 0; i < X_A; ++i)
    {
        x_e_jet[i].a = x_post_[i];
        x_e_jet[i].v[i] = 1.;
        // a 对自己的偏导数为 1.
    }
    ceres::Jet<double, X_A> x_p_jet[X_A];
    predict_state(x_e_jet, x_p_jet);

    for (int i = 0; i < X_A; ++i)
    {
        x_pri_[i] = x_p_jet[i].a;
        Fs.block(i, 0, 1, X_A) = x_p_jet[i].v.transpose();
    }

    Qs = calcQs();
    Ps_pri = Fs * Ps_post * Fs.transpose() + Qs;
    x_post_ = x_pri_;

    return x_pri_;
}

Observer::StateX1 Observer::updateStateSingle(const StateZ1& z) noexcept
{
    ceres::Jet<double, X_A> x_p_jet[X_A];
    for (int i = 0; i < X_A; i++)
    {
        x_p_jet[i].a = x_pri_[i];
        x_p_jet[i].v[i] = 1;
    }
    ceres::Jet<double, X_A> z_p_jet[Z_A];
    measure_state_single(x_p_jet, z_p_jet);

    StateZ1 z_pri;
    for (int i = 0; i < Z_A; i++)
    {
        z_pri[i] = z_p_jet[i].a;
        Hss.block(i, 0, 1, X_A) = z_p_jet[i].v.transpose();
    }

    Rss = calcRss(z);
    Kss = Ps_pri * Hss.transpose() * (Hss * Ps_pri * Hss.transpose() + Rss).inverse();
    x_post_ = x_post_ + Kss * (z - z_pri);
    Ps_post = (StateXX::Identity() - Kss * Hss) * Ps_pri;
    return x_post_;
}

Observer::StateX1 Observer::updateStateDouble(const StateM1& z) noexcept
{
    ceres::Jet<double, X_M> x_p_jet[X_M];
    for (int i = 0; i < X_M; i++)
    {
        x_p_jet[i].a = x_pri_[i];
        x_p_jet[i].v[i] = 1;
    }
    ceres::Jet<double, X_M> z_p_jet[Z_M];
    measure_state_double(x_p_jet, z_p_jet);

    StateM1 z_pri;
    for (int i = 0; i < Z_M; i++)
    {
        z_pri[i] = z_p_jet[i].a;
        Hsd.block(i, 0, 1, X_M) = z_p_jet[i].v.transpose();
    }

    Rsd = calcRsd(z);
    Ksd = Ps_pri * Hsd.transpose() * (Hsd * Ps_pri * Hsd.transpose() + Rsd).inverse();
    x_post_ = x_post_ + Ksd * (z - z_pri);
    Ps_post = (StateXX::Identity() - Ksd * Hsd) * Ps_pri;
    return x_post_;
}

