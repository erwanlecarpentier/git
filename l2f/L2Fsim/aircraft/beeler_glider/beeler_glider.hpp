#ifndef L2FSIM_BEELER_GLIDER_HPP_
#define L2FSIM_BEELER_GLIDER_HPP_

#include <L2Fsim/aircraft/aircraft.hpp>
#include <L2Fsim/aircraft/beeler_glider/beeler_glider_state.hpp>
#include <L2Fsim/aircraft/beeler_glider/beeler_glider_command.hpp>
#include <L2Fsim/utils/quaternion.hpp>
#include <L2Fsim/utils/utils.hpp>
#include <vector>
#include <cmath>

/**
 * @file beeler_glider.hpp
 * @version 1.1
 * @since 1.0
 *
 * Beeler's glider model
 * Equations derived from:
 * Beeler, Moerder and Cox. A Flight Dynamics Model for a Small Glider
 * in Ambient Winds. NASA/TM-2003-212665. 2003.
 * Model validity:
 * - wingspan in [1.52m; 3.55m]
 * - aspect ratio in [6; 16]
 * - weight in [0.22kg; 5.45kg]
 * Notations:
 * (x,y,z) glider position in an Earth-based coordinate system
 * V ground speed of the glider
 * khi azimuth angle (angle between the north and the projection on xy-plan of the velocity vector)
 * gamma elevation angle (angle between the velocity vector and its projection on the xy-plan)
 * sigma bank angle (rotation allong the velocity vector)
 * alpha angle of attack
 * beta sideslip angle
 *
 * @note the earth frame corresponds to the directions of the north, east and downward for the x, y and z axis. However the z notation used in the code corresponds to the altitude i.e. '-z'
 * @note khi, gamma and sigma form an euler sequence leading to the velocity frame
 */

namespace L2Fsim {

class beeler_glider : public aircraft {
public:
    /**
     * Attributes
     * @param {beeler_glider_state} s; the state of the aircraft
     * @param {beeler_glider_command} u; the command of the aircraft
     * @param {double} mass; mass
     * @param {double} wingspan; wing span
     * @param {double} aspect_ratio; aspect ratio
     * @param {double} ARv; aspect ratio of vertical tail
     * @param {double} Sf; fuselage area
     * @param {double} lt; fuselage moment arm length
     * @param {double} Vh; horizontal tail volume ratio
     * @param {double} Vv; vertical tail volume ratio
     * @param {double} c_; mean aerodynamic cord
     * @param {double} S; wing surface area
     * @param {double} St; horizontal tail surface
     * @param {double} Sv; vertical tail surface
     * @param {double} e; Oswald efficiency number
     * @param {double} Re; Reynolds number
     * @param {double} a0; lift curve slope
     * @param {double} alpha0; zero point
     * @param {double} Cd0, Cdl; minimum wing profile drag
     * @param {double} Clmin, Cl_alpha, Cl_beta; minimum lift
     */
    beeler_glider_state s;
    beeler_glider_command u;
    double mass;
    double wingspan;
    double aspect_ratio;
    double ARv = 0.5*aspect_ratio;
    double Sf = 86. * (2.54/100.) * (2.54/100.);
    double lt = 0.28*(2.54/100.);
    double Vh = 0.4;
    double Vv = 0.02;
    double c_ = 1.03 * wingspan / aspect_ratio;
    double S = wingspan * wingspan / aspect_ratio;
    double St = Vh * c_ * S / lt;
    double Sv = Vv * wingspan * S / lt;
    double e = 0.95;
    //double Re = 150000.; //not used?
    double a0 = 0.1*(180./3.14);
    double alpha0 = -2.5*(3.14/180.);
    double Cd0 = 0.01;
    double Cdl = 0.05;
    double Clmin = 0.4;
    double Cl_alpha = a0/(1.+a0/(3.14*e*aspect_ratio));
    double Cc_beta = a0/(1.+a0*(3.14*ARv)) * (Sv / S);

    /**
     * Constructor
     * @param {state} _s: initial state
     * @param {command} _u: initial command
     * @param {double} m; mass
     * @param {double} ws; wing span
     * @param {double} ar; aspect ratio
     */
    beeler_glider(
        beeler_glider_state _s,
        beeler_glider_command _u,
        double m=1.35,
        double ws=2.,
        double ar=15.) :
        s(_s),
        u(_u),
        mass(m),
        wingspan(ws),
        aspect_ratio(ar)
    {}

    /** @brief Get a reference on the state */
    state & get_state() override {return s;}

	/**
	 * @brief Set the state of the aircraft
	 * @param {const beeler_glider_state &} _s; input state
	 */
	void set_state(const beeler_glider_state &_s) {s = _s;}

    /** @brief Get a reference on the command */
    command & get_command() override {return u;}

	/**
	 * @brief Set the command of the aircraft
	 * @param {const beeler_glider_command &} _u; input command
	 */
	void set_command(const beeler_glider_command &_u) {u = _u;}

    double get_distance_to_center() override {
        return sqrt(s.x * s.x + s.y * s.y);
    }

    /** @brief Apply the command i.e. modify the state attribute of the aircraft accordingly to the command */
    aircraft & apply_command() override {
        s.alpha += u.dalpha;
        s.beta += u.dbeta;
        s.sigma += u.dsigma;
        return *this;
    }

    /**
     * @brief Compute the time derivative of the input state
     * @param {const flight_zone &} fz; flight zone
     * @param {const double} t; current time
     * @param {state &} _s; updated state
     * @warning dynamic cast from state to beeler_glider_state
     */
    aircraft & update_state_dynamic(flight_zone &fz, const double t, state &_s) override {
        beeler_glider_state &s = dynamic_cast <beeler_glider_state &> (_s);
        double lift=0., drag=0., sideforce=0.;
        double V = s.V;
        double gamma = s.gamma;
        double khi = s.khi;
        double sigma = s.sigma;

		double cosgamma = cos(gamma);
		double singamma = sin(gamma);
		double cossigma = cos(sigma);
		double sinsigma = sin(sigma);

        calc_aero_forces(fz, t, lift, drag, sideforce);
        s.xdot = V * cosgamma * cos(khi);
        s.ydot = V * cosgamma * sin(khi);
        s.zdot = V * singamma;
        s.Vdot = - drag / mass - 9.81 * singamma;
        s.gammadot = (lift * cossigma + sideforce * sinsigma) / (mass * V) - 9.81 * cosgamma / V;
        s.khidot = (lift * sinsigma - sideforce * cossigma) / (mass * V * cosgamma);
        return *this;
    }

    /**
     * @brief Check if the state vector contains values that are out of the model's range of validity
     */
    aircraft & is_in_model() override {
        double z = s.z;
        double gamma = s.gamma;
        double alpha = s.alpha;
        double limit_gamma_angle = 45. * 3.14159 / 180.;
        double limit_alpha_gamma_angle = 45. * 3.14159 / 180.;
        if(z < 0) {
            std::cout << "STOP: altitude 'z' < 0" << std::endl;
            exit(-1);
        }
        if(gamma > limit_gamma_angle) {
            std::cout << "STOP: elevation angle 'gamma' > " << limit_gamma_angle << " rad" << std::endl;
            exit(-1);
        }
        if(gamma < -limit_gamma_angle) {
            std::cout << "STOP: elevation angle 'gamma' < " << -limit_gamma_angle << " rad" << std::endl;
            exit(-1);
        }
        if(gamma + alpha > limit_alpha_gamma_angle) {
            std::cout << "STOP: inclination angle 'gamma+alpha' > " << limit_alpha_gamma_angle << " rad" << std::endl;
            exit(-1);
        }
        if(gamma + alpha < -limit_alpha_gamma_angle) {
            std::cout << "STOP: inclination angle 'gamma+alpha' < " << -limit_alpha_gamma_angle << " rad" << std::endl;
            exit(-1);
        }
        return *this;
    }

protected:

    /**
     * @brief Compute lift, drag and sideforce
     * @param {flight_zone &} fz; flight zone
     * @param {const double} t; current time
     * @param {double &} lift, drag, sideforce; aerodynamic forces
     */
    void calc_aero_forces(flight_zone &fz,
                        const double t,
                        double &lift,
                        double &drag,
                        double &sideforce) { //const {

        /** Retrieve aircraft's state */
        double x = s.x;
        double y = s.y;
        double z = s.z;
        double V = s.V;
        double gamma = s.gamma;
        double khi = s.khi;
        double alpha = s.alpha;
        double beta = s.beta;
        double sigma = s.sigma;

        /** Relative wind */
        std::vector<double> w(3);

        /** Wind relative velocity */
        std::vector<double> V_w(3);
        std::vector<double> X_w(3);

        /** Wind relative angles */
        double alpha_w, beta_w, gamma_w, khi_w, sigma_w;

        /** Aerodynamic force coefficients with wind */
        double Cc_w, Cl_w, Cd_w;

        /** Dynamic pressure */
        double q;

        /** Rotation matrices and quaternions */
        quaternion rviq; // quaternion version of the Euler rotation sequence R_VI

        std::vector<double> rbv(9); // rotation from velocity frame to body frame R_BV
        std::vector<double> rbv1(9); // rotation of alpha
        std::vector<double> rbv2(9); // rotation of -beta
        quaternion rbvq; //quaternion version R_BV
        quaternion rbv1q;
        quaternion rbv2q;

        std::vector<double> m(9); // M matrix, used to retrieve wind relative angles
        std::vector<double> m11(9);
        std::vector<double> m12(9);
        quaternion mq; // quaternion version of M
        quaternion m11q;
        quaternion m12q;

        /** Calc relative wind */
        fz.wind(x, y, z, t, w);

        V_w.at(0) = V * cos(gamma) * cos(khi) - w.at(0);
        V_w.at(1) = V * cos(gamma) * sin(khi) - w.at(1);
        V_w.at(2) = V * sin(gamma) - w.at(2);

        X_w.at(0) = V_w.at(0) / sqrt(V_w.at(0)*V_w.at(0) + V_w.at(1)*V_w.at(1) + V_w.at(2)*V_w.at(2));
        X_w.at(1) = V_w.at(1) / sqrt(V_w.at(0)*V_w.at(0) + V_w.at(1)*V_w.at(1) + V_w.at(2)*V_w.at(2));
        X_w.at(2) = V_w.at(2) / sqrt(V_w.at(0)*V_w.at(0) + V_w.at(1)*V_w.at(1) + V_w.at(2)*V_w.at(2));

        /** Calc of gamma_w and khi_w */
        gamma_w = asin(X_w.at(2));

        if(X_w.at(0) / cos(gamma_w) > 1) {
            khi_w = 0;
        }else if(X_w.at(0) / cos(gamma_w) < -1){
            khi_w = M_1_PI;
        }else{
            khi_w = sgn(X_w.at(1) / cos(gamma_w)) * acos(X_w.at(0) / cos(gamma_w));
        }

        /** Calc of alpha_w, beta_w and sigma_w :
         Use of rotation matrices and quaternions */

        rviq.fromEuler(khi, gamma, sigma);

        rbv1.at(0) = cos(alpha);
        rbv1.at(2) = sin(alpha);
        rbv1.at(4) = 1;
        rbv1.at(6) = -sin(alpha);
        rbv1.at(8) = cos(alpha);
        rbv1.at(1) = rbv1.at(3) = rbv1.at(5) = rbv1.at(7) = 0;

        rbv2.at(0) = cos(beta);
        rbv2.at(1) = sin(beta);
        rbv2.at(3) = -sin(beta);
        rbv2.at(4) = cos(beta);
        rbv2.at(8) = 1;
        rbv2.at(2) = rbv2.at(5) = rbv2.at(6) = rbv2.at(7) = 0;

        rbv1q.fromRotationMatrix(rbv1);
        rbv2q.fromRotationMatrix(rbv2);
        rbvq = rbv1q;
        rbvq.multRight(rbv2q);

        // On doit commencer par les lignes, il me semble
        m11.at(0) = cos(gamma_w);
        m11.at(2) = -sin(gamma_w);
        m11.at(4) = 1;
        m11.at(6) = sin(gamma_w);
        m11.at(8) = cos(gamma_w);
        m11.at(1) = m11.at(3) = m11.at(5) = m11.at(7) = 0;

        m12.at(0) = cos(khi_w);
        m12.at(1) = sin(khi_w);
        m12.at(3) = -sin(khi_w);
        m12.at(4) = cos(khi_w);
        m12.at(8) = 1;
        m12.at(2) = m12.at(5) = m12.at(6) = m12.at(7) = 0;

        m11q.fromRotationMatrix(m11);
        m12q.fromRotationMatrix(m12);
        mq = m11q;
        mq.multRight(m12q);
        mq.multRight(rviq);
        mq.multRight(rbvq);
        mq.toRotationMatrix(m);

        alpha_w = asin(m.at(2));

        if(m.at(8) / cos(alpha_w) > 1) {
            sigma_w = 0;
        }else if(m.at(8) / cos(alpha_w) < -1){
            sigma_w = M_1_PI;
        }else{
            sigma_w = sgn(-m.at(5) / cos(alpha_w)) * acos(m.at(8) / cos(alpha_w));
        }

        if(m.at(0) / cos(alpha_w) > 1) {
            beta_w = 0;
        }else if(m.at(0) / cos(alpha_w) < -1){
            beta_w = M_1_PI;
        }else{
            beta_w = sgn(m.at(1) / cos(alpha_w)) * acos(m.at(0) / cos(alpha_w));
        }

        /** Calc of the aerodynamic force coefficients with wind */
        Cc_w = Cc_beta * beta_w;
        Cl_w = Cl_alpha * (alpha_w - alpha0);
        Cd_w = Cd0 + Cdl * (Cl_w - Clmin) * (Cl_w - Clmin) + Cl_w * Cl_w / (3.14*e*aspect_ratio) + Cc_w * Cc_w / (3.14*e*aspect_ratio) * (S / Sv);

        /** Calc of the dynamic pressure */
        q = 0.5 * 1.225 * (V_w.at(0)*V_w.at(0) + V_w.at(1)*V_w.at(1) + V_w.at(2)*V_w.at(2));

        /** Calc of the aerodynamic forces */
        drag = q * S * Cd_w;
        sideforce = q * S * Cc_w;
        lift = q * S * Cl_w;
    }
};

}

#endif
