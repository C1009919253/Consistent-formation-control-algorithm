// No used now
#pragma once
#include <ct/core/systems/continuous_time/ControlledSystem.h>

#include <cmath>
#include <memory>
#include <iostream>

using namespace ct::core;

namespace TAC
{
namespace CAR
{
template <typename SCALAR>
class sim_car_system : public ControlledSystem<3, 2, SCALAR>
{
public:
    const size_t STATE_DIM = 3;    //!< state dimension (position, velocity)
    const size_t CONTROL_DIM = 2;  //!< control dimension (force)

    typedef ControlledSystem<3, 2, SCALAR> Base;
    typedef typename Base::time_t time_t;

    //! default constructor
    sim_car_system() = delete;

    //! constructor directly using frequency and damping coefficients
    /*!
     *
     * @param w_n eigenfrequency
     * @param zeta damping ratio
     * @param g_dc DC gain on input, set to \f$ w_n^2 \f$ to achieve amplification 1
     * @param controller controller (optional)
     */
    sim_car_system(SCALAR w_n,
        SCALAR zeta = SCALAR(1.0),
        SCALAR g_dc = SCALAR(1.0),
        std::shared_ptr<Controller<3, 2, SCALAR>> controller = nullptr)
        : ControlledSystem<3, 2, SCALAR>(controller, SYSTEM_TYPE::SECOND_ORDER),
          w_n_(w_n),
          w_n_square_(w_n_ * w_n_),
          zeta_(zeta),
          g_dc_(g_dc)
    {
    }

    //! copy constructor
    sim_car_system(const sim_car_system& arg)
        : ControlledSystem<3, 2, SCALAR>(arg),
          w_n_(arg.w_n_),
          w_n_square_(arg.w_n_square_),
          zeta_(arg.zeta_),
          g_dc_(arg.g_dc_)
    {
    }

    //! constructor using a more mechanical definition (spring-mass-damping)
    /*!
     * @param k spring stiffness
     * @param m mass
     * @param d damper constant
     * @param g_dc DC input gain
     * @param controller controller (optional)
     */
    sim_car_system(SCALAR k, SCALAR m, SCALAR d, SCALAR g_dc, std::shared_ptr<Controller<3, 2>> controller = nullptr)
        : ControlledSystem<3, 2>(controller),
          w_n_(std::sqrt(k / m)),
          w_n_square_(w_n_ * w_n_),
          zeta_(d / (2.0 * m * k)),
          g_dc_(g_dc)
    {
    }

    //! deep copy
    sim_car_system* clone() const override { return new sim_car_system(*this); }
    //! destructor
    virtual ~sim_car_system() {}
    //! set the dynamics
    /*!
     * @param w_n eigenfrequency
     * @param zeta damping ratio
     * @param g_dc DC gain
     */
    void setDynamics(SCALAR w_n, SCALAR zeta = SCALAR(1.0), SCALAR g_dc = SCALAR(1.0))
    {
        w_n_ = w_n;
        w_n_square_ = w_n_ * w_n_;
        zeta_ = zeta;
        g_dc_ = g_dc;
    }

    //! evaluate the system dynamics
    /*!
     * @param state current state (position, velocity)
     * @param t current time (gets ignored)
     * @param control control action
     * @param derivative derivative (velocity, acceleration)
     */
    virtual void computeControlledDynamics(const StateVector<3, SCALAR>& state,
        const time_t& t,
        const ControlVector<2, SCALAR>& control,
        StateVector<3, SCALAR>& derivative) override
    {
        /*derivative(0) = ct::core::tpl::TraitSelector<SCALAR>::Trait::cos(state(2)) * control(0);
        derivative(1) = ct::core::tpl::TraitSelector<SCALAR>::Trait::sin(state(2)) * control(0);
        derivative(2) = control(1);*/
        derivative(0) = control(0);
        /*derivative(0) = state(0) - 0.05 * ct::core::tpl::TraitSelector<SCALAR>::Trait::sin(state(2)) * state(2) + 0.05 * ct::core::tpl::TraitSelector<SCALAR>::Trait::cos(state(2)) * control(0);
        derivative(1) = state(1) + 0.05 * ct::core::tpl::TraitSelector<SCALAR>::Trait::cos(state(2)) * state(2) + 0.05 * ct::core::tpl::TraitSelector<SCALAR>::Trait::sin(state(2)) * control(0);
        derivative(2) = state(2) + 0.05 * control(1);*/
    }

    //! check the parameters
    /*!
     * @return true if parameters are physical
     */
    bool checkParameters()
    {
        if (zeta_ < 0)
        {
            std::cout << "Warning: Damping is negative!" << std::endl;
            return false;
        }
        if (w_n_ < 0)
        {
            std::cout << "Warning: Frequency w_n is negative!" << std::endl;
            return false;
        }
        if (g_dc_ < 0)
        {
            std::cout << "Warning: Steady state gain is negative!" << std::endl;
            return false;
        }
        if (g_dc_ == 0)
        {
            std::cout << "Warning: Steady state gain is zero!" << std::endl;
            return false;
        }

        return true;
    }

    //! print out infos about the system
    void printSystemInfo()
    {
        std::cout << "Frequency: " << w_n_ << std::endl;
        std::cout << "Zeta: " << zeta_ << std::endl;
        std::cout << "DC gain: " << g_dc_ << std::endl;

        std::cout << "System is ";
        if (zeta_ == 0.0)
        {
            std::cout << "undamped" << std::endl;
        }
        if (zeta_ == 1.0)
        {
            std::cout << "critically damped" << std::endl;
        }
        if (zeta_ > 1.0)
        {
            std::cout << "overdamped" << std::endl;
        }
        if (zeta_ < 1.0)
        {
            std::cout << "underdamped" << std::endl;
        }
    }

private:
    SCALAR w_n_;         //!< eigenfrequency
    SCALAR w_n_square_;  //!< eigenfrequency squared
    SCALAR zeta_;        //!< damping ratio
    SCALAR g_dc_;        //!< input DC gain
};

}

typedef TAC::CAR::sim_car_system<double> sim_car_system;  //!< harmonic oscillator (double)

}
