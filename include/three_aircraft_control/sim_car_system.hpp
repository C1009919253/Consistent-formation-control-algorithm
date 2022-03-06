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
    static const size_t STATE_DIM = 3;
    static const size_t CONTROL_DIM = 2;

    typedef ControlledSystem<3, 2, SCALAR> Base;
    typedef typename Base::time_t time_t;

    //! default constructor
    sim_car_system() = delete;
    sim_car_system(
        std::shared_ptr<Controller<3, 2, SCALAR>> controller = nullptr)
        : ControlledSystem<3, 2, SCALAR>(controller, SYSTEM_TYPE::GENERAL)
    {
    }
    //! copy constructor
    sim_car_system(const sim_car_system& arg)
        : ControlledSystem<3, 2, SCALAR>(arg)
    {
    }

    //! deep copy
    sim_car_system* clone() const override { return new sim_car_system(*this); }
    //! destructor
    virtual ~sim_car_system() {}

    virtual void computeControlledDynamics(const StateVector<3, SCALAR>& state,
        const time_t& t,
        const ControlVector<2, SCALAR>& control,
        StateVector<3, SCALAR>& derivative) override
    {
        derivative(0) = ct::core::tpl::TraitSelector<SCALAR>::Trait::cos(state(2)) * control(0);
        derivative(1) = ct::core::tpl::TraitSelector<SCALAR>::Trait::sin(state(2)) * control(0);
        derivative(2) = control(1);
    }

    bool checkParameters()
    {


        return true;
    }

    //! print out infos about the system
    void printSystemInfo()
    {

        std::cout << "System is ";
    }

private:
};

}

typedef TAC::CAR::sim_car_system<double> sim_car_system;  //!< harmonic oscillator (double)

}
