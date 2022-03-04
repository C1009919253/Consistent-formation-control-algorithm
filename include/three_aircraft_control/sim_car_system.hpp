#pragma once
#include <ct/core/systems/continuous_time/ControlledSystem.h>

namespace TAC
{
namespace CAR
{

template <typename SCALAR>
class sim_car_system : ct::core::ControlledSystem<3, 2, SCALAR>
{

public:

    sim_car_system()
    {
        
    }

    size_t STATE_DIM = 3;
    size_t CONTROL_DIM = 2;

    void computeControlledDynamics(const ct::core::StateVector<3, SCALAR>& state,
                                   const double& t,
                                   const ct::core::ControlVector<2, SCALAR>& control,
                                   ct::core::StateVector<3, SCALAR>& derivative) override
    {
        derivative(0) = ct::core::tpl::TraitSelector<SCALAR>::Trait::cos(state(2)) * control(0);
        derivative(1) = ct::core::tpl::TraitSelector<SCALAR>::Trait::sin(state(2)) * control(0);
        derivative(2) = control(1);
    }
};


}
}
