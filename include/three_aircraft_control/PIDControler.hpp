#pragma once

class PIDControler : public ct::core::Controller<2, 1>
{
public:
    static const size_t state_dim = 2;
    static const size_t control_dim = 1;

    PIDControler(const ct::core::ControlVector<control_dim>& uff, const double& kp, const double& ki, const double& kd): uff_(uff), kp_(kp), ki_(ki), kd_(kd)
    {

    }

    ~PIDControler()
    {

    }

    PIDControler(const PIDControler& other) : uff_(other.uff_), kp_(other.kp_), ki_(other.ki_), kd_(other.kd_)
    {

    }

    PIDControler* clone() const override
    {
        return new PIDControler(*this);
    }

    void computeControl(const ct::core::StateVector<state_dim>& state, const double& t, ct::core::ControlVector<control_dim>& controlAction) override
    {
        controlAction = uff_;
        controlAction(0) -= kp_ * state(0) + ki_ * state(1) + 
    }

}
