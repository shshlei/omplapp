/*********************************************************************
* Software License Agreement (BSD License)
*
*  Copyright (c) 2010, Rice University
*  All rights reserved.
*
*  Redistribution and use in source and binary forms, with or without
*  modification, are permitted provided that the following conditions
*  are met:
*
*   * Redistributions of source code must retain the above copyright
*     notice, this list of conditions and the following disclaimer.
*   * Redistributions in binary form must reproduce the above
*     copyright notice, this list of conditions and the following
*     disclaimer in the documentation and/or other materials provided
*     with the distribution.
*   * Neither the name of the Rice University nor the names of its
*     contributors may be used to endorse or promote products derived
*     from this software without specific prior written permission.
*
*  THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
*  "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
*  LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS
*  FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE
*  COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
*  INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING,
*  BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
*  LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
*  CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
*  LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN
*  ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
*  POSSIBILITY OF SUCH DAMAGE.
*********************************************************************/

/* Author: Shi Shenglei */

#include <psopt/problem.hpp>
#include <psopt/solver.hpp>
#include <bomp/utils.hpp>
#include <bomp/collision_constraints/J_function_collision_constraints.h>
#include <bomp/collision_constraints/active_points_collision_constraints.h>

#include <boost/program_options.hpp>

bool argParse(int argc, char** argv, std::size_t & expected_nnodes);

template <typename Scalar = double, typename Scalar2 = Scalar>
class DrivingProblemBase : public psopt::Problem<Scalar, Scalar2>
{
public:

    DrivingProblemBase(psopt::ProblemInfo<Scalar2>* prob, const VehicleParam<Scalar2>* vehicleParam) : psopt::Problem<Scalar, Scalar2>(prob), vehicleParam_(vehicleParam)
    {
    }

    virtual ~DrivingProblemBase() = default;

    psopt::Problem<adouble, Scalar2>* clone() const override
    {
        DrivingProblemBase<adouble, Scalar2>* prob = new DrivingProblemBase<adouble, Scalar2>(this->problemInfo_, this->vehicleParam_);
        prob->setLinearizedParameters(this);
        return prob;
    }

    template <typename Scalar3 = Scalar>
    void setLinearizedParameters(const DrivingProblemBase<Scalar3, Scalar2> * prob)
    {
        dx_dv_ = prob->dx_dv_, dy_dv_ = prob->dy_dv_, dx_dtheta_ = prob->dx_dtheta_, dy_dtheta_ = prob->dy_dtheta_; 
        dtheta_dv_ = prob->dtheta_dv_, dtheta_dalpha_ = prob->dtheta_dalpha_; 
        di_dv_ = prob->di_dv_;
    }

    Scalar endpointCost(const Scalar* /*initial_states*/, const Scalar* /*final_states*/,
        const Scalar* /*initial_controls*/, const Scalar* /*final_controls*/, 
        const Scalar* /*parameters*/, const Scalar& t0, const Scalar& tf,
        const Scalar* /*xad*/, std::size_t /*iphase*/) const override
    {
        return (tf - t0);
    }

    Scalar integrandCost(const Scalar* /*states*/, const Scalar* controls, const Scalar* /*parameters*/,
        const Scalar& /*time*/, const Scalar* /*xad*/, std::size_t /*iphase*/) const override
    {
        const Scalar& v = controls[0];
        return 0.5 * v * v;
    }

    void dae(Scalar* derivatives, const Scalar* states,
        const Scalar* controls, const Scalar* /*parameters*/, const Scalar& /*time*/, const Scalar* /*xad*/, std::size_t /*iphase*/) const override
    {
        // parameters for the car
        Scalar2 l = this->vehicleParam_->l;

        const Scalar& theta = states[2];
        const Scalar& alpha = states[3];
        const Scalar& v = controls[0];
        const Scalar& omega = controls[1];

        // kinematics
        derivatives[0] = v * cos(theta);
        derivatives[1] = v * sin(theta);
        derivatives[2] = v * tan(alpha) / l;
        derivatives[3] = omega;
    }

    void pathAlongTrajectory(Scalar* /*paths*/, const Scalar* /*states*/,
        const Scalar* /*controls*/, const Scalar* /*parameters*/, const Scalar& /*time*/, const Scalar* /*xad*/, std::size_t /*iphase*/) const override
    {
    }

    void path(Scalar* /*paths*/, const Scalar* /*states*/, const Scalar* /*parameters*/,
        const Scalar* /*x*/, std::size_t /*iphase*/) const override
    {
    }

    void events(Scalar* e, const Scalar* initial_states, const Scalar* final_states,
        const Scalar* initial_controls, const Scalar* final_controls, const Scalar* /*parameters*/,
        const Scalar& /*t0*/, const Scalar& /*tf*/, const Scalar* /*xad*/, std::size_t iphase) const override
    {
        if (iphase == 0) // first phase
        {
            const Scalar& x0 = initial_states[0];
            const Scalar& y0 = initial_states[1];
            const Scalar& theta0 = initial_states[2];
            const Scalar& v0 = initial_controls[0];

            // 4 initial events
            int offset = 0;
            e[offset] = x0;
            offset += 1;
            e[offset] = y0;
            offset += 1;
            e[offset] = theta0;
            offset += 1;
            e[offset] = v0;
            offset += 1;
        }
        if (iphase == this->problemInfo_->getPhaseNumbers() - 1) // last phase
        {
            const Scalar& xf = final_states[0];
            const Scalar& yf = final_states[1];
            const Scalar& thetaf = final_states[2];
            const Scalar& vf = final_controls[0];

            int offset = 0;
            if (this->problemInfo_->getPhaseNumbers() == 1) // one phase
                offset = 4; // 4 initial events

            // 4 final events
            e[offset] = thetaf;
            offset += 1;
            e[offset] = vf;
            offset += 1;
            e[offset] = xf;
            offset += 1;
            e[offset] = yf;
            offset += 1;
        }
    }

protected:
    
    const VehicleParam<Scalar2>* vehicleParam_;

public:

    Scalar linearized_integrandCost(const Scalar* /*dstates*/, const Scalar* dcontrols, const Scalar* /*parameters*/,
        const Scalar& /*dtime*/, const Scalar* /*x*/, std::size_t iphase, std::size_t node) const override
    {
        const Scalar& dv = dcontrols[0];
        const std::vector<Scalar2> & di_dv = di_dv_[iphase]; 
        return di_dv[node] * dv;
    }

    void linearized_dae(Scalar* derivatives, const Scalar* dstates, const Scalar* dcontrols, const Scalar* /*parameters*/,
        const Scalar& /*dtime*/, const Scalar* /*x*/, std::size_t iphase, std::size_t node) const override
    {
        const Scalar& dtheta = dstates[2];
        const Scalar& dalpha = dstates[3];
        const Scalar& dv = dcontrols[0];
        const Scalar& domega = dcontrols[1];

        const std::vector<Scalar2> & dx_dv = dx_dv_[iphase]; 
        const std::vector<Scalar2> & dy_dv = dy_dv_[iphase]; 
        const std::vector<Scalar2> & dx_dtheta = dx_dtheta_[iphase]; 
        const std::vector<Scalar2> & dy_dtheta = dy_dtheta_[iphase]; 
        const std::vector<Scalar2> & dtheta_dv = dtheta_dv_[iphase];
        const std::vector<Scalar2> & dtheta_dalpha = dtheta_dalpha_[iphase];
        
        // linearized kinematics
        derivatives[0] = dx_dtheta[node] * dtheta + dx_dv[node] * dv;
        derivatives[1] = dy_dtheta[node] * dtheta + dy_dv[node] * dv;
        derivatives[2] = dtheta_dalpha[node] * dalpha + dtheta_dv[node] * dv;
        derivatives[3] = domega;
    }

    void setUpLinearizedParameters(const Scalar2 * nominal_variables) override 
    {
        // parameters for the car
        Scalar2 l = this->vehicleParam_->l;

        DrivingProblemBase<Scalar2, Scalar2> prob(this->problemInfo_, this->vehicleParam_);

        std::size_t nphases = this->problemInfo_->getPhaseNumbers();
        std::vector<Scalar2> nominal_integral_quadrature(nphases);
        std::vector<std::vector<Scalar2>> nominal_derivs_traj(nphases);

        dx_dv_.clear(), dy_dv_.clear(), dx_dtheta_.clear(), dy_dtheta_.clear(); 
        dtheta_dv_.clear(), dtheta_dalpha_.clear(); 
        dx_dv_.resize(nphases), dy_dv_.resize(nphases), dx_dtheta_.resize(nphases), dy_dtheta_.resize(nphases); 
        dtheta_dv_.resize(nphases), dtheta_dalpha_.resize(nphases); 

        di_dv_.clear();
        di_dv_.resize(nphases);

        for (std::size_t iphase = 0; iphase < nphases; iphase++)
        {
            std::size_t nnodes = this->problemInfo_->getPhaseNumberNodes(iphase);
            std::size_t nstates = this->problemInfo_->getPhaseNumberStates(iphase);
            std::size_t ncontrols = this->problemInfo_->getPhaseNumberControls(iphase);

            std::size_t x_phase_offset = this->problemInfo_->getPhaseOffsetVariables(iphase);
            std::size_t state_offset = x_phase_offset + ncontrols * nnodes;
            const Scalar2 * controls = nominal_variables + x_phase_offset;
            const Scalar2 * states = nominal_variables + state_offset;

            const std::vector<Scalar2>& nodes = this->problemInfo_->getNodes(nnodes - 1);
            const std::vector<Scalar2>& weights = this->problemInfo_->getWeights(nnodes - 1);

            std::vector<Scalar2> dx_dv(nnodes), dy_dv(nnodes), dx_dtheta(nnodes), dy_dtheta(nnodes); 
            std::vector<Scalar2> dtheta_dv(nnodes), dtheta_dalpha(nnodes), di_dv(nnodes); 
            nominal_derivs_traj[iphase].resize(nstates * nnodes);

            Scalar2 sum_cost(0.0);
            std::size_t derivs_offset = 0;
            Scalar2 * derivatives = new Scalar2[nstates];
            for (std::size_t i = 0; i < nnodes; i++)
            {
                Scalar2 theta = states[2];
                Scalar2 alpha = states[3];
                Scalar2 v = controls[0];
                Scalar2 c = std::cos(theta);
                Scalar2 s = std::sin(theta);

                dx_dv[i] = c;
                dy_dv[i] = s;
                dx_dtheta[i] = -v * s;
                dy_dtheta[i] = v * c;
                dtheta_dv[i] = std::tan(alpha) / l;
                dtheta_dalpha[i] = v / (l * std::cos(alpha) * std::cos(alpha));
                di_dv[i] = v;

                prob.dae(derivatives, states, controls, nullptr, 0.0, nullptr, iphase);
                std::copy_n(derivatives, nstates, nominal_derivs_traj[iphase].data() + derivs_offset);
                derivs_offset += nstates;

                Scalar2 integrand_cost = prob.integrandCost(states, controls, nullptr, 0.0, nullptr, iphase);
                if (this->problemInfo_->getCollocationMethod() == "Chebyshev")
                    // Multiply by the reciprocal of the Chebyshev weighting function to evaluate the correct integral.
                    integrand_cost *= std::sqrt(1.0 - nodes[i] * nodes[i]);
                sum_cost += integrand_cost * weights[i];

                states += nstates;
                controls += ncontrols;
            }

            dx_dv_[iphase].swap(dx_dv), dy_dv_[iphase].swap(dy_dv), dx_dtheta_[iphase].swap(dx_dtheta), dy_dtheta_[iphase].swap(dy_dtheta); 
            dtheta_dv_[iphase].swap(dtheta_dv), dtheta_dalpha_[iphase].swap(dtheta_dalpha);
            di_dv_[iphase].swap(di_dv);

            nominal_integral_quadrature[iphase] = sum_cost;
            delete[] derivatives;
            derivatives = nullptr;
        }
        this->problemInfo_->setNominalIntegralQuadrature(nominal_integral_quadrature);
        this->problemInfo_->setNominalDerivsTraj(nominal_derivs_traj);
    }

    // linearized dynamics
    std::vector<std::vector<Scalar2>> dx_dv_, dy_dv_, dx_dtheta_, dy_dtheta_; 
    std::vector<std::vector<Scalar2>> dtheta_dv_, dtheta_dalpha_; 

    // linearized integral
    std::vector<std::vector<Scalar2>> di_dv_; 
};

void setStartAndGoal(double & x0, double & y0, double & theta0, double & xf, double & yf, double & thetaf)
{
    x0 = 0.05;
    y0 = 0.05;
    theta0 = 0.0;

    xf = 0.95;
    yf = 0.95;
    thetaf = 0.0;
}

template <typename Scalar = double>
void driving_bounds(psopt::ProblemInfo<Scalar>* info, Scalar x0, Scalar y0, Scalar theta0, Scalar xf, Scalar yf, Scalar thetaf)
{
    Scalar xL = 0.0;
    Scalar xU = 1.0;
    Scalar yL = 0.0;
    Scalar yU = 1.0;

    std::vector<Scalar> slower{xL, yL, -1.1 * M_PI, -0.714};
    std::vector<Scalar> supper{xU, yU, 1.1 * M_PI, 0.714};
    std::vector<Scalar> clower{-0.2, -0.05};
    std::vector<Scalar> cupper{0.2, 0.05};
    for (std::size_t i = 0; i < info->getPhaseNumbers(); i++)
    {
        info->setPhaseLowerBoundStates(slower, i);
        info->setPhaseUpperBoundStates(supper, i);

        info->setPhaseLowerBoundControls(clower, i);
        info->setPhaseUpperBoundControls(cupper, i);

        info->setPhaseConstantLowerBoundPaths(Scalar(0.0001), i);
        info->setPhaseInfinityUpperBoundPaths(i);
    }

    if (info->getPhaseNumbers() == 1)
    {
        Scalar epsf = 0.001;
        Scalar epsthetaf = 0.035;
        std::vector<Scalar> elower{x0, y0, theta0, 0.0, thetaf - epsthetaf, -epsf, xf - epsf, yf - epsf};
        std::vector<Scalar> eupper{x0, y0, theta0, 0.0, thetaf + epsthetaf, epsf, xf + epsf, yf + epsf};
        info->setPhaseLowerBoundEvents(elower);
        info->setPhaseUpperBoundEvents(eupper);
    }
    else
    {
        std::vector<Scalar> elower{x0, y0, theta0, 0.0};
        std::vector<Scalar> eupper{x0, y0, theta0, 0.0};
        info->setPhaseLowerBoundEvents(elower);
        info->setPhaseUpperBoundEvents(eupper);

        Scalar epsf = 0.001;
        Scalar epsthetaf = 0.035;
        std::vector<Scalar> felower{thetaf - epsthetaf, -epsf, xf - epsf, yf - epsf};
        std::vector<Scalar> feupper{thetaf + epsthetaf, epsf, xf + epsf, yf + epsf};
        info->setPhaseLowerBoundEvents(felower, info->getPhaseNumbers() - 1);
        info->setPhaseUpperBoundEvents(feupper, info->getPhaseNumbers() - 1);
    }
}

// states, controls, time guess
template <typename Scalar = double>
void driving_guess(psopt::ProblemInfo<Scalar>* info, Scalar x0, Scalar y0, Scalar theta0)
{
    Scalar stime = 0.0;
    Scalar dtime = 10.0 / info->getPhaseNumbers();
    std::vector<Scalar> x_guess{x0, y0, theta0, 0.0};
    std::vector<Scalar> u_guess{0.0, 0.0}; // TODO
    for (std::size_t i = 0; i < info->getPhaseNumbers(); i++)
    {
        info->setPhaseConstantGuessStates(x_guess, i);
        info->setPhaseConstantGuessControls(u_guess, i);
        info->setPhaseGuessTime(stime, stime + dtime, i);
        stime += dtime;
    }
}

bool argParse(int argc, char** argv, std::size_t & expected_nnodes)
{
    namespace bpo = boost::program_options;
    // Declare the supported options.
    bpo::options_description desc("Allowed options");
    desc.add_options()
        ("help,h", "produce help message")
        ("expected_nnodes", bpo::value<std::size_t>()->default_value(20), "(Optional) Specify the discrete nnodes");
    bpo::variables_map vm;
    bpo::store(bpo::parse_command_line(argc, argv, desc), vm);
    bpo::notify(vm);
    // Check if the help flag has been given:
    if (vm.count("help") != 0u)
    {
        std::cout << desc << std::endl;
        return false;
    }
    expected_nnodes = vm["expected_nnodes"].as<std::size_t>();
    return true;
}
