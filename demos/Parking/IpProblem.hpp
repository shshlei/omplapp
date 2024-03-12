#ifndef IP_PROBLEM_H 
#define IP_PROBLEM_H

#include "IpProblemBasicSetup.hpp" 

#include <Eigen/Core>

#include <adolc/adolc.h>
#include <adolc/adolc_sparse.h>

#include <IpTNLP.hpp>
#include <IpIpoptApplication.hpp>
using namespace Ipopt;

template <typename Scalar = double>
class IpProblem : public TNLP
{
public:

    /** default costructor */
    IpProblem(IpProblemBasicSetup<Scalar> * problem);

    /** This method is called when the algorithm is complete so the TNLP can store/write the solution */
    virtual void finalize_solution(SolverReturn status,
        Index n, const Number* x, const Number* z_L, const Number* z_U,
        Index m, const Number* g, const Number* lambda,
        Number obj_value, const IpoptData* ip_data, IpoptCalculatedQuantities* ip_cq);

    /************* The following code does not need to change  ******************/
public:

    /** default destructor */
    virtual ~IpProblem();

    /** Method to return some info about the nlp */
    bool intermediate_callback(AlgorithmMode mode,
        Index iter, Number obj_value,
        Number inf_pr, Number inf_du,
        Number mu, Number d_norm,
        Number regularization_size,
        Number alpha_du, Number alpha_pr,
        Index ls_trials,
        const IpoptData* ip_data,
        IpoptCalculatedQuantities* ip_cq);

    virtual bool get_nlp_info(Index& n, Index& m, Index& nnz_jac_g,
        Index& nnz_h_lag, IndexStyleEnum& index_style);

    /** Method to return the bounds for my problem */
    virtual bool get_bounds_info(Index n, Number* x_l, Number* x_u,
        Index m, Number* g_l, Number* g_u);

    /** Method to return the starting point for the algorithm */
    virtual bool get_starting_point(Index n, bool init_x, Number* x,
        bool init_z, Number* z_L, Number* z_U,
        Index m, bool init_lambda, Number* lambda);

    /** Method to return the objective value */
    virtual bool eval_f(Index n, const Number* x, bool new_x, Number& obj_value);

    /** Method to return the gradient of the objective */
    virtual bool eval_grad_f(Index n, const Number* x, bool new_x, Number* grad_f);

    /** Method to return the constraint residuals */
    virtual bool eval_g(Index n, const Number* x, bool new_x, Index m, Number* g);

    /** Method to return:
     *   1) The structure of the jacobian (if "values" is NULL)
     *   2) The values of the jacobian (if "values" is not NULL)
     */
    virtual bool eval_jac_g(Index n, const Number* x, bool new_x,
        Index m, Index nele_jac, Index* iRow, Index* jCol, Number* values);

    /** Method to return:
     *   1) The structure of the hessian of the lagrangian (if "values" is NULL)
     *   2) The values of the hessian of the lagrangian (if "values" is not NULL)
     */
    virtual bool eval_h(Index n, const Number* x, bool new_x, Number obj_factor,
        Index m, const Number* lambda, bool new_lambda, Index nele_hess, Index* iRow, Index* jCol, Number* values);

private:

    IpProblem(const IpProblem&);

    IpProblem& operator=(const IpProblem&);

    /** Method to generate the required tapes */
    void generate_tapes(Index n, Index m, Index& nnz_jac_g, Index& nnz_h_lag);

private:

    IpProblemBasicSetup<Scalar> * problem_;

    // auto differentiation
    Scalar* obj_lam_{nullptr};

    //** variables for sparsity exploitation
    unsigned int* rind_g_{nullptr};

    unsigned int* cind_g_{nullptr};

    Scalar* jacval_{nullptr};

    unsigned int* rind_L_{nullptr};

    unsigned int* cind_L_{nullptr};

    Scalar* hessval_{nullptr};

    int nnz_jac_{0};

    int nnz_L_{0};

    int options_g_[4] = {0, 0, 0, 0};

    int options_L_[2] = {0, 1};

    // tape tags to be used by ADOL_C
    int tag_f_{1};

    int tag_g_{2};

    int tag_hess_{3};
};

template <typename Scalar>
IpProblem<Scalar>::IpProblem(IpProblemBasicSetup<Scalar> * problem) : problem_(problem)
{
}

template <typename Scalar>
IpProblem<Scalar>::~IpProblem()
{
    delete[] obj_lam_;
    obj_lam_ = nullptr;
    free(rind_g_);
    free(cind_g_);
    free(jacval_);
    rind_g_ = nullptr;
    cind_g_ = nullptr;
    jacval_ = nullptr;

    free(rind_L_);
    free(cind_L_);
    free(hessval_);
    rind_L_ = nullptr;
    cind_L_ = nullptr;
    hessval_ = nullptr;
}

template <typename Scalar>
void IpProblem<Scalar>::finalize_solution(SolverReturn /*status*/,
    Index n, const Number* x, const Number* /*z_L*/, const Number* /*z_U*/,
    Index /*m*/, const Number* /*g*/, const Number* /*lambda*/,
    Number /*obj_value*/, const IpoptData* /*ip_data*/, IpoptCalculatedQuantities* /*ip_cq*/)
{
    // here is where we would store the solution to variables, or write to a file, etc
    // so we could use the solution.
    std::vector<Scalar> variables;
    variables.reserve(n);
    for (int i = 0; i < n; i++)
        variables.push_back(x[i]);
    problem_->setVariables(variables);
}

template <typename Scalar>
bool IpProblem<Scalar>::intermediate_callback(AlgorithmMode /*mode*/,
    Index /*iter*/, Number /*obj_value*/,
    Number /*inf_pr*/, Number /*inf_du*/,
    Number /*mu*/, Number /*d_norm*/,
    Number /*regularization_size*/,
    Number /*alpha_du*/, Number /*alpha_pr*/,
    Index /*ls_trials*/,
    const IpoptData* /*ip_data*/,
    IpoptCalculatedQuantities* /*ip_cq*/)
{
    return true;
}

template <typename Scalar>
bool IpProblem<Scalar>::get_nlp_info(Index& n, Index& m, Index& nnz_jac_g, Index& nnz_h_lag, IndexStyleEnum& index_style)
{
    problem_->get_nlp_info(n, m);

    generate_tapes(n, m, nnz_jac_g, nnz_h_lag);

    // use the C style indexing (0-based)
    index_style = TNLP::C_STYLE;

    return true;
}

// returns the variable bounds
template <typename Scalar>
bool IpProblem<Scalar>::get_bounds_info(Index n, Number* x_l, Number* x_u, Index m, Number* g_l, Number* g_u)
{
    // here, the n and m we gave IPOPT in get_nlp_info are passed back to us.
    // If desired, we could assert to make sure they are what we think they are.
    problem_->get_nlp_bounds(n, x_l, x_u);
    problem_->get_constraint_bounds(m, g_l, g_u);
    return true;
}

// returns the initial point for the problem
template <typename Scalar>
bool IpProblem<Scalar>::get_starting_point(Index n, bool /*init_x*/, Number* x,
    bool /*init_z*/, Number* /*z_L*/, Number* /*z_U*/,
    Index m, bool /*init_lambda*/, Number* lambda)
{
    problem_->get_starting_point(n, x);
    for (int i = 0; i < m; i++)
    {
        lambda[i] = 1.0;
    }
    return true;
}

// returns the value of the objective function
template <typename Scalar>
bool IpProblem<Scalar>::eval_f(Index /*n*/, const Number* x, bool /*new_x*/, Number& obj_value)
{
    obj_value = problem_->eval_obj(x);
    return true;
}

// return the gradient of the objective function grad_{x} f(x)
template <typename Scalar>
bool IpProblem<Scalar>::eval_grad_f(Index n, const Number* x, bool /*new_x*/, Number* grad_f)
{
    gradient(tag_f_, n, x, grad_f);
    return true;
}

// return the value of the constraints: g(x)
template <typename Scalar>
bool IpProblem<Scalar>::eval_g(Index /*n*/, const Number* x, bool /*new_x*/, Index m, Number* g)
{
    problem_->eval_constraints(m, x, g);
    return true;
}

// return the structure or values of the jacobian
template <typename Scalar>
bool IpProblem<Scalar>::eval_jac_g(Index n, const Number* x, bool /*new_x*/,
    Index m, Index /*nele_jac*/, Index* iRow, Index* jCol, Number* values)
{
    if (values == nullptr)
    {
        // return the structure of the jacobian
        for(int i = 0; i < nnz_jac_; i++)
        {
            iRow[i] = rind_g_[i];
            jCol[i] = cind_g_[i];
        }
    }
    else
    {
        // return the values of the jacobian of the constraints
        sparse_jac(tag_g_, m, n, 1, x, &nnz_jac_, &rind_g_, &cind_g_, &jacval_, options_g_); 
        std::copy_n(jacval_, nnz_jac_, values);
    }

    return true;
}

//return the structure or values of the hessian
template <typename Scalar>
bool IpProblem<Scalar>::eval_h(Index n, const Number* x, bool /*new_x*/,
    Number obj_factor, Index m, const Number* lambda, bool /*new_lambda*/,
    Index /*nele_hess*/, Index* iRow, Index* jCol, Number* values)
{
    if (values == NULL)
    {
        // return the structure. This is a symmetric matrix, fill the lower left
        // triangle only.
        for (int i = 0; i < nnz_L_; i++)
        {
            iRow[i] = rind_L_[i];
            jCol[i] = cind_L_[i];
        }
    }
    else
    {
        // return the values. This is a symmetric matrix, fill the lower left
        // triangle only
        obj_lam_[0] = obj_factor;
        std::copy_n(lambda, m, obj_lam_ + 1);

        set_param_vec(tag_hess_, m + 1, obj_lam_);
        sparse_hess(tag_hess_, n, 1, const_cast<Scalar*>(x), &nnz_L_, &rind_L_, &cind_L_, &hessval_, options_L_);
        std::copy_n(hessval_, nnz_L_, values);
    }

    return true;
}

template <typename Scalar>
void IpProblem<Scalar>::generate_tapes(Index n, Index m, Index& nnz_jac_g, Index& nnz_h_lag)
{
    delete[] obj_lam_;
    obj_lam_ = new Scalar[m + 1];

    IpProblemBasicSetup<adouble, Scalar>* prob = problem_->clone();

    Scalar x[n];
    problem_->get_starting_point(n, x);

    adouble* xad = new adouble[n];
    adouble* gad = new adouble[m];

    trace_on(tag_f_);
    for (int i = 0; i < n; i++)
        xad[i] <<= x[i];
    adouble yad = prob->eval_obj(xad);
    Scalar yp(0.0);
    yad >>= yp;
    trace_off();

    int repeat = 0;
    trace_on(tag_g_);
    for (int i = 0; i < n; i++) // todo
        xad[i] <<= x[i];
    prob->eval_constraints(m, xad, gad);

    Scalar g_value(0.0);
    for (int i = 0; i < m; i++)
        gad[i] >>= g_value;
    trace_off();

    sparse_jac(tag_g_, m, n, repeat, x, &nnz_jac_, &rind_g_, &cind_g_, &jacval_, options_g_);
    nnz_jac_g = nnz_jac_;

    repeat = 0;
    trace_on(tag_hess_);
    for (int i = 0; i < n; i++) // todo
        xad[i] <<= x[i];
    std::fill_n(obj_lam_, m + 1, Scalar(1.0));

    yad = prob->eval_obj(xad);
    yad *= mkparam(obj_lam_[0]);

    prob->eval_constraints(m, xad, gad);
    for (int i = 0; i < m; i++)
        yad += gad[i] * mkparam(obj_lam_[i + 1]);

    yad >>= yp;
    trace_off();

    sparse_hess(tag_hess_, n, repeat, x, &nnz_L_, &rind_L_, &cind_L_, &hessval_, options_L_);
    nnz_h_lag = nnz_L_;

    delete prob;
    delete[] xad;
    delete[] gad;
    prob = nullptr;
    xad = nullptr;
    gad = nullptr;
}

template <typename Scalar = double>
bool solveIpProblem(IpProblemBasicSetup<Scalar> * problem)
{
    // Create a new instance of nlp
    SmartPtr<TNLP> mynlp = new IpProblem<Scalar>(problem);

    // Create a new instance of IpoptApplication
    SmartPtr<IpoptApplication> app = new IpoptApplication();

    // Change some options
    app->Options()->SetNumericValue("tol", 1.e-8);
    app->Options()->SetIntegerValue("max_iter", 3000);
    app->Options()->SetStringValue("linear_solver", "ma57");
    app->Options()->SetIntegerValue("print_level", 0);
    app->Options()->SetStringValue("mu_strategy", "adaptive");
    app->Options()->SetStringValue("nlp_scaling_method", "gradient-based");
    app->Options()->SetStringValue("hessian_approximation", "exact");
    app->Options()->SetNumericValue("nlp_scaling_max_gradient", 1e6);

    // Intialize the IpoptApplication and process the options
    ApplicationReturnStatus status = app->Initialize();
    if (status != Solve_Succeeded)
    {
        return false;
    }

    // Ask Ipopt to solve the problem
    status = app->OptimizeTNLP(mynlp);
    if (status == Solve_Succeeded)
    {
        return true;
    }

    return false;
}

#endif
