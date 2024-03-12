#ifndef IP_PROBLEM_BASICSETUP_H 
#define IP_PROBLEM_BASICSETUP_H

#include <Eigen/Core>

#include <adolc/adolc.h>

template <typename Scalar = double, typename Scalar2 = Scalar>
class IpProblemBasicSetup
{
public:

    /** default costructor */
    IpProblemBasicSetup() = default;

    virtual ~IpProblemBasicSetup() = default;

    virtual IpProblemBasicSetup<adouble, Scalar2>* clone() const;

    void setVariables(const std::vector<Scalar2> & variables);

    const std::vector<Scalar2> & getVariables() const;

public:

    virtual void get_nlp_info(int& n, int& m);

    virtual void get_starting_point(int n, Scalar2* x);

    virtual void get_nlp_bounds(int n, Scalar2* xlb, Scalar2* xub);

    virtual void get_constraint_bounds(int m, Scalar2* g_l, Scalar2* g_u);

    virtual Scalar eval_obj(const Scalar* x);

    virtual void eval_constraints(int m, const Scalar* x, Scalar* g);

    std::vector<Scalar2> variables_;
};

template <typename Scalar, typename Scalar2>
IpProblemBasicSetup<adouble, Scalar2>* IpProblemBasicSetup<Scalar, Scalar2>::clone() const
{
    IpProblemBasicSetup<adouble, Scalar2>* prob = new IpProblemBasicSetup<adouble, Scalar2>();
    return prob;
}

template <typename Scalar, typename Scalar2>
void IpProblemBasicSetup<Scalar, Scalar2>::setVariables(const std::vector<Scalar2> & variables)
{
    variables_ = variables;
}

template <typename Scalar, typename Scalar2>
const std::vector<Scalar2> & IpProblemBasicSetup<Scalar, Scalar2>::getVariables() const
{
    return variables_;
}

/****************************************************************************************************/
// specialization for a particular problem
// returns the size of the problem
template <typename Scalar, typename Scalar2>
void IpProblemBasicSetup<Scalar, Scalar2>::get_nlp_info(int& n, int& m)
{
    // Number of variables
    n = 0;
    // Number of constraints in g(x)
    m = 0;
}

template <typename Scalar, typename Scalar2>
void IpProblemBasicSetup<Scalar, Scalar2>::get_starting_point(int n, Scalar2* x)
{
    for (int i = 0; i < n; i++)
    {
        x[i] = 0.0;
    }
}

template <typename Scalar, typename Scalar2>
void IpProblemBasicSetup<Scalar, Scalar2>::get_nlp_bounds(int n, Scalar2* xlb, Scalar2* xub)
{
    for (int i = 0; i < n; i++)
    {
        xlb[i] = 0.0;
        xub[i] = 0.0;
    }
}

template <typename Scalar, typename Scalar2>
void IpProblemBasicSetup<Scalar, Scalar2>::get_constraint_bounds(int m, Scalar2* g_l, Scalar2* g_u)
{
    for (int i = 0; i < m; i++)
    {
        g_l[i] = 0.0;
        g_u[i] = 0.0;
    }
}

template <typename Scalar, typename Scalar2>
Scalar IpProblemBasicSetup<Scalar, Scalar2>::eval_obj(const Scalar* /*x*/)
{
    return Scalar(0.0);
}

template <typename Scalar, typename Scalar2>
void IpProblemBasicSetup<Scalar, Scalar2>::eval_constraints(int m, const Scalar* /*x*/, Scalar* g)
{
    for (int i = 0; i < m; i++)
    {
        g[i] = Scalar(0.0);
    }
}
#endif
