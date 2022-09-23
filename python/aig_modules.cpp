#include <boost/python/module.hpp>
#include <eigenpy/eigenpy.hpp>
// #include <Eigen/Dense>

#include "aig/python.hpp"

typedef Eigen::Matrix<double, 6, 1, 0, 6, 1> eMatrix61;

BOOST_PYTHON_MODULE(aig) {
  // Enabling eigenpy support, i.e. numpy/eigen compatibility.
  eigenpy::enableEigenPy();
  ENABLE_SPECIFIC_MATRIX_TYPE(eMatrix61);
  ENABLE_SPECIFIC_MATRIX_TYPE(Eigen::MatrixXd);
  ENABLE_SPECIFIC_MATRIX_TYPE(Eigen::VectorXd);
  aig::python::exposeContact6D();
  aig::python::exposeDynaCoM();
}
