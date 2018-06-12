#ifndef UTILITEST_H_
#define UTILITEST_H_
#include <Eigen/Core>
#include <Eigen/Cholesky>
#include <Eigen/LU>
#include <cmath>


namespace vct {

    template <typename T>
    using State = Eigen::Matrix<T, Eigen::Dynamic, 1>;

    template<typename T>
    using Control = State<T>;

    template<typename T>
    using Measurement = State<T>;

    template <typename T>
    using Weights = State<T>;

}


namespace mtx {

    template <typename T>
    using Matrix = Eigen::Matrix<T, Eigen::Dynamic, Eigen::Dynamic>;

}


#endif

