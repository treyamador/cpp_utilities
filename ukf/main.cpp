#include <iostream>
#include "unscented_kalman_filter.h"


int main(int argc, char* argv[]) {

    int state_size = 3;
    int data_size = 10;

    //model::ProcessModel<double>* motion_model =
    //    new model::OdometryMotionModel<double>();

    model::ProcessModel<double>* motion_model =
        new model::SimpleMotionModel<double>();

    ukf::UnscentedKalmanFilter filter(state_size);
    filter.setMotionModel(motion_model);

    state::StateVector<double> state_vector(state_size);
    mtx::CovarianceMatrix<double> covariance_matrix(state_size,state_size);
    state::ControlVector<double> control_vector(state_size);
    state::MeasurementVector<double> measurement_vector(state_size);

    filter.update(
        state_vector,
        covariance_matrix,
        control_vector,
        measurement_vector);

	return 0;
}

