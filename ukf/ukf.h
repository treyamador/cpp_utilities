#ifndef UKF_H_
#define UKF_H_
#include "utilities.h"
#include "models.h"
#include <cmath>


// TODO remove
// #include "../test-eigen/utilitest.h"


namespace ukf {

    namespace {
        const double ALPHA_DEF = 0.5;
        const double BETA_DEF = 2.0;
        const double KAPPA_DEF = 3.0;
    }


    template<class T>
    class UnscentedKalmanFilter {

        public:
            UnscentedKalmanFilter(int vars, int meas) :
                vars_(0),measures_(0),points_(0),
                lambda_(0.0),gamma_(0.0),

                process_model_(nullptr),sensor_model_(nullptr),

                state_belief_(vct::State<T>(vars)),
                state_obser_(vct::State<T>(meas)),

                covar_belief_(mtx::Matrix<T>(vars,vars)),
                covar_obser_(mtx::Matrix<T>(meas,meas)),
                covar_cross_(mtx::Matrix<T>(vars,meas)),

                covar_obser_inv_(mtx::Matrix<T>(meas,meas)),
                covar_cholesky_(mtx::Matrix<T>(vars,vars)),

                sigma_belief_(this->pointsPerState(vars),vars),
                sigma_predict_(this->pointsPerState(vars),vars),
                sigma_obser_(this->pointsPerState(vars),meas),

                mean_weight_(vct::Weights<T>(this->pointsPerState(vars))),
                covar_weight_(vct::Weights<T>(this->pointsPerState(vars))),

                noise_r_(mtx::Matrix<T>(vars,vars)),
                noise_q_(mtx::Matrix<T>(meas,meas)),
                kalman_gain_(mtx::Matrix<T>(vars,meas)),

                alpha_(ALPHA_DEF),
                beta_(BETA_DEF),
                kappa_(vars-KAPPA_DEF),

                // TODO remove this, will be unnecessary
                //      or replace with a motion vector
                place_holder_(vct::State<T>(vars))
            {
                this->init(vars,meas);
            }

            ~UnscentedKalmanFilter() {
                this->destroy();
            }

            void init(int vars, int meas) {
                this->vars_ = vars;
                this->measures_ = meas;
                this->points_ = this->pointsPerState(this->vars_);

                this->lambda_ = this->calculateLambda(
                    this->alpha_,this->kappa_,this->vars_);
                this->gamma_ = this->calculateGamma(
                    this->lambda_,this->vars_);

                this->populateMean(this->mean_weight_, this->lambda_);
                this->populateCovariance(this->covar_weight_,
                    this->lambda_,this->alpha_,this->beta_);

                this->noise_r_.setIdentity();
                this->noise_q_.setIdentity();

            }

            void destroy() {
                this->deallocateModel(this->process_model_);
                this->deallocateModel(this->sensor_model_);
            }

            template<typename S>
            void deallocateModel(S* elem) {
                if (elem != nullptr) {
                    delete elem;
                    elem = nullptr;
                }
            }

            T zeroMean(T lambda) {
                return lambda / (this->vars_+lambda);
            }

            T zeroCovariance(T lambda, T alpha, T beta) {
                T mean = this->zeroMean(lambda);
                return mean + (1 - std::pow(alpha,2) + beta);
            }

            T ithElement(T lambda) {
                return 1.0 / (2.0 * (lambda + this->vars_));
            }

            void populateMean(vct::Weights<T>& weights, T lambda) {
                weights(0) = this->zeroMean(lambda);
                T ith_elem = this->ithElement(lambda);
                for (int i = 1; i < this->points_; ++i)
                    weights(i) = ith_elem;
            }

            void populateCovariance(
                vct::Weights<T>& weights,
                T lambda, T alpha, T beta)
            {
                weights(0) = this->zeroCovariance(lambda,alpha,beta);
                T ith_elem = this->ithElement(lambda);
                for (int i = 1; i < this->points_; ++i)
                    weights(i) = ith_elem;
            }

            void setProcessModel(model::ProcessModel<T>* model) {
                this->deallocateModel(this->process_model_);
                this->process_model_ = model;
            }

            void setSensorModel(model::SensorModel<T>* sensor) {
                this->deallocateModel(this->sensor_model_);
                this->sensor_model_ = sensor;
            }

            void setNoiseR(mtx::Matrix<T>& noise) {
                this->noise_r_ = noise;
            }

            void setNoiseQ(mtx::Matrix<T>& noise) {
                this->noise_q_ = noise;
            }

            T calculateLambda(T alpha, T kappa, int n) {
                return std::pow(alpha,2)*(n+kappa)-n;
            }

            T calculateGamma(T lambda, int n) {
                return std::sqrt(n+lambda);
            }

            int pointsPerState(int state) {
                return 2*state+1;
            }

            void sigmaPoints(
                sigma::SigmaPoints<T>& sigma,
                vct::State<T>& state,
                mtx::Matrix<T>& covariance,
                mtx::Matrix<T>& cholesky,
                T gamma)
            {
                cholesky = covariance.llt().matrixL();
                sigma.generatePoints(state, cholesky, gamma);
            }

            void gFunction(
                sigma::SigmaPoints<T>& predict,
                sigma::SigmaPoints<T>& belief,
                vct::State<T>& control)
            {
                int points = predict.getNumPoints();
                for (int i = 0; i < points; ++i)
                    this->process_model_->calculate(
                        predict[i],belief[i],
                        control,this->place_holder_);
            }

            void hFunction(
                sigma::SigmaPoints<T>& predict,
                sigma::SigmaPoints<T>& belief)
            {
                this->sensor_model_->calculate(predict,belief);
            }

            void sumWeightedMean(
                vct::State<T>& state,
                sigma::SigmaPoints<T>& sigma,
                vct::Weights<T>& weights)
            {
                state.setZero();
                int points = sigma.getNumPoints(),
                    vars = sigma.getStateSize();
                for (int i = 0; i < points; ++i)
                    for (int j = 0; j < vars; ++j)
                        state(j) += sigma[i](j) * weights(i);
            }

            void sumWeightedCovariance(
                mtx::Matrix<T>& covariance,
                sigma::SigmaPoints<T>& sigma,
                vct::State<T>& belief,
                vct::Weights<T>& weight,
                mtx::Matrix<T>& noise)
            {
                this->sumWeightedCovariance(
                    covariance,
                    sigma,belief,
                    sigma,belief,
                    weight);
                covariance += noise;
            }


            void sumWeightedCovariance(
                mtx::Matrix<T>& covariance,
                sigma::SigmaPoints<T>& sigma_x,
                vct::State<T>& state_a,
                sigma::SigmaPoints<T>& sigma_z,
                vct::State<T>& state_b,
                vct::Weights<T>& weight)
            {
                covariance.setZero();
                int rows = sigma_x.getStateSize(),
                    inner = sigma_x.getNumPoints(),
                    cols = sigma_z.getStateSize();
                for (int i = 0; i < rows; ++i)
                    for (int j = 0; j < cols; ++j)
                        for (int k = 0; k < inner; ++k)
                            covariance(i,j) +=
                                weight(k) *
                                (sigma_x[k](i)-state_a(i)) *
                                (sigma_z[k](j)-state_b(j));
            }

            void kalmanGain(
                mtx::Matrix<T>& gain,
                mtx::Matrix<T>& covar,
                mtx::Matrix<T>& obser,
                mtx::Matrix<T>& inv)
            {
                inv = obser.inverse();
                gain = covar * inv;
            }

            void updateState(
                vct::State<T>& state,
                vct::State<T>& prev,
                vct::State<T>& meas,
                vct::State<T>& obser,
                mtx::Matrix<T>& gain)
            {
                int row = gain.rows(),
                    col = gain.cols();
                for (int i = 0; i < row; ++i) {
                    T elem = 0.0;
                    for (int j = 0; j < col; ++j)
                        elem += gain(i,j) * (meas(j)-obser(j));
                    state(i) = prev(i) + elem;
                }
            }

            void updateCovariance(
                mtx::Matrix<T>& covar,
                mtx::Matrix<T>& belief,
                mtx::Matrix<T>& gain,
                mtx::Matrix<T>& obser)
            {
                covar = gain * obser * gain.transpose();
                covar = belief - covar;
            }


            void update(
                vct::State<T>& state,
                mtx::Matrix<T>& covariance,
                vct::State<T>& control,
                vct::State<T>& measurement)
            {


                //std::cout << "\n1\n" << std::endl;
                //state.precisionPrint();
                //covariance.precisionPrint();
                //control.precisionPrint();
                //measurement.precisionPrint();



                this->sigmaPoints(
                    this->sigma_belief_,
                    state,
                    covariance,
                    this->covar_cholesky_,
                    this->gamma_);



                //std::cout << "\n2\n" << std::endl;
                //this->sigma_belief_.precisionPrint();
                //state.precisionPrint();
                //covariance.precisionPrint();
                //this->covar_cholesky_.precisionPrint();
                //std::cout << this->gamma_ << std::endl;



                this->gFunction(
                    this->sigma_predict_,
                    this->sigma_belief_,
                    control);



                //std::cout << "\n3\n" << std::endl;
                //this->sigma_predict_.precisionPrint();
                //this->sigma_belief_.precisionPrint();
                //control.precisionPrint();



                this->sumWeightedMean(
                    this->state_belief_,
                    this->sigma_predict_,
                    this->mean_weight_);



                //std::cout << "\n4\n" << std::endl;
                //this->state_belief_.precisionPrint();
                //this->sigma_predict_.precisionPrint();
                //this->mean_weight_.precisionPrint();



                this->sumWeightedCovariance(
                    this->covar_belief_,
                    this->sigma_predict_,
                    this->state_belief_,
                    this->covar_weight_,
                    this->noise_r_);



                //std::cout << "\n5\n" << std::endl;
                //this->covar_belief_.precisionPrint();
                //this->sigma_predict_.precisionPrint();
                //this->state_belief_.precisionPrint();
                //this->covar_weight_.precisionPrint();
                //this->noise_r_.precisionPrint();



                this->sigmaPoints(
                    this->sigma_belief_,
                    this->state_belief_,
                    this->covar_belief_,
                    this->covar_cholesky_,
                    this->gamma_);



                //std::cout << "\n6\n" << std::endl;
                //this->sigma_belief_.precisionPrint();
                //this->state_belief_.precisionPrint();
                //this->covar_belief_.precisionPrint();
                //this->covar_cholesky_.precisionPrint();
                //std::cout << this->gamma_ << std::endl;



                this->hFunction(
                    this->sigma_obser_,
                    this->sigma_belief_);



                //std::cout << "\n7\n" << std::endl;
                //this->sigma_obser_.precisionPrint();
                //this->sigma_belief_.precisionPrint();



                this->sumWeightedMean(
                    this->state_obser_,
                    this->sigma_obser_,
                    this->mean_weight_);



                //std::cout << "\n8\n" << std::endl;
                //this->state_obser_.precisionPrint();
                //this->sigma_predict_.precisionPrint();
                //this->mean_weight_.precisionPrint();



                this->sumWeightedCovariance(
                    this->covar_obser_,
                    this->sigma_obser_,
                    this->state_obser_,
                    this->covar_weight_,
                    this->noise_q_);



                //std::cout << "\n9\n" << std::endl;
                //this->covar_obser_.precisionPrint();
                //this->sigma_obser_.precisionPrint();
                //this->state_obser_.precisionPrint();
                //this->covar_weight_.precisionPrint();
                //this->noise_q_.precisionPrint();



                this->sumWeightedCovariance(
                    this->covar_cross_,
                    this->sigma_belief_,
                    this->state_belief_,
                    this->sigma_obser_,
                    this->state_obser_,
                    this->covar_weight_);



                //std::cout << "\n10\n" << std::endl;
                //this->covar_cross_.precisionPrint();
                //this->sigma_belief_.precisionPrint();
                //this->state_belief_.precisionPrint();
                //this->sigma_obser_.precisionPrint();
                //this->state_obser_.precisionPrint();
                //this->covar_weight_.precisionPrint();



                this->kalmanGain(
                    this->kalman_gain_,
                    this->covar_cross_,
                    this->covar_obser_,
                    this->covar_obser_inv_);



                //std::cout << "\n11\n" << std::endl;
                //this->kalman_gain_.precisionPrint();
                //this->covar_cross_.precisionPrint();
                //this->covar_obser_.precisionPrint();
                //this->covar_obser_inv_.precisionPrint();



                this->updateState(
                    state,
                    this->state_belief_,
                    measurement,
                    this->state_obser_,
                    this->kalman_gain_);



                //std::cout << "\n12\n" << std::endl;
                //state.precisionPrint();
                //this->state_belief_.precisionPrint();
                //measurement.precisionPrint();
                //this->state_obser_.precisionPrint();
                //this->kalman_gain_.precisionPrint();



                this->updateCovariance(
                    covariance,
                    this->covar_belief_,
                    this->kalman_gain_,
                    this->covar_obser_);



                //std::cout << "\n13\n" << std::endl;
                //covariance.precisionPrint();
                //this->covar_belief_.precisionPrint();
                //this->kalman_gain_.precisionPrint();
                //this->covar_obser_.precisionPrint();



            }


        private:
            model::ProcessModel<T>* process_model_;
            model::SensorModel<T>* sensor_model_;

            vct::State<T> state_belief_;
            vct::State<T> state_obser_;

            mtx::Matrix<T> covar_belief_;
            mtx::Matrix<T> covar_obser_;
            mtx::Matrix<T> covar_cross_;

            mtx::Matrix<T> covar_obser_inv_;
            mtx::Matrix<T> covar_cholesky_;

            sigma::SigmaPoints<T> sigma_belief_;
            sigma::SigmaPoints<T> sigma_predict_;
            sigma::SigmaPoints<T> sigma_obser_;

            vct::Weights<T> mean_weight_;
            vct::Weights<T> covar_weight_;

            mtx::Matrix<T> noise_r_;
            mtx::Matrix<T> noise_q_;
            mtx::Matrix<T> kalman_gain_;

            T lambda_;
            T gamma_;

            T alpha_;
            T beta_;
            T kappa_;

            int vars_;
            int measures_;
            int points_;

            vct::State<T> place_holder_;

    };

}


#endif // UKF_H_

