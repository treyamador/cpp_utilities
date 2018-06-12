#ifndef SIGMA_POINTS_H_
#define SIGMA_POINTS_H_
#include <cmath>
#include "utilities.h"

// include for testing
#include <iostream>
#include <iomanip>


namespace sigma {

    template<class T>
    class SigmaPoints {

        public:
            SigmaPoints(int points,int vars) :
                sigma_(nullptr),
                points_(points),
                vars_(vars)
            {
                this->init(points,vars);
            }

            ~SigmaPoints() {
                this->destroy();
            }

            void init(int points,int vars) {
                this->destroy();
                this->points_ = points;
                this->vars_ = vars;
                this->sigma_ = new vct::State<T>[this->points_];
                for (int i = 0; i < this->points_; ++i) {
                    this->sigma_[i] = vct::State<T>(this->vars_);
                }
            }

            void destroy() {
                if (this->sigma_ != nullptr) {
                    delete[] this->sigma_;
                    this->sigma_ = nullptr;
                }
                this->points_ = this->vars_ = 0;
            }

            void generatePoints(
                vct::State<T>& mean,
                mtx::Matrix<T>& cholesky,
                T gamma)
            {
                int i = 0;
                this->sigma_[i++] = mean;
                this->generatePoints(mean,cholesky,gamma,i,this->vars_+1);
                this->generatePoints(mean,cholesky,-gamma,i,this->points_);
            }

            /*

            void generatePoints(
                state::StateVector<T>& mean,
                mtx::Matrix<T>& cholesky,
                T coef, int& i, int bound)
            {
                int row = cholesky.getRows(),
                    col = cholesky.getCols();
                while (i < bound) {
                    for (int j = 0; j < this->vars_; ++j) {
                        this->sigma_[i][j] = mean[j]+coef*cholesky[j*row+(i-1)%col];
                    }
                    ++i;
                }
            }

            */

            void generatePoints(
                vct::State<T>& mean,
                mtx::Matrix<T>& cholesky,
                T coef, int& i, int bound)
            {
                int row = cholesky.rows(),
                    col = cholesky.cols();
                while (i < bound) {
                    for (int j = 0; j < this->vars_; ++j) {
                        //this->sigma_[i](j) = mean(j)+coef*cholesky[j*row+(i-1)%col];
                        this->sigma_[i](j) = mean(j)+coef*cholesky(j,(i-1)%col);
                    }
                    ++i;
                }
            }

            vct::State<T>& operator[](int i) {
                return this->sigma_[i];
            }

            int getNumPoints() {
                return this->points_;
            }

            int getStateSize() {
                return this->vars_;
            }


            /*

            // for debugging purposes
            void print() {
                int num_points = this->points_;
                int vars = this->vars_;
                for (int n = 0; n < num_points; ++n) {
                    std::cout << n+1 << ": ";
                    for (int i = 0; i < vars; ++i) {
                        std::cout << this->sigma_[n][i] << " ";
                    }
                    std::cout << "\n";
                }
                std::cout << "size " << num_points << "\n";
                std::cout << std::endl;
            }

            void precisionPrint() {
                int def_prec = 6,
                    prec = 17;
                std::cout << std::setprecision(prec);
                this->print();
                std::cout << std::setprecision(def_prec);
            }

            */


        private:
            vct::State<T>* sigma_;
            int points_;
            int vars_;

    };

}


#endif // SIGMA_POINTS_H_

