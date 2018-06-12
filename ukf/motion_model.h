#ifndef MOTION_MODEL_H_
#define MOTION_MODEL_H_
#include <cmath>
#include "models.h"


namespace {

    // TODO these are empirically found values
    //      and will be modified after their
    //      empirical values are tested
    const double alpha_1 = 1.0;
    const double alpha_2 = 1.0;
    const double alpha_3 = 1.0;
    const double alpha_4 = 1.0;

}


namespace model {



    /*

    template<class T>
    class OdometryMotionModel : public ProcessModel<T> {

        public:
            OdometryMotionModel() :
                ProcessModel<T>(),
                delta_i_(state::ParameterVector<T>(this->params_)),
                delta_f_(state::ParameterVector<T>(this->params_))
            {}

            ~OdometryMotionModel() {

            }

            virtual double calculate(
                state::StateVector<T>& x_tf,
                state::StateVector<T>& x_ti,
                state::ControlVector<T>& u_tf,
                state::ControlVector<T>& u_ti)
            {
                this->delta_i_[MOTION::ROT_1] = this->init_rotation(u_tf,u_ti);
                this->delta_i_[MOTION::TRANS] = this->translation(u_tf,u_ti);
                this->delta_i_[MOTION::ROT_2] = this->final_rotation(
                    u_tf,u_ti,this->delta_i_[MOTION::ROT_1]);

                this->delta_f_[MOTION::ROT_1] = this->init_rotation(x_tf,x_ti);
                this->delta_f_[MOTION::TRANS] = this->translation(x_tf,x_ti);
                this->delta_f_[MOTION::ROT_2] = this->final_rotation(
                    x_tf,x_ti,this->delta_f_[MOTION::ROT_1]);

                double p1 = this->probability(
                    this->delta_i_[MOTION::ROT_1]-this->delta_f_[MOTION::ROT_1],
                    this->variance(
                        alpha_1,this->delta_f_[MOTION::ROT_1],
                        alpha_2,this->delta_f_[MOTION::TRANS]));

                double p2 = this->probability(
                    this->delta_i_[MOTION::TRANS]-this->delta_f_[MOTION::TRANS],
                    this->variance(
                        alpha_3,this->delta_f_[MOTION::TRANS],
                        alpha_4,this->delta_f_[MOTION::ROT_1],
                        this->delta_f_[MOTION::ROT_2]));

                double p3 = this->probability(
                    this->delta_i_[MOTION::ROT_2]-this->delta_f_[MOTION::ROT_2],
                    this->variance(
                        alpha_1,this->delta_f_[MOTION::ROT_2],
                        alpha_2,this->delta_f_[MOTION::TRANS]));

                return p1*p2*p3;

            }

        private:
            double init_rotation(
                state::StateVector<T>& x_tf,
                state::StateVector<T>& x_ti)
            {
                double y = x_tf[POSE::Y] - x_ti[POSE::Y];
                double x = x_tf[POSE::X] - x_ti[POSE::X];
                double theta = x_ti[POSE::A];
                return atan2(y,x) - theta;
            }

            double translation(
                state::StateVector<T>& x_tf,
                state::StateVector<T>& x_ti)
            {
                return this->distance(
                    x_tf[POSE::X],x_ti[POSE::X],
                    x_tf[POSE::Y],x_ti[POSE::Y]);
            }

            double final_rotation(
                state::StateVector<T>& x_tf,
                state::StateVector<T>& x_ti,
                T rotation_1)
            {
                return x_tf[POSE::A] - x_ti[POSE::A] - rotation_1;
            }

            virtual double probability(
                T a, T b)
            {
                double norm = 1.0/std::sqrt(2*M_PI*b);
                double expon = std::exp(-std::pow(a,2)/(2*b));
                return norm*expon;
            }

            enum MOTION {
                ROT_1 = 0,
                TRANS = 1,
                ROT_2 = 2
            };

            enum POSE {
                X = 0,
                Y = 1,
                A = 2
            };

        private:
            state::ParameterVector<T> delta_i_;
            state::ParameterVector<T> delta_f_;
            const int params_ = 3;

    };

    */

    // a linear motion model created just for testing purposes
    // not to be used in actual implementation
    template<class T>
    class SimpleMotionModel : public ProcessModel<T> {

        public:
            SimpleMotionModel() :
                ProcessModel<T>()
            {}

            ~SimpleMotionModel() {

            }

            virtual double calculate(
                vct::State<T>& x_tf,
                vct::State<T>& x_ti,
                vct::State<T>& u_tf,
                vct::State<T>& u_ti)
            {
                // motion model created just for testing purposes
                // assumes state and control vector same dimension
                //int vars = x_ti.getVars();
                //for (int i = 0; i < vars; ++i)
                //    x_tf[i] = x_ti[i] + u_tf[i];
                x_tf = x_ti;
                return 1.0;
            }

            virtual double probability(
                T a, T b)
            {
                return 1.0;
            }

        private:

    };


    // assumes global position is same as control
    template<class T>
    class SimpleVelocityModel : public ProcessModel<T> {

        public:
            SimpleVelocityModel() :
                ProcessModel<T>()
            {}

            ~SimpleVelocityModel() {

            }


            //  state vector assumed
            //      x1 == x
            //      x2 == y
            //      x3 == vx
            //      x4 == xy

            //  control vector assumed
            //      u1 == delta x
            //      u2 == delta y
            //      u3 == delta t
            virtual double calculate(
                vct::State<T>& x_tf,
                vct::State<T>& x_ti,
                vct::State<T>& u_tf,
                vct::State<T>& u_ti)
            {
                //T delta_x = u_tf[0] - u_ti[0];
                //T delta_y = u_tf[1] - u_ti[1];
                //T delta_t = u_tf[2] - u_ti[2];

                T delta_x = u_tf(0);
                T delta_y = u_tf(1);
                T delta_t = u_tf(2);

                T vx = delta_x / delta_t;
                T vy = delta_y / delta_t;

                x_tf(0) = x_ti(0) + delta_x;
                x_tf(1) = x_ti(1) + delta_y;
                x_tf(2) = x_ti(2);
                x_tf(3) = x_ti(3);

                return 1.0;
            }

            virtual double probability(T a, T b) {
                return 1.0;
            }


        private:

    };

}


#endif // MOTION_MODEL_H_

