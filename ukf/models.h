#ifndef MODELS_H_
#define MODELS_H_
#include "sigma.h"
#include "utilities.h"


namespace model {


    // base process model calss
    template<class T>
    class ProcessModel {

        public:
            ProcessModel() {

            }

            virtual ~ProcessModel() {

            }

            virtual double calculate(
                vct::State<T>& x_tf,
                vct::State<T>& x_ti,
                vct::State<T>& u_tf,
                vct::State<T>& u_ti) = 0;

            virtual double probability(
                T a, T b) = 0;

        protected:
            double distance(
                T x_f, T x_i,
                T y_f, T y_i)
            {
                return std::sqrt(std::pow(x_f-x_i,2)+std::pow(y_f-y_i,2));
            }

            double variance(
                T a1, T d1,
                T a2, T d2)
            {
                return
                    a1*std::pow(d1,2)+
                    a2*std::pow(d2,2);
            }

            double variance(
                T a1, T d1,
                T a2, T d2,
                T d3)
            {
                return
                    a1*std::pow(d1,2)+
                    a2*std::pow(d2,2)+
                    a2*std::pow(d3,2);
            }

        private:

    };


    // base sensor class
    template<class T>
    class SensorModel {

        public:
            SensorModel() {

            }

            virtual ~SensorModel() {

            }

            virtual void calculate(
                sigma::SigmaPoints<T>& observation,
                sigma::SigmaPoints<T>& prediction) = 0;

        private:

    };


}


#endif // MODELS_H_
