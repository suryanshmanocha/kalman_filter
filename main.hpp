//
// Created by Suryansh Manocha on 21/12/2021.
//

#ifndef KALMAN_FILTER_MAIN_HPP
#define KALMAN_FILTER_MAIN_HPP

#endif //KALMAN_FILTER_MAIN_HPP

#include <iostream>
#include "eigen/Eigen/Dense"

using Eigen::MatrixXd;
using Eigen::Vector2d;

class KalmanFilter {
    public:
        /* Attributes */
        Vector2d initialState_M;
        /* Constructor */
        KalmanFilter(int disp, int acc);
        /* Methods */
        void mult();
    private:
        void multiply();
};