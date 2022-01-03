//
// Created by Suryansh Manocha on 21/12/2021.
//

#ifndef KALMAN_FILTER_MAIN_HPP
#define KALMAN_FILTER_MAIN_HPP


#include <iostream>
#include <fstream>
#include "eigen/Eigen/Dense"

using Eigen::Matrix2d;
using Eigen::Vector2d;
using Eigen::Vector3d;

static constexpr double IR_PROCESS_ERR = 2;
static constexpr double IMU_PROCESS_ERR = 2;

static constexpr double IR_UNCERTAINTY = 10;
static constexpr double IMU_UNCERTAINTY = 5;

static constexpr double TIME_DELTA = 1.0;

class KalmanFilter {
    public:
        /* Attributes */
        Vector2d previousState_M;
        Matrix2d previousErrorCovariance_M;
        std::ofstream myfile;
        int iteration;
        /* Constructor */
        KalmanFilter(double displacement, double velocity);
        /* Methods */
        Vector2d getEstimate(double displacement, double velocity, double acceleration);
    private:
        /* Attributes */
        const Matrix2d transform_M = (Matrix2d() << 1, TIME_DELTA, 0, 1).finished();
        /* Methods */
        Vector2d calculatePrediction(double acceleration);
        Matrix2d calculateErrorCovariance();
        Matrix2d calculateKalmanGain(Matrix2d predictedCovar_M, Matrix2d measurementCovar_M);
        Vector2d calculateNewState(Vector2d predictedInput_M, Matrix2d kalmanGain, Vector2d measurementInput_M);
        Vector2d getMeasurementInput(double displacement, double velocity);
        Matrix2d calculateInitialErrorCovariance();
        Matrix2d getMeasurementCovariance();
};

#endif //KALMAN_FILTER_MAIN_HPP