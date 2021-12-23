#include "main.hpp"
#include <math.h>


KalmanFilter::KalmanFilter(double displacement, double velocity, double acceleration) {
    previousState_M = Vector2d(displacement, velocity);
    previousErrorCovariance_M = calculateInitialErrorCovariance();
}

Vector2d KalmanFilter::getEstimate(double displacement, double velocity, double acceleration) {
    // From actual measurements
    Vector2d measurementInput_M = getMeasurementInput(displacement, velocity);  // Y_k
    Matrix2d measurementCovar_M = getMeasurementCovariance();                   // R
    std::cout << "Measurement: "; std::cout << measurementInput_M.x();
    // From model predictions
    Vector2d predictedInput_M = calculatePrediction(acceleration);              // X_(k_p)
    Matrix2d predictedCovar_M = calculateErrorCovariance();                     // P_(k_p)
    std::cout << " | Prediction: "; std::cout << predictedInput_M.x();

    // Calculate kalman gain (K)
    Matrix2d kalmanGain_M = calculateKalmanGain(predictedCovar_M, measurementCovar_M);
    // Calculate kalman estimate (X_k)
    Vector2d kalmanEstimate_M = calculateNewState(predictedInput_M, kalmanGain_M, measurementInput_M);
    std::cout << " | Kalman Estimate: "; std::cout << kalmanEstimate_M.x() << std::endl;

    // Transition parameters P_(k-1) + P_k AND X_(k-1) = X_k
    const Matrix2d identity_M = (Matrix2d() << 1, 0, 0, 1).finished();
    previousState_M = kalmanEstimate_M;
    previousErrorCovariance_M = (identity_M - kalmanGain_M) * predictedCovar_M; // (I - K) * P_(k_p)

    return kalmanEstimate_M;
}

/** New, predicted state
 * Of form: X_(k_p) = AX_(k-1) + Bu_k + w_k
 * Where w is bias
 * Where u_k is acceleration
 * Where B is transformation matrix for acceleration
 * Where X_(k-1) is previous displacement
 * Where A is transformation matrix for state vector
 * @return
 */
Vector2d KalmanFilter::calculatePrediction(double acceleration) {
    // 1/2 * t^2
    const Vector2d transformAcceleration_M = Vector2d(0.5 * pow(TIME_DELTA, 2), TIME_DELTA);
    const Vector2d error_M = Vector2d(0, 0);

    return (transform_M * previousState_M) + (transformAcceleration_M * acceleration) + error_M;
}

/** New, predicted state
 * Of form: P_(k_p) = AP_(k-1)A^T + Q_k
 * @return
 */
Matrix2d KalmanFilter::calculateErrorCovariance() {
    // Q_k is assumed to be 0 vector
    return transform_M * previousErrorCovariance_M * transform_M.transpose();
}

/** New, kalman gain
 *
 * @return
 */
Matrix2d KalmanFilter::calculateKalmanGain(Matrix2d predictedCovar_M, Matrix2d measurementCovar_M) {
    const Matrix2d transformCovar_M = (Matrix2d() << 1, 0, 0, 1).finished();

    Matrix2d processesError_M = predictedCovar_M * transformCovar_M;
    Matrix2d totalError_M = processesError_M + measurementCovar_M;
    return processesError_M.array() / totalError_M.array();
}

/** New, kalman state
 * Of form: X_k = X_(k_p) + K[Y_k - HX_(k_p)]
 */
Vector2d KalmanFilter::calculateNewState(Vector2d predictedInput_M, Matrix2d kalmanGain, Vector2d measurementInput_M) {
    const Matrix2d transformCovar_M = (Matrix2d() << 1, 0, 0, 1).finished();

    Vector2d measurementDiff_M = measurementInput_M - (transformCovar_M * predictedInput_M);
    return predictedInput_M + (kalmanGain * measurementDiff_M);
}

/**
 * Of form: Y_k = CX_(k_M) + Z_k
 * Where Z_k is measurement noise
 * @param displacement
 * @param velocity
 * @return
 */
Vector2d KalmanFilter::getMeasurementInput(double displacement, double velocity) {
    const Matrix2d transform_M = (Matrix2d() << 1, 0, 0, 1).finished();
    const Vector2d noise_M = Vector2d(0, 0);
    Vector2d measurement_M = Vector2d(displacement, velocity);
    return (transform_M * measurement_M) + noise_M;
}

/**
 * Of form: P_0
 * @return
 */
const Matrix2d KalmanFilter::calculateInitialErrorCovariance() {
    return (Matrix2d() << pow(IR_PROCESS_ERR, 2), IR_PROCESS_ERR * IMU_PROCESS_ERR, IR_PROCESS_ERR * IMU_PROCESS_ERR, pow(IMU_PROCESS_ERR, 2)).finished();
}

/**
 * Of form: R
 * @return
 */
const Matrix2d KalmanFilter::getMeasurementCovariance() {
    return (Matrix2d() << pow(IR_UNCERTAINTY, 2), IR_UNCERTAINTY * IMU_UNCERTAINTY, IR_UNCERTAINTY * IMU_UNCERTAINTY, pow(IMU_UNCERTAINTY, 2)).finished();
}

int main() {
    KalmanFilter kalmanFilter(100, 10, 0);
    kalmanFilter.getEstimate(112, 10, 0);
    kalmanFilter.getEstimate(123, 10, 0);
    kalmanFilter.getEstimate(131, 10, 2);
    kalmanFilter.getEstimate(173, 12, 5);
}


