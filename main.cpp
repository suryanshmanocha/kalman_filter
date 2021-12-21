#include "main.hpp"


KalmanFilter::KalmanFilter(int disp, int acc) {
    initialState_M = Vector2d(disp, acc);
}

void KalmanFilter::mult() {
    initialState_M += initialState_M;
}

void KalmanFilter::multiply() {
    initialState_M *= 2;
}

int main() {
    int displacement = 5;
    int acceleration = 2;
    KalmanFilter kalmanFilter(displacement, acceleration);
    kalmanFilter.mult();
    std::cout << kalmanFilter.initialState_M << std::endl;
}


