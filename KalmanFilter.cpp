#include "KalmanFilter.h"

KalmanFilter::KalmanFilter(float process_noise, float measurement_noise, float error)
    : state_(0.0f), error_(error), process_noise_(process_noise), measurement_noise_(measurement_noise) {
}

float KalmanFilter::update(float measurement) {
    // Prediction step - project the state ahead
    // (In this simple case with no control input, we just add the process noise)
    error_ = error_ + process_noise_;
    
    // Update step - compute Kalman gain and update state
    float kalman_gain = error_ / (error_ + measurement_noise_);
    state_ = state_ + kalman_gain * (measurement - state_);
    error_ = (1.0f - kalman_gain) * error_;
    
    return state_;
}

float KalmanFilter::getState() const {
    return state_;
}

void KalmanFilter::reset(float state, float error) {
    state_ = state;
    error_ = error;
}

void KalmanFilter::setProcessNoise(float noise) {
    process_noise_ = noise;
}

void KalmanFilter::setMeasurementNoise(float noise) {
    measurement_noise_ = noise;
}
