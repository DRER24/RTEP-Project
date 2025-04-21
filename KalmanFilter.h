#ifndef KALMAN_FILTER_H
#define KALMAN_FILTER_H

/**
 * @class KalmanFilter
 * @brief Simple implementation of a Kalman filter for 1D data
 * 
 * This class implements a simplified Kalman filter for smoothing
 * measurement data in one dimension.
 */
class KalmanFilter {
public:
    /**
     * @brief Construct a new KalmanFilter object
     * @param process_noise Process noise covariance (Q)
     * @param measurement_noise Measurement noise covariance (R)
     * @param error Initial estimate error covariance (P)
     */
    explicit KalmanFilter(float process_noise = 0.01f, 
                         float measurement_noise = 0.1f, 
                         float error = 1.0f);
    
    /**
     * @brief Update the filter with a new measurement
     * @param measurement The new measurement value
     * @return The new filtered state
     */
    float update(float measurement);
    
    /**
     * @brief Get the current state estimate
     * @return The current state estimate
     */
    float getState() const;
    
    /**
     * @brief Reset the filter state
     * @param state New initial state value
     * @param error New initial error covariance
     */
    void reset(float state = 0.0f, float error = 1.0f);
    
    /**
     * @brief Set the process noise covariance
     * @param noise New process noise value
     */
    void setProcessNoise(float noise);
    
    /**
     * @brief Set the measurement noise covariance
     * @param noise New measurement noise value
     */
    void setMeasurementNoise(float noise);

private:
    float state_;  ///< Current state estimate (X)
    float error_;  ///< Estimate error covariance (P)
    float process_noise_;      ///< Process noise covariance (Q)
    float measurement_noise_;  ///< Measurement noise covariance (R)
};

#endif // KALMAN_FILTER_H
