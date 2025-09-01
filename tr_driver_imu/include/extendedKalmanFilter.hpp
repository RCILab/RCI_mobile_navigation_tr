#include <iostream>
#include <Eigen/Dense> // For matrix operations

class ExtendedKalmanFilter
{
public:
    // Constructor to initialize the filter state and parameters
    ExtendedKalmanFilter(double process_noise_cov = 1e-3, double measurement_noise_cov = 1e-2)
    {
        // Initialize state () and covariance
        state_ = 0.0;       // We only estimate velocity in 1D
        covariance_ = 0.01; // Small initial uncertainty

        // Set process and measurement noise covariance matrices
        Q_ = process_noise_cov;     // Process noise covariance (scalar since it's 1D)
        R_ = measurement_noise_cov; // Measurement noise covariance (scalar since it's 1D)

        H_ = 1.0; // Measurement model matrix (just maps the velocity part of the state)

        I_ = 1.0; // Identity matrix for updates (scalar in 1D)
    }

    // Prediction step (predicts the next state based on the motion model)
    void predict(double dt)
    {
        // Predict state (since we're only modeling velocity, no change in state, i.e., constant velocity)
        // State transition matrix A is simply 1 for velocity

        // Predict covariance
        covariance_ = covariance_ + Q_; // Add process noise to the covariance
    }

    // Update step (updates the state based on the new measurement)
    void update(double measured_velocity)
    {
        // Measurement residual (innovation)
        double y = measured_velocity - H_ * state_;

        // Kalman gain
        double S = H_ * covariance_ * H_ + R_;
        double K = covariance_ * H_ / S;

        // Update state
        state_ = state_ + K * y;

        // Update covariance
        covariance_ = (I_ - K * H_) * covariance_;
    }

    // Get the current filtered velocity
    double getFiltered() const
    {
        return state_; // Return the velocity component of the state
    }

private:
    double state_;      // State (velocity)
    double covariance_; // Covariance matrix for state (1D scalar)
    double Q_;          // Process noise covariance (1D scalar)
    double R_;          // Measurement noise covariance (1D scalar)
    double H_;          // Measurement matrix (1D scalar)
    double I_;          // Identity matrix (1D scalar)
};