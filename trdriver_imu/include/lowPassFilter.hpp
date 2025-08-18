#include <iostream>
#include <cmath> // for any math functions like sin, cos if needed

class LowPassFilter
{
public:
    // Constructor to initialize the filter state and parameters
    LowPassFilter(double natural_frequency = 15.0, double damping_factor = 0.707)
        : wn(natural_frequency), c(damping_factor)
    {
        filter_data[0] = filter_data[1] = filter_data[2] = 0.0;
        ue[0] = ue[1] = ue[2] = 0.0;
    }

    // Method to apply the filter to new data
    double applyLPF(double raw_data, double sampling_time)
    {
        // Calculate parameters
        double D = wn * sampling_time;
        double P1 = D * D + 4.0 * c * D;
        double P2 = 2.0 * D * D;
        double P3 = D * D - 4.0 * c * D;

        // Phase detector
        ue[0] = raw_data - filter_data[0];

        // PLL (low-pass filter) calculation
        filter_data[0] = (8.0 * filter_data[1] - 4.0 * filter_data[2] + P1 * ue[0] + P2 * ue[1] + P3 * ue[2]) / 4.0;

        // Shift the arrays for next iteration
        ue[2] = ue[1];
        ue[1] = ue[0];
        filter_data[2] = filter_data[1];
        filter_data[1] = filter_data[0];

        return filter_data[0];
    }

private:
    double filter_data[3]; // Stores previous filtered values
    double ue[3];          // Error values used in the phase detector
    double wn;             // Natural frequency (Hz)
    double c;              // Damping factor
};