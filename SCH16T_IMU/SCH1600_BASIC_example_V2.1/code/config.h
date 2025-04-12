/**

  Configuration file for project.

*/

#ifndef CONFIG_H
#define CONFIG_H


#define AVG_FACTOR  10      // SCH1 sample averaging

#define FILTER_RATE         30.0f       // Hz, LPF1 Nominal Cut-off Frequency (-3dB).
#define FILTER_ACC12        30.0f
#define FILTER_ACC3         30.0f
#define SENSITIVITY_RATE1   1600.0f     // LSB / dps, DYN1 Nominal Sensitivity for 20 bit data.
#define SENSITIVITY_RATE2   1600.0f
#define SENSITIVITY_ACC1    3200.0f     // LSB / m/s2, DYN1 Nominal Sensitivity for 20 bit data.
#define SENSITIVITY_ACC2    3200.0f
#define SENSITIVITY_ACC3    3200.0f     // LSB / m/s2, DYN1 Nominal Sensitivity for 20 bit data.
#define DECIMATION_RATE     32          // DEC5, Output sample rate decimation.
#define DECIMATION_ACC      32

#endif // CONFIG_H
