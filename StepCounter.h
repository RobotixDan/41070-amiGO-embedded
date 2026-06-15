#ifndef StepCounter_h
#define StepCounter_h

#include <Arduino.h>

enum Pace {
  STILL,
  WALKING,
  RUNNING
};

enum Orientation {
  FRONT,
  BACK,
  TOP,
  BOTTOM,
  LEFT,
  RIGHT
};

struct CalibrationResult {
  bool passedVarianceTest;
  bool passedMeanTest;
  double means[3];
  double variances[3];
};

// ADDED (Daniya 26/05/2026): New struct to return actual ADC readings from
// selfTest() so the display can show measured vs expected values per axis.
// Previously selfTest() only returned bool (pass/fail) with no visibility
// into what the actual readings were.
struct SelfTestResult {
  bool passed;
  int initialReadings[3];  // ADC readings with ST pin inactive (normal mode)
  int finalReadings[3];    // ADC readings with ST pin active (self-test mode)
};

class StepCounter {
  public:
    StepCounter(int X_PIN, int Y_PIN, int Z_PIN, int ST_PIN);
    int getSteps();
    void setSteps(int steps);
    Pace getPace();
    double getCadence();
    SelfTestResult selfTest(); // CHANGED: was bool selfTest() - now returns SelfTestResult struct
    CalibrationResult calibrate(Orientation orientation);
    void readADC();

    void resetFactors();
    void setNewFactors();

    void resetOffsets();
    void setNewOffsets();

    double getAccel();

  private:
    const int _ADC_PINS[3];
    const int _ST_PIN;

    int _steps;
    Pace _pace;

    void _updatePace();
    static constexpr int STEP_HISTORY_SIZE = 8;
    unsigned long _stepTimes[STEP_HISTORY_SIZE];
    int _stepIndex;
    int _stepCount;
    unsigned long _lastStepTime;

    const double _defaultFactors[3];
    double _factors[3];
    double _newFactors[3];

    const double _defaultOffsets[3];
    double _offsets[3];
    double _newOffsets[3];

    double _adcBuffer[3][3]; 
    double _filterBuffer[3][3]; 
    const double _a[3];
    const double _b[3];

    int _cycle;
    const int _CYCLES_PER_THRESHOLD;

    double _accel;
    double _lastAccel;
    double _accelMax;
    double _accelMin;
    double _threshold;
    double _range;

    double _axisMax[3];
    double _axisMin[3];
    double _axisThreshold[3];
    double _axisRange[3];
    int _dominantAxis;
    double _lastFiltered[3];

    double _adcToG(int channel);
};

#endif
