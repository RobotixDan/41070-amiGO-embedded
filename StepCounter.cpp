/*
 * AMIGO- Self-test and calibration routines
 * Author:Chris El Hachem
 * Edits made by: Daniya Syed

 * Team notes:
 *  for accelerometer processing
 * CHANGED (Daniya 02/06/2026):
 * - Pace tracking adjusted 3000 -> 1500
 * 
 *
 */

#include "StepCounter.h"
StepCounter::StepCounter(int X_PIN, int Y_PIN, int Z_PIN, int ST_PIN) :
  _ADC_PINS({X_PIN, Y_PIN, Z_PIN}),
  _ST_PIN(ST_PIN),

  _steps(0),
  _pace(Pace::STILL),

  _stepIndex(0),
  _stepCount(0),
  _lastStepTime(0),

  _defaultFactors({0.002326, 0.002326, 0.002326}),
  _newFactors({0.002326, 0.002326, 0.002326}),

  _defaultOffsets({0, 0, 0}),
  _newOffsets({0, 0, 0}),

  _a({0,        -1.1429805,  0.4128016}),
  _b({0.06745527, 0.13491055, 0.06745527}),

  _cycle(0),
  _CYCLES_PER_THRESHOLD(75),

  _accel(0),
  _lastAccel(0),
  _accelMax(0),
  _accelMin(1200),
  _threshold(0),
  _range(0),

  _dominantAxis(0)

{
  pinMode(ST_PIN, OUTPUT);
  digitalWrite(ST_PIN, HIGH); // CHANGED: was LOW. HIGH keeps MOSFET off = ST pin low = normal mode by default

  resetFactors();
  resetOffsets();

  for (int channel = 0; channel < 3; channel++) {
    for (int i = 0; i < 3; i++) {
      _adcBuffer[channel][i] = 0.0;
      _filterBuffer[channel][i] = 0.0;
    }
    _axisMax[channel] = 0;
    _axisMin[channel] = 1200;

    _axisThreshold[channel] = 0;
    _axisRange[channel] = 0;

    _lastFiltered[channel] = 0;
  }

  for (int i = 0; i < STEP_HISTORY_SIZE; i++) {
    _stepTimes[i] = 0;  
  }
}

int StepCounter::getSteps() {
  return _steps;
}

void StepCounter::setSteps(int steps) {
  _steps = steps;
}

Pace StepCounter::getPace() {
  return _pace;
}

double StepCounter::getCadence() {
  if (_stepCount < 2) {
    return 0;
  }

  int newest =
    (_stepIndex - 1 + STEP_HISTORY_SIZE)
    % STEP_HISTORY_SIZE;

  int oldest =
    (_stepIndex - _stepCount + STEP_HISTORY_SIZE)
    % STEP_HISTORY_SIZE;

  unsigned long dt = _stepTimes[newest] - _stepTimes[oldest];

  if (dt == 0) {
    return 0;
  }

  double cadence = 60000.0 * (_stepCount - 1) / dt;

  return cadence;
}

void StepCounter::_updatePace() {

  double cadence = getCadence();

  // Hysteresis thresholds
  switch (_pace) {
    case Pace::STILL:
      if (cadence > 40) {
        _pace = Pace::WALKING;
      }
      break;
    case Pace::WALKING:
      if (cadence < 20) {
        _pace = Pace::STILL;
      }
      else if (cadence > 145) {
        _pace = Pace::RUNNING;
      }
      break;
    case Pace::RUNNING:
      if (cadence < 130) {
        _pace = Pace::WALKING;
      }
      break;
  }
}

// CHANGED: Return type changed from bool to SelfTestResult.
// Logic inverted due to PMOS inverter (Q1) on PCB - see header comment.
// Now stores and returns initial/final ADC readings so display can show
// measured vs expected deltas per axis.
SelfTestResult StepCounter::selfTest() {
  SelfTestResult result;

  // CHANGED: was digitalWrite(_ST_PIN, LOW)
  // HIGH = MOSFET off = ST pin low = normal mode (no self-test)
  digitalWrite(_ST_PIN, HIGH);
  delay(50);
  result.initialReadings[0] = analogRead(_ADC_PINS[0]);
  result.initialReadings[1] = analogRead(_ADC_PINS[1]);
  result.initialReadings[2] = analogRead(_ADC_PINS[2]);

  // CHANGED: was digitalWrite(_ST_PIN, HIGH)
  // LOW = MOSFET on = ST pin high = self-test active
  digitalWrite(_ST_PIN, LOW);
  delay(50);
  result.finalReadings[0] = analogRead(_ADC_PINS[0]);
  result.finalReadings[1] = analogRead(_ADC_PINS[1]);
  result.finalReadings[2] = analogRead(_ADC_PINS[2]);

  // CHANGED: was digitalWrite(_ST_PIN, LOW)
  // Back to normal mode
  digitalWrite(_ST_PIN, HIGH);

  // Expected ADC deltas for 3.3V supply (from ADXL335 datasheet)
  int expX = (int)((-338)*(4095/3300.)); // ~-419 counts
  int expY = (int)((338)*(4095/3300.));  // ~419 counts
  int expZ = (int)((661)*(4095/3300.));  // ~820 counts

  int dX = result.finalReadings[0] - result.initialReadings[0];
  int dY = result.finalReadings[1] - result.initialReadings[1];
  int dZ = result.finalReadings[2] - result.initialReadings[2];

  result.passed = (
    abs(dX - expX) < 0.2*abs(expX)
    && abs(dY - expY) < 0.2*abs(expY)
    && abs(dZ - expZ) < 0.2*abs(expZ)
  );

  if (result.passed) {
    _newFactors[0] = (-338)/(330.0*dX);
    _newFactors[1] = (338)/(330.0*dY);
    _newFactors[2] = (661)/(330.0*dZ);
  }

  return result;
}

CalibrationResult StepCounter::calibrate(Orientation orientation) {
  
  CalibrationResult result;
  result.passedVarianceTest = false;
  result.passedMeanTest = false;

  delay(1000);
  int cacheSize = 30;

  double readings[3][cacheSize];
  for (int i = 0; i < cacheSize; i++) {
    for (int channel = 0; channel < 3; channel++) {
      readings[channel][i] = _adcToG(channel);
    }
    delay(20);
  }

  for (int channel = 0; channel < 3; channel++) {
    result.means[channel] = 0;
    for (int i = 0; i < cacheSize; i++) {
      result.means[channel] += readings[channel][i];
    }
    result.means[channel] /= cacheSize;
  }

  for (int channel = 0; channel < 3; channel++) {
    result.variances[channel] = 0;
    for (int i = 0; i < cacheSize; i++) {
      double error = readings[channel][i] - result.means[channel];
      result.variances[channel] += error * error;
    }
    result.variances[channel] /= (cacheSize - 1);
  }

  result.passedVarianceTest = (
    result.variances[0] < 0.07
    && result.variances[1] < 0.07
    && result.variances[2] < 0.07
  );

  // CHANGED (Daniya 01/06/2026): Replaced generic expectedMeans check with
  // per-axis dominant check. Strict tolerance (0.35g) on the axis that should
  // read ±1g, lenient tolerance (0.5g) on the other two axes which will have
  // some residual gravity component when tilted.
  // Chris's original generic check commented out below.

  // double expectedMeans[3] = {0, 0, 0};
  // switch (orientation) {
  //   case Orientation::FRONT:  expectedMeans[1] = -1; break;
  //   case Orientation::BACK:   expectedMeans[1] =  1; break;
  //   case Orientation::TOP:    expectedMeans[2] =  1; break;
  //   case Orientation::BOTTOM: expectedMeans[2] = -1; break;
  //   case Orientation::LEFT:   expectedMeans[0] = -1; break;
  //   case Orientation::RIGHT:  expectedMeans[0] =  1; break;
  // }
  // result.passedMeanTest = (
  //   abs(result.means[0] - expectedMeans[0]) < 0.35
  //   && abs(result.means[1] - expectedMeans[1]) < 0.35
  //   && abs(result.means[2] - expectedMeans[2]) < 0.35
  // );

  switch (orientation) {
    case Orientation::RIGHT:
      result.passedMeanTest = (abs(result.means[0] - 1.0) < 0.35 && abs(result.means[1]) < 0.5 && abs(result.means[2]) < 0.5);
      break;
    case Orientation::LEFT:
      result.passedMeanTest = (abs(result.means[0] + 1.0) < 0.35 && abs(result.means[1]) < 0.5 && abs(result.means[2]) < 0.5);
      break;
    case Orientation::BACK:
      result.passedMeanTest = (abs(result.means[0]) < 0.5 && abs(result.means[1] - 1.0) < 0.35 && abs(result.means[2]) < 0.5);
      break;
    case Orientation::FRONT:
      result.passedMeanTest = (abs(result.means[0]) < 0.5 && abs(result.means[1] + 1.0) < 0.35 && abs(result.means[2]) < 0.5);
      break;
    case Orientation::TOP:
      result.passedMeanTest = (abs(result.means[0]) < 0.5 && abs(result.means[1]) < 0.5 && abs(result.means[2] - 1.0) < 0.35);
      break;
    case Orientation::BOTTOM:
      result.passedMeanTest = (abs(result.means[0]) < 0.5 && abs(result.means[1]) < 0.5 && abs(result.means[2] + 1.0) < 0.35);
      break;
  }

  // Store offsets if both tests passed
  if (result.passedMeanTest && result.passedVarianceTest) {
    double expectedMeans[3] = {0, 0, 0};
    switch (orientation) {
      case Orientation::FRONT:  expectedMeans[1] = -1; break;
      case Orientation::BACK:   expectedMeans[1] =  1; break;
      case Orientation::TOP:    expectedMeans[2] =  1; break;
      case Orientation::BOTTOM: expectedMeans[2] = -1; break;
      case Orientation::LEFT:   expectedMeans[0] = -1; break;
      case Orientation::RIGHT:  expectedMeans[0] =  1; break;
    }
    _newOffsets[0] = - result.means[0] + expectedMeans[0];
    _newOffsets[1] = - result.means[1] + expectedMeans[1];
    _newOffsets[2] = - result.means[2] + expectedMeans[2];
  }

  return result;
}

void StepCounter::readADC() {
  _cycle++;
  for (int channel = 0; channel < 3; channel++) {
    _adcBuffer[channel][2] = _adcBuffer[channel][1];
    _adcBuffer[channel][1] = _adcBuffer[channel][0];
    _filterBuffer[channel][2] = _filterBuffer[channel][1];
    _filterBuffer[channel][1] = _filterBuffer[channel][0];

    _adcBuffer[channel][0] = _adcToG(channel);

    double x0 = _adcBuffer[channel][0];
    double x1 = _adcBuffer[channel][1];
    double x2 = _adcBuffer[channel][2];

    double y1 = _filterBuffer[channel][1];
    double y2 = _filterBuffer[channel][2];

    _filterBuffer[channel][0] =
      _b[0] * x0
      + _b[1] * x1
      + _b[2] * x2
      - _a[1] * y1
      - _a[2] * y2;
  }

  if (_cycle >= _CYCLES_PER_THRESHOLD) {
    _cycle = 0;

    for (int ch = 0; ch < 3; ch++) {
      _axisThreshold[ch] = 0.5 * (_axisMax[ch] + _axisMin[ch]);
      _axisRange[ch] = _axisMax[ch] - _axisMin[ch];
      _axisMax[ch] = -4;
      _axisMin[ch] = 4;
    }

    // Select dominant walking axis
    _dominantAxis = 0;
    for (int ch = 1; ch < 3; ch++) {
      if (_axisRange[ch] > _axisRange[_dominantAxis]) {
        _dominantAxis = ch;
      }
    }
  }

  // Track min/max
  for (int ch = 0; ch < 3; ch++) {
    double value = _filterBuffer[ch][0];

    if (value > _axisMax[ch]) {
      _axisMax[ch] = value;
    }

    if (value < _axisMin[ch]) {
      _axisMin[ch] = value;
    }
  }

  // Step detection on dominant axis
  int ch = _dominantAxis;

  double current = _filterBuffer[ch][0];
  double previous = _lastFiltered[ch];

  bool crossingThreshold = (current <= _axisThreshold[ch]) && (previous > _axisThreshold[ch]);

  bool enoughMovement = (_axisRange[ch] > 0.2);

  unsigned long now = millis();
  bool refractoryExpired = (now - _lastStepTime > 250);

  if (
    crossingThreshold
    && enoughMovement
    && refractoryExpired
  ) {
    _steps++;
    _lastStepTime = now;

    // Store timestamp
    _stepTimes[_stepIndex] = now;

    _stepIndex = (_stepIndex + 1) % STEP_HISTORY_SIZE;

    if (_stepCount < STEP_HISTORY_SIZE) {
      _stepCount++;
    }

    _updatePace();
  }

  // Detect inactivity
  if (now - _lastStepTime > 1500) { /////changed from 3000
    _pace = Pace::STILL;
    _stepCount = 0;
    _stepIndex = 0;
  }

  for (int ch = 0; ch < 3; ch++) {
    _lastFiltered[ch] = _filterBuffer[ch][0];
  }
  
  _lastAccel = _accel;
}

void StepCounter::resetFactors() {
  for (int i = 0; i < 3; i++) {
    _factors[i] = _defaultFactors[i];
  }
}

void StepCounter::setNewFactors() {
  for (int i = 0; i < 3; i++) {
    _factors[i] = _newFactors[i];
  }
}

void StepCounter::resetOffsets() {
  for (int i = 0; i < 3; i++) {
    _offsets[i] = _defaultOffsets[i];
  }
}

void StepCounter::setNewOffsets() {
  for (int i = 0; i < 3; i++) {
    _offsets[i] = _newOffsets[i];
  }
}

double StepCounter::getAccel() {
  Serial.print("X:");
  Serial.print(_filterBuffer[0][0]);
  Serial.print(",");
  Serial.print("Y:");
  Serial.print(_filterBuffer[1][0]);
  Serial.print(",");
  Serial.print("Z:");
  Serial.print(_filterBuffer[2][0]);
  Serial.print(",");
  return sqrt(_filterBuffer[0][0] * _filterBuffer[0][0] 
    + _filterBuffer[1][0] * _filterBuffer[1][0]
    + _filterBuffer[2][0] * _filterBuffer[2][0]);
}

double StepCounter::_adcToG(int channel) {
  return _factors[channel] * ((int)analogRead(_ADC_PINS[channel]) - 2048) + _offsets[channel];
}
