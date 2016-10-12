/*
 Controlling a servo position using a potentiometer (variable resistor)
 by Michal Rinott <http://people.interaction-ivrea.it/m.rinott>

 modified on 8 Nov 2013
 by Scott Fitzgerald
 http://www.arduino.cc/en/Tutorial/Knob
*/

#pragma GCC diagnostic error "-Wall"

#include <Servo.h>

typedef unsigned long microsec_t;
typedef float degree_t;
typedef unsigned long millidegree_t;

template <
  microsec_t _pulseCycle,
  microsec_t _pulseWidthMin,
  microsec_t _pulseWidthMax,
  millidegree_t _rotationalRange
> class ServoSpecification
{
public:
  static constexpr microsec_t pulseCycle = _pulseCycle;
  static constexpr microsec_t pulseWidthMin = _pulseWidthMin;
  static constexpr microsec_t pulseWidthMax = _pulseWidthMax;
  static constexpr degree_t rotationalRange = _rotationalRange / 1000.0;
};

class ServoImplementation
{
public:
  virtual float operator=(float amount) = 0;
  virtual operator float() = 0;
  virtual void update() {}
  virtual void disengage() = 0;
};

typedef ServoSpecification<20000, 450, 2450, 180000> HXT900;

template <int pin, typename ServoSpecification>
class ArduinoLibraryServo : public ServoImplementation, ServoSpecification
{
  using S = ServoSpecification;
public:
  using S::pulseCycle;
  using S::pulseWidthMin;
  using S::pulseWidthMax;
  using S::rotationalRange;

  float operator=(float amount)
  {
    if (!servo.attached())
      servo.attach(pin, pulseWidthMin, pulseWidthMax);
    servo.writeMicroseconds(amount * (pulseWidthMax - pulseWidthMin) + pulseWidthMin + 0.5);
    lastWrite = micros();
    return (servo.readMicroseconds() - pulseWidthMin) / float(pulseWidthMax - pulseWidthMin);
  }

  operator float()
  {
    return float(servo.readMicroseconds() - pulseWidthMin) / (pulseWidthMax - pulseWidthMin);
  }
  
  void disengage()
  {
    microsec_t delay = micros() - lastWrite;
  
    if (delay < pulseCycle) {
      delayMicroseconds(pulseCycle - delay);
    }

    servo.detach();
  }
  
private:
  Servo servo;
  microsec_t lastWrite;
};

template <int pin, typename ServoSpecification>
class NoInterruptServo : public ServoImplementation, ServoSpecification
{
  using S = ServoSpecification;
public:
  using S::pulseCycle;
  using S::pulseWidthMin;
  using S::pulseWidthMax;
  using S::rotationalRange;

  NoInterruptServo()
  : engaged(false)
  {
    pinMode(pin, OUTPUT);
    digitalWrite(pin, LOW);
  }
  
  float operator=(float amount)
  {
    if (!engaged) {
      engaged = true;
      disengaging = false;
      pulsing = false;
      cycleStart = micros() - pulseCycle;
    }
    pulseWidth = amount * (pulseWidthMax - pulseWidthMin) + pulseWidthMin + 0.5;
    return (pulseWidth - pulseWidthMin) / float(pulseWidthMax - pulseWidthMin);
  }

  operator float()
  {
    return float(pulseWidth - pulseWidthMin) / (pulseWidthMax - pulseWidthMin);
  }

  void disengage()
  {
    disengaging = true;
  }
  
  void update()
  {
    if (!engaged) return;
  
    if (pulsing) {
      if (micros() - cycleStart >= pulseWidth) {
        digitalWrite(pin, LOW);
        pulsing = false;
        if (disengaging) engaged = false;
      }
    } else { // already pulsed
      microsec_t now = micros();
      if (now - cycleStart >= pulseCycle) {
        digitalWrite(pin, HIGH);
        pulsing = true;
        cycleStart = now;
      }
    }
  }

private:
  microsec_t cycleStart;
  microsec_t pulseWidth;
  bool engaged;
  bool disengaging;
  bool pulsing;
};


template <int pin>
class AnalogInput
{
public:
  AnalogInput()
  {
    pinMode(pin, INPUT);
  }
  operator double()
  {
    return analogRead(pin) / 1023.0;
  }
};

template <unsigned long baud>
class ArduinoLibrarySerial
{
public:
  ArduinoLibrarySerial()
  {
    Serial.begin(baud);
  }

  ArduinoLibrarySerial & operator<<(double v)
  {
    Serial.print(v, 4);
    return *this;
  }

  ArduinoLibrarySerial & operator<<(const char * s)
  {
    Serial.print(s);
    return *this;
  }

  ArduinoLibrarySerial & operator<<(char c)
  {
    Serial.print(c);
    return *this;
  }
};


void setup() {
  ArduinoLibraryServo<9, HXT900> servo;
  AnalogInput<0> potentiometer;
  AnalogInput<3> currentProbe;
  ArduinoLibrarySerial<115200> serial;

  for (;;) {
    serial << potentiometer << ',' << currentProbe << ',' << (servo = potentiometer) << '\n';
    servo.update();
  }
}

void loop() {}

