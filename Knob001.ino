/*
 Controlling a servo position using a potentiometer (variable resistor)
 by Michal Rinott <http://people.interaction-ivrea.it/m.rinott>

 modified on 8 Nov 2013
 by Scott Fitzgerald
 http://www.arduino.cc/en/Tutorial/Knob
*/

#include <Servo.h>

typedef unsigned long microsec_t;
typedef float degree_t;

struct ServoSpecification
{
  microsec_t pulseCycle;
  microsec_t pulseWidthMin;
  microsec_t pulseWidthMax;
  degree_t rotationalRange;
};

class ServoImplementation
{
public:
  virtual float operator=(float amount) = 0;
  virtual operator float() = 0;
  virtual void update();
  virtual void disengage() = 0;
};

void ServoImplementation::update() {}

constexpr ServoSpecification HXT900{20000, 450, 2450, 180};

template <int pin, ServoSpecification const & specification>
class ArduinoLibraryServo : public ServoImplementation
{
public:
  float operator=(float amount)
  {
    if (!servo.attached())
      servo.attach(pin, specification.pulseWidthMin, specification.pulseWidthMax);
    servo.writeMicroseconds(amount * (specification.pulseWidthMax - specification.pulseWidthMin) + specification.pulseWidthMin + 0.5);
    lastWrite = micros();
  }

  operator float()
  {
    return float(servo.readMicroseconds() - specification.pulseWidthMin) / (specification.pulseWidthMax - specification.pulseWidthMin);
  }
  
  void disengage()
  {
    microsec_t delay = micros() - lastWrite;
  
    if (delay < specification.pulseCycle) {
      delayMicroseconds(specification.pulseCycle - delay);
    }

    servo.detach();
  }
  
private:
  Servo servo;
  microsec_t lastWrite;
};

template <int pin, ServoSpecification const & specification>
class NoInterruptServo : public ServoImplementation
{
public:
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
      cycleStart = micros() - specification.pulseCycle;
    }
    pulseWidth = amount * (specification.pulseWidthMax - specification.pulseWidthMin) + specification.pulseWidthMin + 0.5;
  }

  operator float()
  {
    return float(pulseWidth - specification.pulseWidthMin) / (specification.pulseWidthMax - specification.pulseWidthMin);
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
      if (now - cycleStart >= specification.pulseCycle) {
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
  operator float()
  {
    return analogRead(pin) / 1023.0;
  }
};

template <int baud>
class ArduinoLibrarySerial
{
public:
  ArduinoLibrarySerial()
  {
    Serial.begin(baud);
  }

  int printf(const char *format, ...)
  {
    char buffer[1024];
    int done;
    
    va_list arg;
    va_start(arg, format);
    done = vsnprintf(buffer, sizeof(buffer), format, arg);
    va_end(arg);

    Serial.write(buffer);

    return done;
  }
};


NoInterruptServo<9, HXT900> servo;
AnalogInput<0> potentiometer;
AnalogInput<3> currentProbe;
ArduinoLibrarySerial<115200> serial;

void loop() {
  float amount = potentiometer;
  servo = amount;
  serial.printf("%f,%f,%f\n", amount, servo, currentProbe);
}

