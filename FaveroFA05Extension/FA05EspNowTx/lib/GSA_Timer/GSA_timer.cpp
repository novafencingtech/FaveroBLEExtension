
#include "GSA_timer.h"

Timer::Timer(unsigned long time)
{
    _duration = time;
    this->reset();
};

void Timer::setDuration(unsigned long duration)
{
    _duration = duration;
    _tExpire = _tStart + _duration;
}
bool Timer::isElapsed()
{
    if (_run==false) {
        return true;
    }
    return (millis() > _tExpire);
}
void Timer::reset()
{
    _run = true;
    _tStart = millis();
    _tExpire = _tStart + _duration;
}
void Timer::stop()
{
    _run = false;
}
void Timer::start()
{
    _run = true;
};
bool Timer::isRunning()
{
    return _run;
}

/*
        Blinker(unsigned long period);
        Blinker(unsigned long period, uint8_t dutyCycle);
        bool isOn();
        bool isOff();
        void setPeriod(unsigned long period);
        void setDutyCycle(uint8_t dutyCycle);
        void reset();
        void stop();

        unsigned long _period;
        uint8_t _dc;
        unsigned long _tOff;
        unsigned long _onDuration;
        unsigned long _tEnd;
        unsigned long _t0;
        bool _run;

        */

Blinker::Blinker(unsigned long period)
{
    Blinker(period, 50);
}

Blinker::Blinker(unsigned long period, uint8_t duty_cycle)
{
    _dc=duty_cycle;
    _period = period;
    _onDuration = (_period * _dc) / 100;
    _t0 = millis();
    _tOff = _t0 + _onDuration;
    _tEnd = _t0 + _period;
    _run = true;
    _on = true;
}

void Blinker::update()
{
    _tUpd = millis();
    while (_tUpd >= _tEnd)
    {
        _t0 = _tEnd;
        _tEnd = _t0 + _period;
        _tOff = _t0 + _onDuration;
    }
    if (_run)
    {
        _on = (_tUpd <= _tOff);
    }
    else
    {
        _on = false;
    }
}

bool Blinker::isOn()
{
    this->update();
    return (_on);
}

bool Blinker::isOff()
{
    this->update();
    return (!_on);
}

void Blinker::setPeriod(unsigned long period) {
    _period=period;
    _t0=millis();

    this->update();
}

void Blinker::setDutyCycle(uint8_t dutyCycle) {
    _dc=dutyCycle;
    _onDuration = (_period * _dc) / 100;
    this->reset();    
}

void Blinker::reset() {
    _run=true;
    _t0=millis();
    _tEnd = _t0 + _period;
    _tOff = _t0 + _onDuration;
    this->update();
}
void Blinker::stop() {
    _run=false;
    this->update();
}



