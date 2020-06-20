#ifndef TIMER_GSA_H
#define TIMER_GSA_H

#include <Arduino.h>

/*Simple timer for automating things,
Stopped timers will always return true for isElapsed
Timers do not reset automatically
*/
class Timer {
    public:
        Timer(unsigned long time); 
        void setDuration(unsigned long duration); 
        bool isElapsed();
        bool isRunning();
        void reset();
        void stop();
        void start();

    private:
        unsigned long _tStart;
        unsigned long _tExpire;
        unsigned long _duration;
        bool _run;
};

class Blinker {
    public:
        Blinker(unsigned long period);
        Blinker(unsigned long period, uint8_t dutyCycle);
        bool isOn();
        bool isOff();
        void setPeriod(unsigned long period);
        void setDutyCycle(uint8_t dutyCycle);
        void reset();
        void stop();
        
    private:
        void update();
        unsigned long _period;
        uint8_t _dc;
        unsigned long _tOff;
        unsigned long _tUpd;
        unsigned long _onDuration;
        unsigned long _tEnd;
        unsigned long _t0;
        bool _run;
        bool _on;
};

#endif