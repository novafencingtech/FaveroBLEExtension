#include "eventCounter.h"

eventCounter::eventCounter() {
    _window=30*1000;
}

eventCounter::eventCounter(unsigned long timeWindow) {
    _window=timeWindow;
}

void eventCounter::purgeQueue() {
    unsigned long tNow=millis();
    unsigned long front;

    while (!_queue.empty()) {
        front=_queue.front();
        if ((tNow-front)>_window) {
            _queue.pop();
        } else {
            break;
        }
    }
}

void eventCounter::addItem() {
    unsigned long tNow=millis();
    
    purgeQueue();    
    _queue.push(tNow);
}

void eventCounter::reset() {
    while (!_queue.empty()) {
        _queue.pop();
    }    
}

uint16_t eventCounter::getCount() {
    purgeQueue();
    return _queue.size();
}
     