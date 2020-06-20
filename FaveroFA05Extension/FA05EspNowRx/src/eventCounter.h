#include <Arduino.h>
#include <queue>



class eventCounter {
    public:
        eventCounter();
        eventCounter(unsigned long timeWindow);
        void addItem();
        void reset();        
        void setWindow();
        uint16_t getCount();
        
    private:        
        std::queue<unsigned long> _queue;
        void purgeQueue();
        unsigned long _window=30*1000; //30s windows
};