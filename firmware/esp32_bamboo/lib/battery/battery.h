#ifndef BATTERY_H
#define BATTERY_H

class Battery{

    public:
        struct Battery_t
        {
            float voltage;
            float current;
            float percentage;
            float capacity;
            float design_capacity;
            bool present;
        };
        void initBattery();
        Battery& readBattery();
        float getVoltage() { return _battery.voltage; }
        float getCurrent() { return _battery.current; }
        float getPercentage() { return _battery.percentage; }
        float getCapacity() { return _battery.capacity; }
        float getDesignCapacity() { return _battery.design_capacity; }
        bool getPresent() { return _battery.present; }
        Battery_t getData() { return _battery; }
        
    private:
        void getBatteryPercentage();
        Battery::Battery_t _battery;
};

#endif
