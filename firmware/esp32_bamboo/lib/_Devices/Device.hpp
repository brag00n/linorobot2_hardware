#ifndef DEVICE_H
#define DEVICE_H

class Device {
public:
    struct Device_t{
        long id;
        const char* name;

    } deviceData;
    struct DeviceError_t{
        int error_code;
        const char* error_message;
    } error;
    Device() = default;
    virtual ~Device() = default;

    virtual void init(const char* name) { deviceData.name = name; };
    virtual void read() {  };
    virtual void write() {  };
    virtual Device_t* getData()  { return &deviceData; };
    virtual bool isReady()  { return true; };
    virtual bool isError()  { return error.error_code != 0; };
    virtual DeviceError_t getError()  { return error; };
protected:
    virtual void setData(Device::Device_t *pData) { deviceData = *pData; };
    virtual void setError(int errorCode, const char* errorMessage) { error = {errorCode, errorMessage}; };
};
#endif // DEVICE_H
