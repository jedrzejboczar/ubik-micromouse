#pragma once

#include "device_driver.h"
#include "stm32f1xx.h"

struct DataToSend {
    void * data;
    size_t size;
};

class UART: public Device<DataToSend, bool> {
    UART_HandleTypeDef *huart;
public:
    UART(UART_HandleTypeDef *huart);

    using RequestData_t = DataToSend;
    using ResponseData_t = bool;

    // prepare the device, this function should panic! on fail
    virtual void configure() override;

    // start device, use given request data, configure interrupt
    virtual bool start_async(const RequestData_t &data) override;
    // (!) there has to be some connection between the interrupt and the RTOS component

    // stop device, clean it up and prepare it for another usage
    virtual void abort() override;

    // set the data from the device for the response
    virtual void prepare_response(ResponseData_t &data) override;
};
