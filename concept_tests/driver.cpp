#include "driver.h"


/*

   class UART: Device<int, int> { ... }


   UARTDriverTask = Device + Task


   MyDevice dev(...)
   DeviceDriver driver(dev)
   DeviceTask task(driver)

 */

/******************************************************************************/

// some test code start
#include <iostream>
#include <cstdlib>
#include <unistd.h>

float randfloat() {
    float result = static_cast<float>(std::rand()) / RAND_MAX;
    // std::cout << "  randfloat -> " << result << std::endl;
    return result;
}

bool randbool(float threshold) { // true if below thershold
    bool result = randfloat() < threshold;
    // std::cout << "  randbool -> " << result << std::endl;
    return result;
}
// some test code end

/******************************************************************************/
/*** DUMMY IMPLEMENTATION *****************************************************/
/******************************************************************************/

/*
 * We have to:
 * - define device with Request&Response Data + implementation
 * - define queue type
 *
 */

class DummyDevice: public Device<int, int> {
public:
    void configure() {
        std::cout << "    void configure()" << std::endl;
    }
    bool start_async(const RequestData_t &data) {
        std::cout << "    bool start_async(const RequestData_t &data)";
        bool ok = randbool(.6);
        std::cout << (ok ? " -> [OK]" : " -> [ERROR]") << std::endl;
        return ok;
    }
    // (!) there has to be some connection between the interrupt and the RTOS component
    void abort() {
        std::cout << "    void abort()" << std::endl;
    }
    void prepare_response(ResponseData_t &data) {
        std::cout << "    void prepare_response(ResponseData_t &data)" << std::endl;
    }
};

// this we should get from our RTOS
typedef int* DymmyRTOSFrameworkQueue;

class DummyAsyncInterface: public AsyncInterface<DymmyRTOSFrameworkQueue> {
public:
    using Queue_t = DymmyRTOSFrameworkQueue;

    // wait for queue with some timeout, store to given reference
    bool receive_request(void *into_buffer) {
        std::cout << "    bool receive_request(Request_t &request)";
        bool ok = randbool(.4);
        std::cout << (ok ? " -> [OK]" : " -> [ERROR]") << std::endl;
        return ok;
    }

    // asynchronious wait for semaphore/task notification/whatever...
    bool wait_for_isr() {
        int msecs = 1 + randfloat() * 1000;
        std::cout << "    bool wait() - start (secs: " << msecs / 1000.0 << ")" << std::endl;
        usleep(msecs * 1000);
        std::cout << "    bool wait() - stop";
        bool ok = randbool(.8);
        std::cout << (ok ? " -> [OK]" : " -> [ERROR]") << std::endl;
        return ok;
    }

    // send the response to the queue
    void send_response(Queue_t *into_queue, const void *from_buffer) {
        std::cout << "    void send_response(const Response_t &response)" << std::endl;
    }
};

/******************************************************************************/

int main()
{
    DummyDevice device;
    DeviceDriver<DummyAsyncInterface, DummyDevice> driver(device);

    driver.run();
}

