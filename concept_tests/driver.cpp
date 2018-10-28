#include <iostream>

enum struct Status {
    OK,
    ERROR,   // device could not be started
    TIMEOUT, // timeout when waiting on reponse from device
};

template<typename RequestData_t, typename Queue_t>
struct Request {
    RequestData_t data;     // e.g. poitner + size
    Queue_t response_queue; // no response when null, message_type=Response

    bool requires_response() {
        return response_queue != nullptr;
    }
};

template<typename ResponseData_t>
struct Response {
    Status status;       // ok / hardware error / hardware timeout
    ResponseData_t data; // e.g. pointer + size
};


template<typename RequestData_t, typename ResponseData_t>
class Device {
public:
    // defines ResponseData

    // prepare the device, this function should panic! on fail
    virtual void configure() = 0;

    // start device, use given request data, configure interrupt
    virtual bool start_async(const RequestData_t &data) = 0;
    // (!) there has to be some connection between the interrupt and the RTOS component

    // stop device, clean it up and prepare it for another usage
    virtual void abort() = 0;

    // set the data from the device for the response
    virtual void prepare_response(ResponseData_t &data) = 0;
};

template<typename Request_t, typename Response_t>
class RTOS {
public:

    // wait for queue with some timeout, store to given reference
    virtual bool receive_request(Request_t &request) = 0;

    // asynchronious wait for semaphore/task notification/whatever...
    virtual bool wait() = 0;

    // send the response to the queue
    virtual void send_response(const Response_t &response) = 0;

    // cannot have static pure virtual methods!
    // static bool notify();
};


template<typename Device_t, typename RTOS_t>
class DeviceDriver {
    Device_t device;
    RTOS_t rtos;

    typedef Request<typename Device_t::RequestData_t, typename RTOS_t::Queue_t>
        Request_t;
    typedef Response<typename Device_t::ResponseData_t>
        Response_t;

public:
    void run() {
        device.configure();

#if 1
        for (int i = 0; i < 10; i++) {
            std::cout << "### Run number: " << i << std::endl;
#else
        while (1) {
#endif
            Request_t request;
            Response_t response;

            // wait for a request in the queue
            while (!rtos.receive_request(request)) {}

            // prepare the device and its interrupt
            if (!device.start_async(request.data)) {
                response.status = Status::ERROR;
            } else {
                // wait for interrupt from device
                if (!rtos.wait()) {
                    response.status = Status::TIMEOUT;
                    // force stop and clean up the device
                    device.abort();
                } else {
                    response.status = Status::OK;
                    // prepare data for reponse if we need to send response
                    if (request.requires_response()) {
                        device.prepare_response(response.data);
                    }
                }
            }

            // send response if it was requested
            if (request.requires_response()) {
                rtos.send_response(response);
            }
        }
    }

};

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

class DummyDevice: Device<int, int> {
public:
    typedef int RequestData_t;
    typedef int ResponseData_t;

    // defines ResponseData

    // prepare the device, this function should panic! on fail
    void configure() {
        std::cout << "    void configure()" << std::endl;
    }

    // start device, use given request data, configure interrupt
    bool start_async(const RequestData_t &data) {
        std::cout << "    bool start_async(const RequestData_t &data)";
        bool ok = randbool(.6);
        std::cout << (ok ? " -> [OK]" : " -> [ERROR]") << std::endl;
        return ok;
    }
    // (!) there has to be some connection between the interrupt and the RTOS component

    // stop device, clean it up and prepare it for another usage
    void abort() {
        std::cout << "    void abort()" << std::endl;
    }

    // set the data from the device for the response
    void prepare_response(ResponseData_t &data) {
        std::cout << "    void prepare_response(ResponseData_t &data)" << std::endl;
    }
};

// this we should get from our RTOS
typedef int* DymmyRTOSFrameworkQueue;

class DummyRTOS: RTOS<Request<int, DymmyRTOSFrameworkQueue>, Response<int>> {
public:
    typedef DymmyRTOSFrameworkQueue Queue_t;
    typedef Request<int, Queue_t> Request_t;
    typedef Response<int> Response_t;

    // wait for queue with some timeout, store to given reference
    bool receive_request(Request_t &request) {
        std::cout << "    bool receive_request(Request_t &request)";
        bool ok = randbool(.4);
        std::cout << (ok ? " -> [OK]" : " -> [ERROR]") << std::endl;
        return ok;
    }

    // asynchronious wait for semaphore/task notification/whatever...
    bool wait() {
        int msecs = 1 + randfloat() * 1000;
        std::cout << "    bool wait() - start (secs: " << msecs / 1000.0 << ")" << std::endl;
        usleep(msecs * 1000);
        std::cout << "    bool wait() - stop";
        bool ok = randbool(.8);
        std::cout << (ok ? " -> [OK]" : " -> [ERROR]") << std::endl;
        return ok;
    }

    // send the response to the queue
    void send_response(const Response_t &response) {
        std::cout << "    void send_response(const Response_t &response)" << std::endl;
    }
};

int main()
{
    DeviceDriver<DummyDevice, DummyRTOS> driver;

    driver.run();
}

