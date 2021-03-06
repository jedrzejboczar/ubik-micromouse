// TODO: remove debug code
#include <iostream>


/*
 * This is an interface for device configuration and control.
 * It operates directly on hardware. It should confgure device
 * based on request data and start it in interrupt mode (preferably
 * with DMA). After waking up from sleep it is required to either
 * abort() cleaning up the hardware or to prepare the response
 * based on hardware peration results.
 */
template<typename RequestData_T, typename ResponseData_T>
class Device {
public:
    using RequestData_t = RequestData_T;
    using ResponseData_t = ResponseData_T;

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

/*
 * This is the type that is responsible for all the asynchronious
 * operations of a device driver.
 * It should define a type of Queue handles that are sent through
 * Requests (most probably some kind of pointer) to now where to send
 * the response.
 * It creates and manages the request queue of the driver and other
 * resources required to achieve the functionality.
 *
 * It should create a RTOS task/thread that will run the run() method,
 * which gets overwritten in DeviceDriver class depending on the device.
 *
 * ? The queue should probably be defined by length and data size in constructor ?
 */
template<typename Queue_t>
class AsyncInterface {
public:
    // using Queue_t = QueueHandle_T;

    // virtual bool initialise(size_t request_queue_length, size_t data_size) = 0;

    // wait for queue with some timeout, store to given reference
    virtual bool receive_request(void *into_buffer) = 0;

    // asynchronious wait for semaphore/task notification/whatever...
    virtual bool wait_for_isr() = 0;

    // send the response to the queue
    virtual void send_response(Queue_t *into_queue, const void *from_buffer) = 0;

    //
    virtual void run() = 0;

    // cannot have static pure virtual methods!
    // static bool notify();
    // this should probably be deined in implementation class
    // and be called to end wait()
};


/*
 * The class that implements RTOS diver logic.
 * It derivers methods for asynchronious operaiton based on the RTOS
 * used. The AsyncInterface should provide pure virtual method that
 * is the thread/task main function (so this class "takes" the
 * thread/task).
 */
template<typename AsyncInterface_t, typename Device_t>
class DeviceDriver: AsyncInterface_t {
    Device_t &device;
public:
    enum Status {
        OK,
        ERROR,   // device could not be started
        TIMEOUT, // timeout when waiting on reponse from device
    };

    struct Request {
        typename Device_t::RequestData_t data;     // e.g. poitner + size
        typename AsyncInterface_t::Queue_t *response_queue; // no response when null, message_type=Response

        bool requires_response() {
            return response_queue != nullptr;
        }
    };

    struct Response {
        Status status;       // ok / hardware error / hardware timeout
        typename Device_t::ResponseData_t data; // e.g. pointer + size
    };

    DeviceDriver(Device_t &device):
        device(device) { }

    using AsyncInterface_t::wait_for_isr;
    using AsyncInterface_t::receive_request;
    using AsyncInterface_t::send_response;
    using AsyncInterface_t::run;

    virtual void run() final {
        device.configure();

// TODO: remove debug code
//         while (1) {
        for (int i = 0; i < 10; i++) {
            std::cout << "### Run number: " << i << std::endl;


            Request request;
            Response response;

            // wait for a request in the queue
            while (!receive_request(reinterpret_cast<void *>(&request))) {}

            // prepare the device and its interrupt
            if (!device.start_async(request.data)) {
                response.status = Status::ERROR;
            } else {
                // wait for interrupt from device
                if (!wait_for_isr()) {
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
                send_response(request.response_queue, reinterpret_cast<const void *>(&response));
            }
        }
    }

};
