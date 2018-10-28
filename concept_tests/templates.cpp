#include <iostream>

class Device {
public:
    struct Data {
        int x, y;
    };

    Data data;

    Device() {
        data.x = 10;
        data.y = 20;
        print(data);
        data.x++;
        data.y++;
    }

    void print(Data data) {
        std::cout << "Data: " << data.x << ", " << data.y << std::endl;
    }
};


template <typename Device_t>
class Driver {
public:

    Device_t dev;

    Driver() {
        typename Device_t::Data x;
        x.x = 100;
        x.y = 200;

        dev.print(x);
        dev.print(dev.data);
    }

};


int main()
{
    Driver<Device> driver;
}
