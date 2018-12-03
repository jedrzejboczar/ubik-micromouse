#include <cstring>
#include <cstdio>
#include <cstdint>
#include <utility>


struct MyStruct {
    int a;
    // debug all constructors/destructors
    MyStruct(int a): a(a) { std::printf("MyStruct(%d)\n", a); }
    MyStruct(const MyStruct &other): a(other.a) {
        std::printf("MyStruct("); other.print(); std::printf(")\n"); }
    MyStruct& operator=(const MyStruct &other) {
        a = other.a; std::printf("MyStruct = "); other.print(); std::printf("\n"); return *this; }
    ~MyStruct() { std::printf("~MyStruct(a=%d)\n", a); }

    void print() const { std::printf("MyStruct(a=%d)", a); }
};

struct MyStructWithSomethingElseForQueue {
    MyStruct my_struct;
    uint32_t something_else;
    // cannot, MyStruct is not default-initialised
    MyStructWithSomethingElseForQueue(MyStruct &&m, uint32_t something_else):
        my_struct(std::forward<MyStruct>(m)), something_else(something_else)
    { std::printf("MyStructWithSomethingElseForQueue(MyStruct &&m, uint32_t something_else)\n"); }
    ~MyStructWithSomethingElseForQueue() { std::printf("~MyStructWithSomethingElseForQueue()\n"); }
};


void queue_sink_with_id(void *data, size_t size_of_data) {
    uint8_t buffer[size_of_data];
    std::printf("queue_sink: copying the data to queue...\n");
    std::memcpy(buffer, data, size_of_data);
    std::printf("queue_sink: sending...\n");
    (void) buffer;
    // simulate receiving the data on the other side
    MyStructWithSomethingElseForQueue *data_received =
        reinterpret_cast<MyStructWithSomethingElseForQueue *>(buffer);
    std::printf("queue_sink: received something_else = %d, my_struct1 = ", data_received->something_else);
    data_received->my_struct.print();
    std::printf("\n");
    std::printf("queue_sink: leaving...\n");
}

void forward_to_queue_with_id(MyStruct &&m) {
    uint32_t id = 100;
    std::printf("forward_to_queue: creating data for queue...\n");
    // MyStructWithSomethingElseForQueue data(std::forward<MyStruct>(m), id);
    MyStructWithSomethingElseForQueue data(std::move(m), id);
    data.my_struct.a *= 10;
    std::printf("forward_to_queue: created data for queue, sending...\n");
    queue_sink_with_id(&data, sizeof(MyStructWithSomethingElseForQueue));
    std::printf("forward_to_queue: leaving...\n");
}

void queue_sink(void *data, size_t size_of_data) {
    uint8_t buffer[size_of_data];
    std::printf("queue_sink: copying the data to queue...\n");
    std::memcpy(buffer, data, size_of_data);
    std::printf("queue_sink: sending...\n");
    (void) buffer;
    // simulate receiving the data on the other side
    MyStruct *data_received =
        reinterpret_cast<MyStruct*>(buffer);
    std::printf("queue_sink: received my_struct1 = ");
    data_received->print();
    std::printf("\n");
    std::printf("queue_sink: leaving...\n");
}

void forward_to_queue(MyStruct &&m) {
    std::printf("forward_to_queue: sending...\n");
    queue_sink(&m, sizeof(MyStruct));
    std::printf("forward_to_queue: leaving...\n");
}


int main()
{

    std::printf("--------------------------------------------------------------------------------\n");

    {
        forward_to_queue_with_id(MyStruct(1));
    }

    std::printf("--------------------------------------------------------------------------------\n");

    {
        forward_to_queue(MyStruct(1));
    }

    std::printf("--------------------------------------------------------------------------------\n");

    {
        MyStruct my_struct1(2);
        forward_to_queue_with_id(std::forward<MyStruct>(my_struct1));
        std::printf("main: calling my_struct1.print()...\n");
        my_struct1.print();
        std::printf("\nmain: called my_struct1.print()\n");
    }

    std::printf("--------------------------------------------------------------------------------\n");

    {
        MyStruct my_struct1(2);
        forward_to_queue(std::forward<MyStruct>(my_struct1));
        std::printf("main: calling my_struct1.print()...\n");
        my_struct1.print();
        std::printf("\nmain: called my_struct1.print()\n");
    }

    std::printf("--------------------------------------------------------------------------------\n");
}
