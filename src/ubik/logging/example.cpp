/*** Usage example ************************************************************/

#include "logging.h"

#include <cstdio>

bool logging::log(logging::Buffer buf) {
    int result = std::printf("%s", reinterpret_cast<char *>(buf.data)) >= 0;
    if (buf.is_owner)
        delete[] buf.data;
    return result >= 0;
}

int main() {
    int x = 2;
    {
        const size_t size = 100;
        uint8_t *buf = new uint8_t[size];
        snprintf(reinterpret_cast<char*>(buf), size, "My log message, x=%d\n", x);
        logging::log(logging::Buffer{.data=buf, .size=size, .is_owner=true});
        // do NOT use buf now, ownership has been passed to log()!
    }
    {
        // beware! it is unsafe if the call to log() does not block!
        uint8_t buf[100];
        snprintf(reinterpret_cast<char*>(buf), sizeof(buf), "My log message, x=%d\n", x);
        logging::log(logging::Buffer::from_static(buf));
    }
}
