/*** Usage example ************************************************************/

#include "logging.h"

#include <cstdio>
#include <thread>
#include <chrono>

bool logging::log(logging::Buffer buf) {
    int result = std::printf("%s", reinterpret_cast<char *>(buf.data)) >= 0;
    if (buf.is_owner)
        delete[] buf.data;
    return result >= 0;
}

/*
 * To check that memory is handled right run:
 *    g++ example.cpp
 *    valgrind --tool=memcheck a.out
 */
int main() {
    int x = 2;
    {   // Create dynamic buffer that should be deleted by the logger.
        const size_t size = 100;
        uint8_t *buf = new uint8_t[size];
        snprintf(reinterpret_cast<char*>(buf), size, "My log message, x=%d\n", x);
        logging::log(logging::Buffer{.data=buf, .size=size, .is_owner=true});
        // do NOT use buf now, ownership has been passed to log()!
    }
    {   // Create dynamic buffer that should be deleted by the logger.
        // THIS IS THE PREFFERED WAY
        const size_t size = 100;
        logging::Buffer buf = logging::Buffer::dynamic(size);
        snprintf(reinterpret_cast<char*>(buf.data), buf.size, "My log message, x=%d\n", x);
        logging::log(buf);
        // do NOT use buf now, ownership has been passed to log()!
    }
    {   // Create dynamic buffer that should NOT be deleted by the logger.
        // DANGEROUS!
        // this is impractical and rather theoretical, as we may not have
        // any no way to tell if logger has ended, but we must delete the buffer later
        const size_t size = 100;
        logging::Buffer buf = logging::Buffer::dynamic(size, false);
        snprintf(reinterpret_cast<char*>(buf.data), buf.size, "My log message, x=%d\n", x);
        logging::log(buf);
        // do NOT use it now as it may be still used bu the logger!
        std::this_thread::sleep_for(std::chrono::milliseconds(50));
        // we must somehow delete it
        delete [] buf.data;
    }
    {   // Create a buffer from static array
        // DANGEROUS!
        // beware! it is unsafe if the call to log() does not block!
        uint8_t buf[100];
        snprintf(reinterpret_cast<char*>(buf), sizeof(buf), "My log message, x=%d\n", x);
        logging::log(logging::Buffer{.data=buf, .size=sizeof(buf), .is_owner=false});
    }
    {   // Create a buffer from static array
        // DANGEROUS!
        // beware! it is unsafe if the call to log() does not block!
        uint8_t buf[100];
        snprintf(reinterpret_cast<char*>(buf), sizeof(buf), "My log message, x=%d\n", x);
        logging::log(logging::Buffer::from_static(buf));
    }
}
