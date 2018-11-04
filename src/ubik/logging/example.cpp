/*** Usage example ************************************************************/

#include "logging.h"

#include <cstdio>
#include <thread>
#include <chrono>

bool logging::log(logging::Msg msg) {
    int result = std::printf("%s", reinterpret_cast<char *>(msg.data)) >= 0;
    msg.delete_if_owned();
    return result >= 0;
}

/*
 * To check that memory is handled right run:
 *    g++ example.cpp
 *    valgrind --tool=memcheck a.out
 */
int main() {
    int x = 2;
    {   // Create dynamic message that should be deleted by the logger.
        const size_t size = 100;
        uint8_t *msg = new uint8_t[size];
        snprintf(reinterpret_cast<char*>(msg), size, "My log message, x=%d\n", x);
        logging::log(logging::Msg{msg, size, true});
        // do NOT use msg now, ownership has been passed to log()!
    }

    {   // Create dynamic message that should be deleted by the logger.
        // THIS IS THE PREFFERED WAY
        const size_t size = 100;
        logging::Msg msg = logging::Msg::dynamic(size);
        snprintf(msg.as_chars(), msg.size, "My log message, x=%d\n", x);
        logging::log(msg);
        // do NOT use msg now, ownership has been passed to log()!
    }
    {   // Create dynamic message that should NOT be deleted by the logger.
        // DANGEROUS!
        // this is impractical and rather theoretical, as we may not have
        // any no way to tell if logger has ended, but we must delete the message later
        const size_t size = 100;
        logging::Msg msg = logging::Msg::dynamic(size, false);
        snprintf(msg.as_chars(), msg.size, "My log message, x=%d\n", x);
        logging::log(msg);
        // do NOT use it now as it may be still used bu the logger!
        std::this_thread::sleep_for(std::chrono::milliseconds(50));
        // we must somehow delete it
        msg.delete_if_owned();
    }
    {   // Create a message from static array
        // DANGEROUS!
        // beware! it is unsafe if the call to log() does not block!
        uint8_t msg[100];
        snprintf(reinterpret_cast<char*>(msg), sizeof(msg), "My log message, x=%d\n", x);
        logging::log(logging::Msg{msg, sizeof(msg), false});
    }
    {   // Create a message from static array
        // DANGEROUS!
        // beware! it is unsafe if the call to log() does not block!
        uint8_t msg[100];
        snprintf(reinterpret_cast<char*>(msg), sizeof(msg), "My log message, x=%d\n", x);
        logging::log(logging::Msg::from_static(msg));
    }
    {   // Create a message from static array
        // DANGEROUS!
        // beware! it is unsafe if the call to log() does not block!
        uint8_t buf[100];
        logging::Msg msg = logging::Msg::from_static(buf);
        snprintf(msg.as_chars(), sizeof(msg), "My log message, x=%d\n", x);
        logging::log(msg);
    }
}
