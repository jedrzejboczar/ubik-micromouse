# Ubik

This is a hobby project of a micromouse robot.
This repository contains its software.
It wasn't written with public view in mind, so it may not be documented very well - read and use at your own risk.


## Code organization

The code has been written in C++17 for the STM32F103 microcontroller under FreeRTOS.

**Directories hierarchy**

```
src/
├── cube/                      # STM32 Cube libraries with initial device configuraton
├── freeRTOS/                  # standard FreeRTOS library (no CMSIS-OS)
├── mpaland_printf/            # tiny printf implementation for embedded systems by mpaland
└── ubik/                      # main code of this project
    ├── common/                # common utilities, simple hardware wrappers, global state, interrupts, etc.
    ├── localization/          # module resposible for position tracking with dead reckoning 
    ├── logging/               # logging to PC using Bluetooth module via UART
    ├── maze/                  # implementation of maze traversal logic (abstract, moves by discrete cells)
    ├── movement/              # movement controller - trajectory generation and tracking
    ├── system_monitor.cpp     # top-level system control
    └── ubik.cpp               # actual main function
```

**HAL**

As for HAL the libraries from ST have been used. 
Most of uC hardware configuration has been done using STM32Cube utility to minimize development time.
All the required configurations have been "documented" in the modules that directly use given peripherals
or in the *NOTES.md* file.

STM32Cube generates its own *main.c*, and the code is not really good for use in a bigger project, but
it saves a lot of development time.
To decouple the code just a little bit this *main.c* calls my `extern_main()` after
performing the initial hardware configuration, and the rest of the main code is located in *src/ubik/ubik.cpp*.

**RTOS**

FreeRTOS has been used without the CMSIS-OS wrappers as the direct usage was much more convenient.

**printf**

I used a simple implementation of printf function by mpaland as I wanted printf that can handle
simple floating point numbers formatting without blowing up the binray size (there is not much space left anyway,
as the rest of code is quite big and not written with code size as the primary concern).

