
# Creates the files needed for targets created using other functions
function(openocd_configure_files OPENOCD_INTERFACE OPENOCD_TARGET)

    set(OPENOCD_LOGFILE "${CMAKE_CURRENT_BINARY_DIR}/openocd/openocd.log")
    set(OPENOCD_CONNECT "${CMAKE_CURRENT_BINARY_DIR}/openocd/connect.cfg")
    set(OPENOCD_GDB "${CMAKE_CURRENT_BINARY_DIR}/openocd/gdb.cfg")

    configure_file(openocd/connect.cfg.in openocd/connect.cfg)
    configure_file(openocd/gdb.cfg.in openocd/gdb.cfg)
    configure_file(openocd/gdbinit_pipe.in openocd/gdbinit_pipe)

endfunction()

# Adds a custom target to flash the program from given traget's ELF file
function(openocd_add_flash_target ELF_TARGET)

    add_custom_target(flash
        openocd
        -f "${CMAKE_CURRENT_BINARY_DIR}/openocd/connect.cfg"
        -c "program $<TARGET_FILE:${ELF_TARGET}> verify reset exit"
        DEPENDS
        "${ELF_TARGET}"
        "${CMAKE_CURRENT_BINARY_DIR}/openocd/connect.cfg"
        )

endfunction()

# Adds a custom target to start GDB with OpenOCD connected through pipe
function(openocd_add_gdb_target ELF_TARGET)

    add_custom_target(gdb
        ${CMAKE_DEBUGER}
        "--command=${CMAKE_CURRENT_BINARY_DIR}/openocd/gdbinit_pipe"
        "$<TARGET_FILE:${ELF_TARGET}>" # executable
        DEPENDS
        "${ELF_TARGET}"
        "${CMAKE_CURRENT_BINARY_DIR}/openocd/gdb.cfg"
        )

endfunction()

