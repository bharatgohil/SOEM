
set(SOURCES servo_drv.c)
add_executable(servo_drv ${SOURCES})
target_link_libraries(servo_drv soem)
install(TARGETS servo_drv DESTINATION bin)
