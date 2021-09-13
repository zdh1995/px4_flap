set(MODULE_CFLAGS)
px4_add_module(
    MODULE modules__ultrasonic_distance
    MAIN ultrasonic_distance
#    COMPILE_FLAGS
#        -Os
    SRCS
        ultrasonic_distance.c
    DEPENDS
#        platforms__common
    ) 
