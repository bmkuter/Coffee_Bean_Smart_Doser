idf_component_register(SRCS "coffee_doser_main.c"
                            "tasks/led_task.c"
                            "tasks/rotary_encoder_task.c"
                            "tasks/display_task.c"
                            "tasks/nau7802_task.c"
                            "tasks/motor_control_task.c"
                            "hardware/shared_i2c_bus.c"
                       INCLUDE_DIRS "." "include")
