idf_component_register(SRCS "main.c" "gait.c" "mailbox.c" "uart_rx.c" "gait_task.c"
                       INCLUDE_DIRS "."
                       REQUIRES esp_driver_uart)
