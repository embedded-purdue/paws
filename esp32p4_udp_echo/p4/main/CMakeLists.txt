idf_component_register(
    SRCS "main.c"
    INCLUDE_DIRS "."
    PRIV_REQUIRES esp_netif esp_eth esp_event lwip ethernet_init
)
