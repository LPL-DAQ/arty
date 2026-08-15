zephyr_library()
set(CONFIG_MCUX_COMPONENT_driver.lpspi ON)
zephyr_library_sources(adc_ads79xx.c fast_lpspi.c)
