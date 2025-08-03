```mermaid
flowchart TD
    %% Main entry point
    app_main["app_main()"]
    nvs_init["Initialize NVS, logging, watchdog, network stack"]
    boot_btn["Wait for boot button (AP config mode)"]
    ap_config["Start AP config task"]
    load_config["Load config from NVS"]
    sensor_config["Initialize sensor config system"]
    sensors_init["Initialize enabled sensors"]
    i2c_diag["Run I2C diagnostic scan"]
    spiffs["Initialize SPIFFS filesystem"]
    wifi_sta["Initialize WiFi STA mode"]
    sw_watchdog["Configure software watchdog"]
    main_loop["Main loop"]
    watchdog_check["Check software watchdog timeout"]
    read_sensors["read_sensor_data()"]
    publish_mqtt["publish_combined_sensor_mqtt()"]
    delay["Delay for sample interval"]

    %% Sensor reading
    read_sensors -->|Iterate enabled sensors| sensor_bh1750["sensor_bh1750_read()"]
    read_sensors --> sensor_bme680["sensor_bme680_read()"]
    read_sensors --> sensor_wind["sensor_wind_speed_read()"]
    read_sensors --> todo_sensors["TODO: DHT22, DS18B20, BME280"]
    read_sensors --> publish_mqtt

    %% Main flow
    app_main --> nvs_init
    nvs_init --> boot_btn
    boot_btn -->|Pressed| ap_config
    boot_btn -->|Not pressed| load_config
    load_config --> sensor_config
    sensor_config --> sensors_init
    sensors_init --> i2c_diag
    i2c_diag --> spiffs
    spiffs --> wifi_sta
    wifi_sta --> sw_watchdog
    sw_watchdog --> main_loop
    main_loop --> watchdog_check
    main_loop --> read_sensors
    main_loop --> delay

    %% HTTP Handlers
    subgraph HTTP_Handlers
        params_json["params_json_get_handler()"]
        params_update["params_update_post_handler()"]
        parameters_html["parameters_html_handler()"]
        captive_redirect["captive_redirect_handler()"]
        sysinfo_json["sysinfo_json_get_handler()"]
        ota_firmware["ota_firmware_handler()"]
        ota_fs["ota_filesystem_handler()"]
    end

    %% Utility/System functions (not all relationships shown for clarity)
    subgraph Utility_System
        load_wifi["load_wifi_config_from_nvs()"]
        load_mqtt["load_mqtt_config_from_nvs()"]
        load_interval["load_sample_interval_from_nvs()"]
        load_topic["load_data_topic_from_nvs()"]
        load_watchdog["load_watchdog_counter_from_nvs()"]
        save_watchdog["save_watchdog_counter_to_nvs()"]
        init_spiffs["init_spiffs()"]
        url_decode["url_decode()"]
        dns_hijack["dns_hijack_task()"]
        wifi_event["wifi_event_handler()"]
        mqtt_event["mqtt_event_handler()"]
        test_mqtt["test_mqtt_broker_connectivity()"]
        check_wifi["check_wifi_connection()"]
        wifi_init["wifi_init_sta()"]
        mqtt_start["mqtt_app_start()"]
        publish_sensor["publish_sensor_data_mqtt()"]
        publish_combined["publish_combined_sensor_mqtt()"]
    end
    ```