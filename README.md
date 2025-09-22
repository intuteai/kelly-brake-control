# kelly-brake-control
ESP32-C3 firmware bridging Kelly Controller CAN to BLE. It decodes speed, motor current and throttle, runs a relay-based brake state machine, reads multiplexed digital inputs, and reports over BLE. Built-in reliability: CAN supervision/recovery, periodic watchdog, and wake interrupt handling.
