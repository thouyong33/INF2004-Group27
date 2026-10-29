/*
 * MQTT broker for the Buddy 1 telemetry build (WIFI_MQTT=1).
 *
 * Copy this file to config/mqtt_config.h and set the address of the laptop
 * running tools/mqtt_viewer.py (it starts the broker). The local copy is
 * ignored by Git because host addresses vary between networks.
 */
#ifndef UTK3_MQTT_CONFIG_H
#define UTK3_MQTT_CONFIG_H

/* Change to 1 once the address below is set (an unedited copy fails to
   compile rather than connecting nowhere). */
#define UTK_MQTT_SET   0

#if !UTK_MQTT_SET
#error "config/mqtt_config.h: placeholder values. Set the broker address, then set UTK_MQTT_SET to 1."
#endif

#define UTK_MQTT_BROKER_ADDRESS    "192.168.1.100"
#define UTK_MQTT_BROKER_PORT       1883U

#endif
