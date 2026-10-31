
#ifndef MQTT_CONFIG_EXAMPLE_H
#define MQTT_CONFIG_EXAMPLE_H

/*
 * INF2004 Group 27 - MQTT Configuration Template
 *
 * Each team member should:
 * 1. Copy this file to mqtt_config.h
 * 2. Change MQTT_BROKER_HOST to their laptop's local IP address
 * 3. Ensure the Pico W and laptop are on the same network
 *
 * Do not commit mqtt_config.h to GitHub.
 */

#define MQTT_BROKER_HOST "192.168.1.100"  /* Replace with your laptop's IP */
#define MQTT_BROKER_PORT 1883

#define MQTT_CLIENT_ID "inf2004-group27-car"

#define MQTT_TOPIC_TELEMETRY "inf2004/group27/telemetry"
#define MQTT_TOPIC_COMMAND   "inf2004/group27/command"
#define MQTT_TOPIC_HEARTBEAT "inf2004/group27/heartbeat"
#define MQTT_TOPIC_LOG       "inf2004/group27/log"

#endif
