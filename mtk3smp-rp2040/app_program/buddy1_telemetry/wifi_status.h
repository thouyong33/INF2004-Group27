/*
 * wifi_status.h
 * Wi-Fi status task (Buddy 1 starting point). Only with WIFI=cyw43.
 */

#ifndef BUDDY1_WIFI_STATUS_H
#define BUDDY1_WIFI_STATUS_H

#include <tk/tkernel.h>

#if TM_WIFI_CYW43
/** @brief Create and start the task printing link, DHCP and UDP status. */
ER wifi_status_create(void);
#endif

#endif /* BUDDY1_WIFI_STATUS_H */
