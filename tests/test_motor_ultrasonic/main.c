/*!
 * @file main.c
 *
 * @brief Motor and ultrasonic sensor integration test
 *
 * @par
 * Test moves robot forward until obstacle detected within stop distance
 *
 * @par
 * COPYRIGHT NOTICE: (c) 2024 Your Name. All rights reserved.
 */

#include <stdio.h>
#include "pico/stdlib.h"
#include "motor.h"
#include "ultrasonic.h"

// Test configuration
#define STOP_DISTANCE_CM        (20u)     // Stop when obstacle < 20cm
#define MEASUREMENT_INTERVAL_MS (100u)    // Check distance every 100ms
#define BACKUP_DURATION_MS      (500u)    // Back up for 500ms

/*!
 * @brief Main test program entry point
 *
 * @return Should never return in embedded context
 */
int
main (void)
{
    uint16_t distance_cm  = 0u;
    bool     b_obstacle   = false;
    
    // Initialize Pico standard library
    stdio_init_all();
    
    // Initialize subsystems
    motor_init();
    ultrasonic_init();
    
    printf("\n");
    printf("========================================\n");
    printf("MOTOR + ULTRASONIC INTEGRATION TEST\n");
    printf("========================================\n");
    printf("Stop distance: %u cm\n", STOP_DISTANCE_CM);
    printf("Starting in 3 seconds...\n");
    printf("========================================\n\n");
    
    // Countdown before starting
    sleep_ms(3000u);
    
    printf("MOVING FORWARD...\n\n");
    motor_forward();
    
    // Main test loop
    while (false == b_obstacle)
    {
        // Measure distance to obstacle
        distance_cm = ultrasonic_get_distance_cm();
        
        if (0u == distance_cm)
        {
            // Measurement failed
            printf("WARNING: Distance measurement failed\n");
        }
        else if (distance_cm < ULTRASONIC_MIN_DISTANCE_CM)
        {
            // Object too close for reliable measurement
            printf("WARNING: Object too close (< %u cm)\n", 
                   ULTRASONIC_MIN_DISTANCE_CM);
        }
        else if (distance_cm > ULTRASONIC_MAX_DISTANCE_CM)
        {
            // Object out of range
            printf("OUT OF RANGE: > %u cm\n", ULTRASONIC_MAX_DISTANCE_CM);
        }
        else if (distance_cm < STOP_DISTANCE_CM)
        {
            // Obstacle detected within stop distance
            b_obstacle = true;
            
            printf("\n!!! OBSTACLE DETECTED at %u cm !!!\n", distance_cm);
            motor_stop_all();
            printf("MOTORS STOPPED\n\n");
            
            // Back up slightly
            printf("Backing up...\n");
            motor_backward();
            sleep_ms(BACKUP_DURATION_MS);
            motor_stop_all();
            
            printf("\n=== TEST COMPLETE ===\n");
        }
        else
        {
            // Clear path - continue moving
            printf("Distance: %u cm - Moving forward\n", distance_cm);
        }
        
        // Wait before next measurement
        sleep_ms(MEASUREMENT_INTERVAL_MS);
    }
    
    // Ensure motors stopped (should never reach here)
    motor_stop_all();
    
    return 0;
}

/*** end of file ***/
