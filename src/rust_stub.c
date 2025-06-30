// Temporary stub implementations for testing compilation
#include "s1_driver/rust_interface.h"
#include <stdio.h>
#include <stdlib.h>
#include <string.h>

// Stub implementations of Rust FFI functions
int32_t rust_bridge_create(const char* interface) {
    printf("rust_bridge_create called with interface: %s\n", interface ? interface : "NULL");
    return 0; // Return valid handle ID
}

c_bool rust_bridge_initialize(int32_t handle_id, const char* interface) {
    printf("rust_bridge_initialize called with handle_id: %d, interface: %s\n", 
           handle_id, interface ? interface : "NULL");
    return 1; // Success
}

void rust_bridge_destroy(int32_t handle_id) {
    printf("rust_bridge_destroy called with handle_id: %d\n", handle_id);
}

c_bool rust_bridge_send_movement(int32_t handle_id, MovementParams params) {
    printf("rust_bridge_send_movement called with handle_id: %d, vx: %f, vy: %f, vz: %f\n", 
           handle_id, params.vx, params.vy, params.vz);
    return 1; // Success
}

c_bool rust_bridge_stop(int32_t handle_id) {
    printf("rust_bridge_stop called with handle_id: %d\n", handle_id);
    return 1; // Success
}

c_bool rust_bridge_send_led_color(int32_t handle_id, LedColor color) {
    printf("rust_bridge_send_led_color called with handle_id: %d, r: %d, g: %d, b: %d\n", 
           handle_id, color.red, color.green, color.blue);
    return 1; // Success
}

c_bool rust_bridge_read_sensor_data(int32_t handle_id) {
    printf("rust_bridge_read_sensor_data called with handle_id: %d\n", handle_id);
    return 1; // Success
}

float rust_bridge_get_battery_level(int32_t handle_id) {
    printf("rust_bridge_get_battery_level called with handle_id: %d\n", handle_id);
    return 100.0; // Full battery
}

c_bool rust_bridge_get_imu_data(int32_t handle_id, 
                                float* acc_x, float* acc_y, float* acc_z,
                                float* gyro_x, float* gyro_y, float* gyro_z) {
    printf("rust_bridge_get_imu_data called with handle_id: %d\n", handle_id);
    if (acc_x) *acc_x = 0.0;
    if (acc_y) *acc_y = 0.0;
    if (acc_z) *acc_z = 9.81;
    if (gyro_x) *gyro_x = 0.0;
    if (gyro_y) *gyro_y = 0.0;
    if (gyro_z) *gyro_z = 0.0;
    return 1; // Success
}

const char* rust_bridge_get_last_error(int32_t handle_id) {
    printf("rust_bridge_get_last_error called with handle_id: %d\n", handle_id);
    return "No error"; // Static string, no need to free
}

void rust_bridge_free_string(char* s) {
    printf("rust_bridge_free_string called\n");
    // For static strings, do nothing
}
