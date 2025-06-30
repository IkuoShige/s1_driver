/// C interface bridge for robomaster-rust library
/// This module provides a C-compatible interface for integration with ROS2 C++ nodes

use std::ffi::{CStr, CString};
use std::os::raw::{c_char, c_float, c_bool};
use std::ptr;
use std::sync::Mutex;
use tokio::runtime::Runtime;
use tokio::sync::oneshot;
use anyhow::Result;

use robomaster_rust::{RoboMaster, MovementCommand, LedCommand};

// C structure definitions matching the header
#[repr(C)]
pub struct MovementParams {
    pub vx: c_float,
    pub vy: c_float,
    pub vz: c_float,
}

#[repr(C)]
pub struct SensorData {
    // IMU data
    pub accel_x: c_float,
    pub accel_y: c_float,
    pub accel_z: c_float,
    pub gyro_x: c_float,
    pub gyro_y: c_float,
    pub gyro_z: c_float,
    
    // Motor feedback
    pub wheel_speeds: [c_float; 4],
    
    // Battery status
    pub battery_voltage: c_float,
    pub battery_current: c_float,
    pub battery_temperature: c_float,
    
    // System status
    pub is_connected: c_bool,
    pub timestamp_us: u64,
}

#[repr(C)]
pub struct LedColor {
    pub r: u8,
    pub g: u8,
    pub b: u8,
    pub brightness: u8,
}

// Robot handle wrapper
pub struct RoboMasterHandle {
    robot: Option<RoboMaster>,
    runtime: Runtime,
    can_interface: String,
    last_error: Mutex<Option<String>>,
    is_initialized: bool,
}

impl RoboMasterHandle {
    fn new(can_interface: String) -> Result<Self> {
        let runtime = Runtime::new()?;
        
        Ok(RoboMasterHandle {
            robot: None,
            runtime,
            can_interface,
            last_error: Mutex::new(None),
            is_initialized: false,
        })
    }
    
    fn set_error(&self, error: String) {
        if let Ok(mut last_error) = self.last_error.lock() {
            *last_error = Some(error);
        }
    }
    
    fn get_error(&self) -> Option<String> {
        if let Ok(last_error) = self.last_error.lock() {
            last_error.clone()
        } else {
            None
        }
    }
}

// Helper function to execute async operations synchronously
fn block_on_async<F, T>(handle: &mut RoboMasterHandle, future: F) -> Result<T>
where
    F: std::future::Future<Output = Result<T>> + Send + 'static,
    T: Send + 'static,
{
    handle.runtime.block_on(future)
}

// C interface functions
#[no_mangle]
pub extern "C" fn robomaster_create(can_interface: *const c_char) -> *mut RoboMasterHandle {
    if can_interface.is_null() {
        return ptr::null_mut();
    }
    
    let interface = unsafe {
        match CStr::from_ptr(can_interface).to_str() {
            Ok(s) => s.to_string(),
            Err(_) => return ptr::null_mut(),
        }
    };
    
    match RoboMasterHandle::new(interface) {
        Ok(handle) => Box::into_raw(Box::new(handle)),
        Err(_) => ptr::null_mut(),
    }
}

#[no_mangle]
pub extern "C" fn robomaster_initialize(handle: *mut RoboMasterHandle) -> c_bool {
    if handle.is_null() {
        return false as c_bool;
    }
    
    let handle = unsafe { &mut *handle };
    
    let can_interface = handle.can_interface.clone();
    let result = block_on_async(handle, async move {
        let robot = RoboMaster::new(&can_interface).await?;
        Ok(robot)
    });
    
    match result {
        Ok(robot) => {
            // Initialize the robot
            let init_result = block_on_async(handle, async move {
                robot.initialize().await
            });
            
            match init_result {
                Ok(robot) => {
                    handle.robot = Some(robot);
                    handle.is_initialized = true;
                    true as c_bool
                },
                Err(e) => {
                    handle.set_error(format!("Initialization failed: {}", e));
                    false as c_bool
                }
            }
        },
        Err(e) => {
            handle.set_error(format!("Connection failed: {}", e));
            false as c_bool
        }
    }
}

#[no_mangle]
pub extern "C" fn robomaster_destroy(handle: *mut RoboMasterHandle) {
    if !handle.is_null() {
        unsafe {
            let handle = Box::from_raw(handle);
            // Destructor will clean up resources
        }
    }
}

#[no_mangle]
pub extern "C" fn robomaster_move(handle: *mut RoboMasterHandle, params: *const MovementParams) -> c_bool {
    if handle.is_null() || params.is_null() {
        return false as c_bool;
    }
    
    let handle = unsafe { &mut *handle };
    let params = unsafe { &*params };
    
    if !handle.is_initialized || handle.robot.is_none() {
        handle.set_error("Robot not initialized".to_string());
        return false as c_bool;
    }
    
    // Create movement command
    let movement_cmd = MovementCommand::new()
        .forward(params.vx)
        .strafe_right(params.vy)
        .rotate_right(params.vz);
    
    // Execute movement
    if let Some(ref robot) = handle.robot {
        let robot_clone = robot.clone(); // Assuming RoboMaster implements Clone or we use Arc
        let result = block_on_async(handle, async move {
            robot_clone.move_robot(movement_cmd.into_params()).await
        });
        
        match result {
            Ok(_) => true as c_bool,
            Err(e) => {
                handle.set_error(format!("Movement failed: {}", e));
                false as c_bool
            }
        }
    } else {
        false as c_bool
    }
}

#[no_mangle]
pub extern "C" fn robomaster_stop(handle: *mut RoboMasterHandle) -> c_bool {
    if handle.is_null() {
        return false as c_bool;
    }
    
    let handle = unsafe { &mut *handle };
    
    if !handle.is_initialized || handle.robot.is_none() {
        handle.set_error("Robot not initialized".to_string());
        return false as c_bool;
    }
    
    if let Some(ref robot) = handle.robot {
        let robot_clone = robot.clone();
        let result = block_on_async(handle, async move {
            robot_clone.stop().await
        });
        
        match result {
            Ok(_) => true as c_bool,
            Err(e) => {
                handle.set_error(format!("Stop failed: {}", e));
                false as c_bool
            }
        }
    } else {
        false as c_bool
    }
}

#[no_mangle]
pub extern "C" fn robomaster_read_sensors(handle: *mut RoboMasterHandle, data: *mut SensorData) -> c_bool {
    if handle.is_null() || data.is_null() {
        return false as c_bool;
    }
    
    let handle = unsafe { &mut *handle };
    let data = unsafe { &mut *data };
    
    if !handle.is_initialized || handle.robot.is_none() {
        handle.set_error("Robot not initialized".to_string());
        return false as c_bool;
    }
    
    // TODO: Implement actual sensor reading from robot
    // For now, return default/placeholder values
    *data = SensorData {
        accel_x: 0.0,
        accel_y: 0.0,
        accel_z: 9.81,
        gyro_x: 0.0,
        gyro_y: 0.0,
        gyro_z: 0.0,
        wheel_speeds: [0.0; 4],
        battery_voltage: 12.0,
        battery_current: 0.0,
        battery_temperature: 25.0,
        is_connected: handle.is_initialized as c_bool,
        timestamp_us: std::time::SystemTime::now()
            .duration_since(std::time::UNIX_EPOCH)
            .unwrap_or_default()
            .as_micros() as u64,
    };
    
    true as c_bool
}

#[no_mangle]
pub extern "C" fn robomaster_set_led(handle: *mut RoboMasterHandle, color: *const LedColor) -> c_bool {
    if handle.is_null() || color.is_null() {
        return false as c_bool;
    }
    
    let handle = unsafe { &mut *handle };
    let color = unsafe { &*color };
    
    if !handle.is_initialized || handle.robot.is_none() {
        handle.set_error("Robot not initialized".to_string());
        return false as c_bool;
    }
    
    if let Some(ref robot) = handle.robot {
        let led_cmd = LedCommand::new()
            .red(color.r)
            .green(color.g)
            .blue(color.b);
        
        let robot_clone = robot.clone();
        let result = block_on_async(handle, async move {
            robot_clone.control_led(led_cmd.color()).await
        });
        
        match result {
            Ok(_) => true as c_bool,
            Err(e) => {
                handle.set_error(format!("LED control failed: {}", e));
                false as c_bool
            }
        }
    } else {
        false as c_bool
    }
}

#[no_mangle]
pub extern "C" fn robomaster_is_connected(handle: *mut RoboMasterHandle) -> c_bool {
    if handle.is_null() {
        return false as c_bool;
    }
    
    let handle = unsafe { &*handle };
    (handle.is_initialized && handle.robot.is_some()) as c_bool
}

#[no_mangle]
pub extern "C" fn robomaster_get_last_error(handle: *mut RoboMasterHandle) -> *const c_char {
    if handle.is_null() {
        return ptr::null();
    }
    
    let handle = unsafe { &*handle };
    
    match handle.get_error() {
        Some(error) => {
            match CString::new(error) {
                Ok(c_string) => c_string.into_raw(),
                Err(_) => ptr::null(),
            }
        },
        None => ptr::null(),
    }
}

// Note: This implementation assumes robomaster-rust RoboMaster struct can be cloned
// If not, we need to use Arc<Mutex<RoboMaster>> or similar synchronization
