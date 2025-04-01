use std::time::Duration;
use tokio;

use lx16a_rs::Result;
use lx16a_rs::LX16A;

#[tokio::main]
async fn main() -> Result<()> {
    // Parse command line arguments
    let args: Vec<String> = std::env::args().collect();
    if args.len() < 2 {
        eprintln!("Usage: {} <serial_port>", args[0]);
        std::process::exit(1);
    }
    let port = &args[1];
    println!("Initializing LX16A servo on port {}", port);

    // Initialize the controller with a timeout of 20ms
    let controller = LX16A::initialize(port, Duration::from_millis(20))?;

    // Create a new servo instance with ID 10
    let mut servo = LX16A::new(10, controller.clone(), false)?;

    // Display initial information
    println!("Servo ID: {}", servo.get_id(false)?);
    println!("Servo temperature: {}°C", servo.get_temp()?);
    println!("Servo voltage: {} mV", servo.get_vin()?);
    println!("Servo physical angle: {:.2}°", servo.get_physical_angle()?);

    // Example servo movement
    println!("Moving servo to position 90°...");
    servo.move_servo(90.0, 100, false, false)?;
    // servo.move_servo(90.0, 1000, false, false)?;

    // Wait for a moment
    tokio::time::sleep(Duration::from_millis(2000)).await;

    // Move back to original position
    println!("Moving servo to position 0°...");
    servo.move_servo(0.0, 100, false, false)?;
    // servo.move_servo(0.0, 1000, false, false)?;

    // Wait for a moment
    tokio::time::sleep(Duration::from_millis(2000)).await;

    println!("Demo complete!");
    Ok(())
}
