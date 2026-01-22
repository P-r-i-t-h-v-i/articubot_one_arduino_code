Absolutely — here’s a clean, professional README specifically for the Arduino / ESP32 side of your project.
This is written in a way that GitHub reviewers, seniors, and recruiters understand immediately.

You can copy-paste this directly into README.md in your Arduino/ESP32 repo.

ESP32 Firmware – ROS 2 Wi-Fi Mobile Robot

This repository contains the ESP32 (Arduino) firmware for a ROS 2–integrated mobile robot.
The ESP32 handles low-level motor control and ultrasonic sensing, and communicates with a ROS 2 system over Wi-Fi using a lightweight UDP protocol.

This firmware is designed to work alongside a ROS 2 Python bridge node running on a PC.

📌 Features

ESP32-based differential drive control

Wi-Fi communication using WiFi.h and UDP

Receives velocity commands (linear, angular) from ROS 2

Reads ultrasonic distance sensor data

Streams sensor data back to ROS 2 in real time

Simple, reliable design (no micro-ROS, no PWM/ledc dependency)

🧠 System Architecture (Overview)
ROS 2 (PC)
 ├─ teleop_twist_keyboard
 ├─ WiFi bridge node (UDP)
 └─ RViz / Gazebo
        ↑        ↓
        Wi-Fi (UDP)
        ↓        ↑
ESP32
 ├─ Motor Driver (L298N)
 └─ Ultrasonic Sensor


The ESP32 does not run ROS.
It acts as a hardware endpoint that exchanges data with ROS 2 over Wi-Fi.

🔌 Hardware Used

ESP32 Development Board

L298N Motor Driver

2 × DC Motors (Differential Drive)

Ultrasonic Sensor (HC-SR04 or equivalent)

Battery Pack (for motors)

🔗 Pin Connections
Motor Driver (L298N)
L298N Pin	ESP32 GPIO
IN1	GPIO 25
IN2	GPIO 26
IN3	GPIO 27
IN4	GPIO 14
ENA	Jumper ON / tied to 5V
ENB	Jumper ON / tied to 5V

⚠️ ENA and ENB are kept HIGH for constant speed operation.

Ultrasonic Sensor
Sensor Pin	ESP32 GPIO
TRIG	GPIO 12
ECHO	GPIO 13 (via voltage divider)

⚠️ Important: Use a voltage divider on the ECHO pin (5V → 3.3V) to protect the ESP32.

🌐 Wi-Fi Configuration

Edit the following lines in the Arduino code:

const char* ssid = "YOUR_WIFI_SSID";
const char* password = "YOUR_WIFI_PASSWORD";
const char* pc_ip = "PC_IP_ADDRESS";  // e.g. 192.168.43.10


ESP32 and PC must be on the same Wi-Fi network

Mobile hotspot is recommended for reliable communication

📡 Communication Protocol
From ROS 2 → ESP32

Velocity commands sent as a UDP string:

linear_velocity,angular_velocity


Example:

0.5,0.0

From ESP32 → ROS 2

Ultrasonic distance sent as a UDP string (meters):

0.42

▶️ How to Run

Flash the Arduino code to the ESP32

Power the motor driver with a battery

Connect ESP32 and PC to the same Wi-Fi network

Start the ROS 2 Wi-Fi bridge node on the PC

Use teleop_twist_keyboard to control the robot

The robot will:

Move based on ROS 2 teleop commands

Continuously stream ultrasonic distance data back to ROS 2

🧪 Debugging Tips

Open Serial Monitor (115200 baud) to verify:

Wi-Fi connection

Incoming UDP commands

Ultrasonic readings

If motors do not move:

Check ENA / ENB jumpers

Ensure common ground between ESP32 and L298N

Verify motor power supply

🚀 Future Improvements

Add PWM speed control (ENA / ENB)

Integrate IMU feedback

Add servo-mounted ultrasonic for wider scan

Replace ultrasonic with LiDAR

Full SLAM using ROS 2 Nav2 stack

📄 License

This project is open-source and intended for learning and educational use.

🙌 Acknowledgements

Built as part of a ROS 2 learning project focused on hardware–software integration, sensor visualization, and system-level robotics design.
