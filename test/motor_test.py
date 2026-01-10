import serial
import time

class STM32MotorControllerComms:
    def __init__ (self, port: str, baudrate: int = 115200):
        self.ser = serial.Serial(
            port=port,
            baudrate=baudrate,
            timeout=1.0,
            write_timeout=1.0
        )
        time.sleep(2)  # Wait for the serial connection to initialize

        self.ser.reset_input_buffer()
        self.ser.reset_output_buffer()

        if not self.wait_for_ping():
            raise RuntimeError("Failed to communicate with STM32 motor controller.")

    def close(self):
        self.ser.close()

    def send_command(self, cmd: str):
        line = cmd.strip() + "\n"
        print(f">>> {line.strip()}")
        self.ser.write(line.encode('utf-8'))
        self.ser.flush()

    def enable_motors(self):
        self.send_command("EN,1")
        line = self.ser.readline().decode('utf-8', errors='ignore').strip()

    def disable_motors(self):
        self.send_command("EN,0")
        line = self.ser.readline().decode('utf-8', errors='ignore').strip()

    def set_speed(self, left_rpm: int, right_rpm: int):
        self.send_command(f"SET,{left_rpm},{right_rpm}")
        line = self.ser.readline().decode('utf-8', errors='ignore').strip()

    def wait_for_ping(self, timeout_s: float = 2.0):
        seq = int(time.time() * 1000) % 10000
        t0 = time.time()
        while time.time() - t0 < timeout_s:
            self.send_command(f"PING,{seq}")
            line = self.ser.readline().decode('utf-8', errors='ignore').strip()
            if not line:
                continue
            # print(f"<<< {line}")
            if line == f"PONG,{seq}":
                return True

    def run_motor_test(self, left_rpm: int, right_rpm: int, duration_s: float):
        print("Starting motor test...")
        self.enable_motors()
        start_time = time.time()
        while time.time() - start_time < duration_s:
            self.set_speed(left_rpm, right_rpm)
            time.sleep(0.1)
        self.disable_motors()
        print("Motor test completed.")

if __name__ == "__main__":
    # serial_port = "COM3"
    serial_port = "/dev/ttyAMA0"
    baud_rate = 115200
    try:
        comms = STM32MotorControllerComms(serial_port, baud_rate)

        comms.run_motor_test(
            left_rpm=50,
            right_rpm=50,
            duration_s=5.0
        )
    finally:
        comms.close()