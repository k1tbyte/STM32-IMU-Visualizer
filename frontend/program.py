import pygame
from pygame.locals import *
from OpenGL.GL import *
from OpenGL.GLU import *
import serial
import json
import math
import time
import numpy as np
from collections import deque

SERIAL_PORT = "COM3"
BAUD_RATE = 115200
CALIBRATION_SAMPLES = 200
MOVING_AVG_WINDOW = 10
ACCEL_CONVERSION = 0.000598  # м/с²/LSB for ±2g
GYRO_CONVERSION = 0.00381    # °/с/LSB for ±125°/с

class IMUProcessor: 
    def __init__(self):
        self.yaw = 0
        self.last_time = time.time()
        
        self.gyro_offset = np.array([0., 0., 0.])
        self.accel_offset = np.array([0., 0., 0.])
        
        self.gyro_buffer = deque(maxlen=MOVING_AVG_WINDOW)
        self.accel_buffer = deque(maxlen=MOVING_AVG_WINDOW)
        
        # Complementary filter
        self.alpha = 0.96
        self.filtered_angles = np.array([0., 0., 0.])
        self.zero_orientation = np.array([0., 0., 0.])
        
        self.is_calibrated = False
        
        self.gyro_history = []
        self.accel_history = []

    def collect_calibration_data(self, ser):
        print("Collecting calibration data. Keep the sensor still...")
        self.gyro_history = []
        self.accel_history = []
        
        for _ in range(CALIBRATION_SAMPLES):
            try:
                serial_line = ser.readline().decode().strip()
                data = json.loads(serial_line)
                
                gyro = [float(data["Gx"]), float(data["Gy"]), float(data["Gz"])]
                accel = [float(data["Ax"]), float(data["Ay"]), float(data["Az"])]
                
                self.gyro_history.append(gyro)
                self.accel_history.append(accel)
                
            except Exception as e:
                print(f"Error collecting calibration data: {e}")
                continue
            
            if len(self.gyro_history) % 100 == 0:
                print(f"Collected {len(self.gyro_history)}/{CALIBRATION_SAMPLES} samples")

    def calculate_calibration(self):
        try:
            gyro_data = np.array(self.gyro_history)
            accel_data = np.array(self.accel_history)

            # Calculate the average value for the gyroscope and accelerometer
            self.gyro_offset = np.mean(gyro_data, axis=0)
            
            # For the accelerometer, we take into account that one axis should show 1g (16384)
            gravity_axis = np.argmax(np.abs(np.mean(accel_data, axis=0)))
            expected_gravity = 16384 * np.sign(np.mean(accel_data[:, gravity_axis]))
            self.accel_offset = np.mean(accel_data, axis=0)
            self.accel_offset[gravity_axis] -= expected_gravity

            self.is_calibrated = True
            self.filtered_angles = np.array([0., 0., 0.])
            self.zero_orientation = np.array([0., 0., 0.])
            
            print("\nCalibration Results:")
            print(f"Gyro Offset: {self.gyro_offset}")
            print(f"Accel Offset: {self.accel_offset}")
            
        except Exception as e:
            print(f"Error calculating calibration: {e}")
            self.is_calibrated = False

    def calibrate(self, ser):
        self.collect_calibration_data(ser)
        if len(self.gyro_history) > 0:
            self.calculate_calibration()
            print("Calibration complete!")
        else:
            print("Calibration failed - no data collected")

    def apply_moving_average(self, new_values, buffer):
        """Applies a moving average to new values"""
        buffer.append(new_values)
        return np.mean(buffer, axis=0)

    def process_data(self, serial_line):
        try:
            data = json.loads(serial_line)
            
            # Raw data from MC float
            gyro = np.array([float(data["Gx"]), float(data["Gy"]), float(data["Gz"])])
            accel = np.array([float(data["Ax"]), float(data["Ay"]), float(data["Az"])])
            
            # Apply calibration
            if self.is_calibrated:
                gyro = gyro - self.gyro_offset
                accel = accel - self.accel_offset
            
            # Apply anti-aliasing
            gyro = self.apply_moving_average(gyro, self.gyro_buffer)
            accel = self.apply_moving_average(accel, self.accel_buffer)
            
            # Convert from raw
            gyro_dps = gyro * GYRO_CONVERSION
            accel_g = accel * ACCEL_CONVERSION
            
            # Calculate angles by accelerometer (Pitch/ Roll)
            roll_acc = math.atan2(accel_g[1], accel_g[2]) * 180 / math.pi
            pitch_acc = math.atan2(-accel_g[0], math.sqrt(accel_g[1]**2 + accel_g[2]**2)) * 180 / math.pi
            
            # Calculate the change of angles by gyroscope
            current_time = time.time()
            dt = current_time - self.last_time
            self.last_time = current_time

            # Limit delta time to avoid large jumps
            dt = min(dt, 0.1)
            
            # Integration of angular velocity with drift correction Filtered Angle=α⋅(Gyro Angle+Δθ)+(1-α)⋅Acc Angle
            delta_angles = gyro_dps * dt
            self.filtered_angles[0] = self.alpha * (self.filtered_angles[0] + delta_angles[0]) + (1 - self.alpha) * roll_acc
            self.filtered_angles[1] = self.alpha * (self.filtered_angles[1] + delta_angles[1]) + (1 - self.alpha) * pitch_acc
            # yaw
            self.filtered_angles[2] = self.filtered_angles[2] + delta_angles[2]

            # Normalize angles in the range -180 to 180
            self.filtered_angles = np.array([
                ((angle + 180) % 360) - 180 for angle in self.filtered_angles
            ])
            
            return (
                accel[0], accel[1], accel[2],
                gyro[0], gyro[1], gyro[2],
                self.filtered_angles[0],  # roll
                self.filtered_angles[1],  # pitch
                self.filtered_angles[2]   # yaw
            )
            
        except Exception as e:
            print(f"Error processing data: {e}")
            return 0, 0, 0, 0, 0, 0, 0, 0, 0

def draw_rect():
    vertices = (
        (3, -.2, -1),
        (3, .2, -1),
        (-3, .2, -1),
        (-3, -.2, -1),
        (3, -.2, 1),
        (3, .2, 1),
        (-3, .2, 1),
        (-3, -.2, 1)
    )

    edges = (
        (0, 1),
        (0, 3),
        (0, 4),
        (2, 1),
        (2, 3),
        (2, 6),
        (5, 1),
        (5, 4),
        (5, 6),
        (7, 3),
        (7, 4),
        (7, 6)
    )

    glBegin(GL_LINES)
    for edge in edges:
        for vertex in edge:
            glVertex3fv(vertices[vertex])
    glEnd()

def draw_text(text, x, y, color=(255, 255, 0)):
    font = pygame.font.SysFont("Courier New", 15, True)
    text_surface = font.render(text, True, color, (0, 0, 0, 255))
    text_data = pygame.image.tostring(text_surface, "RGBA", True)
    
    glPushMatrix()
    glLoadIdentity()
    glRasterPos2f(
        -1 + (x / pygame.display.get_surface().get_width()) * 2,
        1 - (y / pygame.display.get_surface().get_height()) * 2
    )
    glDrawPixels(
        text_surface.get_width(), 
        text_surface.get_height(), 
        GL_RGBA, 
        GL_UNSIGNED_BYTE, 
        text_data
    )
    glPopMatrix()

def calculate_acceleration_stats(Ax, Ay, Az):
    g2ms_square = 9.81 / 16384 

    Ax_ms2 = Ax * g2ms_square
    Ay_ms2 = Ay * g2ms_square
    Az_ms2 = Az * g2ms_square

    max_accel = max((Ax_ms2, "Ax"), (Ay_ms2, "Ay"), (Az_ms2, "Az"), key=lambda x: abs(x[0]))
    min_accel = min((Ax_ms2, "Ax"), (Ay_ms2, "Ay"), (Az_ms2, "Az"), key=lambda x: abs(x[0]))
    
    total_accel = math.sqrt(Ax_ms2**2 + Ay_ms2**2 + Az_ms2**2)
    
    return max_accel, min_accel, total_accel

def main():
    pygame.init()
    display = (800, 600)
    screen = pygame.display.set_mode(display, DOUBLEBUF | OPENGL)
    gluPerspective(70, (display[0] / display[1]), 0.1, 50.0)
    glTranslatef(0.0, 0.0, -5)

    ser = serial.Serial(SERIAL_PORT, BAUD_RATE)
    imu_processor = IMUProcessor()
    
    imu_processor.calibrate(ser)

    while True:
        for event in pygame.event.get():
            if event.type == pygame.QUIT:
                ser.close()
                pygame.quit()
                quit()
            elif event.type == pygame.KEYDOWN:
                if event.key == pygame.K_c:
                    #  Recalibration
                    imu_processor.calibrate(ser)

        serial_line = ser.readline().decode().strip()
        Ax, Ay, Az, Gx, Gy, Gz, roll, pitch, yaw = imu_processor.process_data(serial_line)
        max_accel, min_accel, total_accel = calculate_acceleration_stats(Ax, Ay, Az)

        glClear(GL_COLOR_BUFFER_BIT | GL_DEPTH_BUFFER_BIT)
        glPushMatrix()

        glRotatef(yaw, 0, 1, 0)
        glRotatef(roll, 0, 0, 1)
        glRotatef(pitch, 1, 0, 0)

        draw_rect()
        glPopMatrix()

        # Debug info
        draw_text(f"Roll: {roll:.2f}, Pitch: {pitch:.2f}, Yaw: {yaw:.2f}", 10, 20, (255, 0, 255))
        draw_text(f"Raw acceleration -> Ax: {Ax:.2f}, Ay: {Ay:.2f}, Az: {Az:.2f}", 10, 40)
        draw_text(f"Raw orientation -> Gx: {Gx:.2f}, Gy: {Gy:.2f}, Gz: {Gz:.2f}", 10, 60)
        draw_text(f"Max Acc: {max_accel[1]} ({max_accel[0]:.2f} m/s^2)", 10, 80, (255, 0, 0))
        draw_text(f"Min Acc: {min_accel[1]} ({min_accel[0]:.2f} m/s^2)", 10, 100, (0, 100, 255))
        draw_text(f"Total Acc: {total_accel:.2f} m/s^2", 10, 120, (0, 255, 0))
        draw_text("Press 'C' to recalibrate", 10, 140, (255, 255, 255))

        pygame.display.flip()
        pygame.time.wait(10)

if __name__ == "__main__":
    main()