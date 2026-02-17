import time
import math
import ntcore
from ICM20948 import ICM20948 
# Ensure ICM20948.py is in the same folder

def run_service():
    # 1. NetworkTables 4 Setup
    inst = ntcore.NetworkTableInstance.getDefault()
    inst.setServerTeam(1234) # Replace with your team number
    inst.startClient4("SenseHat_Pi")
    
    table = inst.getTable("SenseHat")
    yaw_pub = table.getDoubleTopic("yaw").publish()
    rate_pub = table.getDoubleTopic("rate_z").publish()

    # 2. Initialize Waveshare IMU
    # This will trigger the 2-second calibration (KEEP ROBOT STILL)
    imu = ICM20948()
    
    # Waveshare uses global variables in their file for data, 
    # but we can access them through the module if needed.
    # To keep it safe, we'll re-run their logic locally.
    import ICM20948 as ws_vars

    print("Gyro Service Running...")

    while True:
        try:
            # Update internal hardware registers
            imu.icm20948_Gyro_Accel_Read()
            imu.icm20948MagRead()

            if sum(ws_vars.Accel) == 0:
                print("Waiting for valid IMU data...")
                time.sleep(0.1)
                continue

            # Scale raw data to Degrees Per Second (32.8 is the scale factor for 1000DPS)
            # Using the global arrays defined in your ICM20948.py
            gz_dps = ws_vars.Gyro[2] / 32.8

            # Update the AHRS Filter (Math that turns rate into orientation)
            # Constants: 0.0175 converts degrees to radians
            imu.imuAHRSupdate(
                ws_vars.Gyro[0]/32.8 * 0.0175, ws_vars.Gyro[1]/32.8 * 0.0175, ws_vars.Gyro[2]/32.8 * 0.0175,
                ws_vars.Accel[0], ws_vars.Accel[1], ws_vars.Accel[2],
                ws_vars.Mag[0], ws_vars.Mag[1], ws_vars.Mag[2]
            )

            # Calculate Yaw for PathPlanner (-180 to 180 degrees)
            # This uses the Quaternions (q0-q3) updated by the AHRS filter
            yaw = math.atan2(-2 * ws_vars.q1 * ws_vars.q2 - 2 * ws_vars.q0 * ws_vars.q3, 
                             2 * ws_vars.q2 * ws_vars.q2 + 2 * ws_vars.q3 * ws_vars.q3 - 1) * 57.3

            # 3. Send to RoboRIO
            yaw_pub.set(yaw)
            rate_pub.set(gz_dps)

            # 50Hz update rate
            time.sleep(0.02)

        except Exception as e:
            print(f"Service Error: {e}")
            time.sleep(0.5)

if __name__ == "__main__":
    run_service()