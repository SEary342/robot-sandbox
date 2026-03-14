# Command-Based FRC Robot Code (Sandbox)

Welcome to the robot code! This project uses the **Command-Based** framework, which is the standard way to write code for FRC robots. It helps organize your code so multiple people can work on it without stepping on each other's toes.

## 📂 Project Structure (What is all this?)

Think of the code like a robot's body and brain:

*   **`robot.py`**: The **Heart**. It starts the robot and keeps the heartbeat (loop) running. You usually **don't** need to touch this file.
*   **`robotcontainer.py`**: The **Brain & Controller**. This is where you tell the robot what hardware it has (Subsystems) and what buttons make them do things (Commands).
*   **`constants.py`**: The **Master Settings**. Toggle `kSwerveInstalled` here to switch between Tank and Swerve modes.
*   **`tank_constants.py`**, **`swerve_constants.py`**, **`shooter_constants.py`**: Modular settings for each system.
*   **`subsystems/`**: The **Body Parts**.
    *   `drivesubsystem.py`: Classic Tank/Arcade drive.
    *   `swervedrivesubsystem.py`: Independent 4-wheel Swerve drive (REV Ion / SparkMax).
    *   `shootersubsystem.py`: Controls the flywheel to shoot notes (includes Physics & Lookup models).

---

## 🚀 How to Get Started

### 1. Choose Your Drivetrain (`constants.py`)
Open `constants.py`. Set `kSwerveInstalled = True` for Swerve or `False` for Tank. This will automatically load the correct CAN IDs and motor configurations.

### 2. Check Your IDs
Make sure the **CAN IDs** in `tank_constants.py` or `swerve_constants.py` match your physical hardware.
*   The **Drive Motors** are set to **Coast Mode** by default to prevent lurching.
*   The **Swerve Turning Motors** remain in **Brake Mode** for steering precision.

### 3. Setup Your Controls (`robotcontainer.py`)
Open `robotcontainer.py` and look at `configureButtonBindings`.
*   This is where you map buttons (like `A`, `B`, `Bumpers`) to actions.

---

## 🎮 Controls

### Driver Controller (Xbox)

#### 🏎️ Driving (Changes based on `kSwerveInstalled`)
*   **Swerve Mode**:
    *   **Left Stick**: Move Robot (Field-Relative by default).
    *   **Right Stick X**: Rotate Robot.
    *   **Button 'B'**: Toggle Field-Relative vs. Robot-Relative.
*   **Tank Mode**:
    *   **Left Stick Y**: Forward/Backward (Arcade) or Left Wheels (Tank).
    *   **Left Stick X**: Turn (Arcade).
    *   **Right Stick Y**: Right Wheels (Tank).
    *   **Button 'B'**: Toggle Arcade vs. Tank Drive.

#### 🎯 Automation & Shooting
*   **Button 'X' (Hold)**: **Aim at Target**. 
    *   Automatically rotates the robot to face the closest tower AprilTag. Works in both Tank and Swerve!
*   **Right Bumper (Hold)**: **Shoot**.
    *   Automatically calculates shooter speed based on distance using the `LookupTable`.
    *   Check `Shooter/Status` on the dashboard to see why it's not firing.
*   **Left Bumper (Hold)**: **Intake**.
    *   Runs the intake rollers. *Note: Shooting overrides Intaking.*
*   **Button 'A'**: Toggle Shooter Logic (Pure Physics vs. Lookup Table).
*   **Button 'Y' (Hold)**: **Manual RPM Tuning** (Uses "Shooter/TuningRPM" from Dashboard).

#### 📍 Navigation
*   **D-Pad Up**: Reset Robot Position to **Blue Alliance** start.
*   **D-Pad Down**: Reset Robot Position to **Red Alliance** start.

---

## 🔧 How to Calibrate the Shooter

The robot uses a `LookupTable` in `shooter_constants.py` for precision shooting. 

1.  Place the robot at a known distance (e.g., 3.0 meters).
2.  Set **"Shooter/TuningRPM"** on the **SmartDashboard** (e.g., 3500).
3.  Hold **'Y'** to spin up and **Right Bumper** to test the shot.
4.  If the shot is good, update the values in `shooter_constants.py`:
    ```python
    kShooterDistanceToRPM = LookupTable({
        1.5: 2500,
        3.0: 3550, # Updated value
        ...
    })
    ```

## 💡 Tips
*   **Coast Mode**: If the robot feels too "floaty," you can change the motors back to `kBrake` in the constants files.
*   **Swerve Alignment**: If a wheel is facing the wrong way, check the `turnMotorInverted` settings in `subsystems/swervedrivesubsystem.py`.
