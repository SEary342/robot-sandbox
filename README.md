# Command-Based FRC Robot Code (Sandbox)

Welcome to the robot code! This project uses the **Command-Based** framework, which is the standard way to write code for FRC robots. It helps organize your code so multiple people can work on it without stepping on each other's toes.

## 📂 Project Structure (What is all this?)

Think of the code like a robot's body and brain:

*   **`robot.py`**: The **Heart**. It starts the robot and keeps the heartbeat (loop) running. You usually **don't** need to touch this file.
*   **`robotcontainer.py`**: The **Brain & Controller**. This is where you tell the robot what hardware it has (Subsystems) and what buttons make them do things (Commands).
*   **`constants.py`**: The **Settings**. All the magic numbers (Motor IDs, Speeds, PID values) go here. Change numbers here to tune the robot without breaking the logic.
*   **`subsystems/`**: The **Body Parts**. Each file here represents a physical part of the robot.
    *   `drivesubsystem.py`: Controls the wheels, encoders, and navigation.
    *   `shootersubsystem.py`: Controls the flywheel to shoot notes.
*   **`gyro/`**: Code for the **SenseHat Gyro**. This runs on a Raspberry Pi, not the RoboRIO.

---

## 🚀 How to Get Started

### 1. Check Your IDs (`constants.py`)
Open `constants.py`. Make sure the **CAN IDs** match what is physically on your robot.
*   `kLeftMotor1CAN`, `kRightMotor1CAN`, etc.
*   `kDriverControllerPort` (usually 0).

### 2. Configure the Drivetrain (`subsystems/drivesubsystem.py`)
If your robot drives backwards or spins in place when it should go straight:
*   Look at `__init__` in `DriveSubsystem`.
*   Change `l1MotorInverted`, `r1MotorInverted`, etc., to `True` or `False` until it behaves correctly.

### 3. Setup Your Controls (`robotcontainer.py`)
Open `robotcontainer.py` and look at `configureButtonBindings`.
*   This is where you map buttons (like `A`, `B`, `Bumpers`) to actions.
*   Example: `self.driverController.rightBumper().whileTrue(...)` makes the shooter run when the bumper is held.

### 4. The Gyro (Raspberry Pi)
This robot uses a Raspberry Pi with a SenseHat for its Gyroscope.
1.  Connect the Pi to the robot network.
2.  Run `gyro/gyro_service.py` on the Pi.
3.  Run `gyro_test.py` on your laptop to see if the data is coming through.

---

## 🎮 Controls

### Driver Controller (Xbox)
*   **Left Stick Y**: Drive Forward/Backward (Arcade Mode) or Left Wheels (Tank Mode).
*   **Left Stick X**: Turn Left/Right (Arcade Mode).
*   **Right Stick Y**: Right Wheels (Tank Mode).
*   **Button 'B'**: Toggle Drive Mode (Arcade vs. Tank). Swerve zeros the Gyro
*   **Button 'A'**: Toggle Shooter Calculation (Physics vs. Lookup Table).
*   **Button 'Y' (Hold)**: **Manual RPM Tuning**.
    *   Spins the shooter to the RPM value set in the "Shooter/TuningRPM" field on the SmartDashboard.
*   **Right Bumper (Hold)**: **Shoot**.
    *   Automatically aims shooter speed based on distance to the Speaker.
    *   Feeds the note only when the flywheel is at the correct speed.
*   **Left Bumper (Hold)**: **Intake**.
    *   Runs the intake rollers and spins the flywheel slowly.
    *   *Note: Shooting (Right Bumper) overrides Intaking.*
*   **D-Pad Up**: Reset Robot Position to **Blue Alliance** start.
*   **D-Pad Down**: Reset Robot Position to **Red Alliance** start.

---

## 🔧 How to Calibrate the Shooter

The robot can use a lookup table (`kShooterDistanceToRPM` in `constants.py`) to quickly set the shooter speed based on distance. To get accurate values for this table, you need to calibrate.

1.  Place the robot at a known distance from the target (e.g., 2.0 meters).
2.  Open the **SmartDashboard** on the driver station laptop.
3.  Find the text box labeled **"Shooter/TuningRPM"**.
4.  Enter a starting RPM (e.g., 2800).
5.  Press and hold the **'Y' button** on the controller. The shooter will spin up to the target RPM.
6.  Once the "Shooter/AtSpeed" indicator is true, press the **Right Bumper** to shoot a note.
7.  Observe the shot. If it's too high, lower the RPM in the dashboard. If it's too low, increase it.
8.  Repeat steps 5-7 until you find the perfect RPM for that distance.
9.  Open `constants.py` and update the `kShooterDistanceToRPM` dictionary with your new value (e.g., `2.0: 2850`).
10. Repeat the entire process for several different distances to build out the table.

This process will give you a highly accurate `kShooterDistanceToRPM` map and make your automated shooting much more reliable.

---

## 💡 Tips for Coding
*   **Read the Comments**: We've added notes in the code to explain what complex lines do.
*   **One thing at a time**: Test the drivetrain first. Then test the shooter. Don't try to do everything at once!
*   **Ask Questions**: If `PID` or `Odometry` sounds scary, that's okay! These are tools to make the robot move precisely.
