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
*   **Button 'B'**: Toggle Drive Mode (Arcade vs. Tank).
*   **Button 'A'**: Toggle Shooter Calculation (Physics vs. Lookup Table).
*   **Right Bumper (Hold)**: **Shoot**.
    *   Automatically aims shooter speed based on distance to the Speaker.
    *   Feeds the note only when the flywheel is at the correct speed.
*   **Left Bumper (Hold)**: **Intake**.
    *   Runs the intake rollers and spins the flywheel slowly.
    *   *Note: Shooting (Right Bumper) overrides Intaking.*
*   **D-Pad Up**: Reset Robot Position to **Blue Alliance** start.
*   **D-Pad Down**: Reset Robot Position to **Red Alliance** start.

---

## 💡 Tips for Coding
*   **Read the Comments**: We've added notes in the code to explain what complex lines do.
*   **One thing at a time**: Test the drivetrain first. Then test the shooter. Don't try to do everything at once!
*   **Ask Questions**: If `PID` or `Odometry` sounds scary, that's okay! These are tools to make the robot move precisely.
