# Setting Up the SmartDashboard / Shuffleboard

**SmartDashboard** (or the newer **Shuffleboard**) is the screen on your laptop that lets you see what the robot is doing while it's running. It's like the dashboard in a car.

## 1. How to Open It
1.  Open the **FRC Driver Station** software.
2.  Click the **Gear Icon** (Settings) on the left side.
3.  Look for the dropdown box labeled **"Dashboard Type"**.
4.  Select **"Shuffleboard"** (Recommended) or "SmartDashboard".
5.  The dashboard application should launch automatically. When your robot code connects, values will start appearing!

---

## 2. How to Send Data (The Code)
To put a number or text on the screen, use `SmartDashboard.put...` commands in your Python code.

### Example: Shooter Speed
In `subsystems/shootersubsystem.py`, we send the RPM to the dashboard:
```python
SmartDashboard.putNumber("Shooter/CurrentRPM", self.encoder.getVelocity())
```
*   **"Shooter/CurrentRPM"**: This is the *Name* (Key) that appears on the screen. Using a slash `/` groups things into folders (like a "Shooter" folder).
*   **self.encoder.getVelocity()**: This is the *Value* (the actual number).

---

## 3. Special Widgets

### The Field Map
In `subsystems/drivesubsystem.py`, we send a `Field2d` object.
*   **In Code**: `SmartDashboard.putData("Field", self.field)`
*   **On Screen**: Drag the "Field" value onto the layout. It shows a map of the field and a little robot icon that moves as you drive!

### Auto Chooser
In `robotcontainer.py`, we send the Auto Chooser.
*   **In Code**: `wpilib.SmartDashboard.putData("Chosen Auto", self.chosenAuto)`
*   **On Screen**: This creates a dropdown menu that lets you pick which Autonomous routine to run before the match starts.

---

## 4. Tuning Numbers (PID)
You can also *change* numbers on the dashboard and have the robot read them. This is great for tuning PID without redeploying code.

1.  **Send a default value**: `SmartDashboard.setDefaultNumber("ShooterP", 0.0005)`
2.  **Read it back**: `p = SmartDashboard.getNumber("ShooterP", 0.0005)`
3.  **Tune**: Type a new number into the box on the dashboard, and the robot will use it instantly!