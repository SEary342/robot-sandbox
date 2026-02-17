# How to Set Up Multiple PhotonVision Cameras

Using two or more cameras is a great way to improve your robot's vision. It allows the robot to see more of the field at once, which makes its position estimate (odometry) much more accurate and reliable.

This guide will walk you through setting up a second camera.

---

## Step 1: Hardware & PhotonVision GUI

1.  **Mount the Cameras**: Physically mount two cameras to your robot. Try to point them in slightly different directions to maximize how much they can see.
2.  **Connect Them**: Plug both cameras into your vision coprocessor (e.g., Raspberry Pi, Orange Pi).
3.  **Name Them**: Open the PhotonVision web dashboard (usually at `http://photonvision.local:5800`).
    *   Go to the "Settings" tab.
    *   In the "Camera" section, give each camera a **unique name**. This is very important!
    *   Good names are simple, like `Camera_Left` and `Camera_Right`.
    *   Write these names down. You will need them for the code.

## Step 2: Update `constants.py`

Next, we need to tell the code about our new cameras and where they are on the robot.

1.  Open `constants.py`.
2.  Find the `# --- Camera Constants ---` section.
3.  Replace the old single-camera constants with a block for each camera you have.

```python
# --- Camera Constants ---
# MEASURE: For each camera, measure its position relative to the center of the robot.
# X is forward, Y is left, Z is up.

# Camera 1 Constants (e.g., the left camera)
kCamera1Name = "Camera_Left" # CHECK: Must match the name in the PhotonVision UI
kCamera1OffsetX = 0.2  # meters
kCamera1OffsetY = 0.1  # meters
kCamera1Height = 0.5   # meters
kCamera1Pitch = math.radians(-30.0)

# Camera 2 Constants (e.g., the right camera)
kCamera2Name = "Camera_Right" # CHECK: Must match the name in the PhotonVision UI
kCamera2OffsetX = 0.2  # meters
kCamera2OffsetY = -0.1 # meters
kCamera2Height = 0.5   # meters
kCamera2Pitch = math.radians(-30.0)
```

## Step 3: Update `subsystems/drivesubsystem.py`

Finally, we'll update the drivetrain code to initialize and use both cameras. The code is designed to handle any number of cameras you add in `constants.py`.

1.  Open `subsystems/drivesubsystem.py`.
2.  In the `__init__` method, the code now loops through your cameras and creates the necessary objects. You just need to make sure you have the constants defined.
3.  In the `periodic` method, the code automatically loops through all vision sources, gets their data, and feeds it to the robot's main pose estimator.

The changes in the diffs above make this process modular. To add a third camera, you would simply add `kCamera3...` constants and uncomment/add another setup block in `drivesubsystem.py`'s `__init__`.
