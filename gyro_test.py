import ntcore
import time

def main():
    inst = ntcore.NetworkTableInstance.getDefault()
    
    # 1. Connect to the RoboRIO (Pi and Laptop both connect here)
    # If the RoboRIO isn't on, use the Pi's IP instead: inst.setServer("10.97.21.XX")
    print("Connecting to NetworkTables...")
    #inst.setServerTeam(9721) 
    inst.setServer("192.168.8.178")
    inst.startClient4("LaptopDebugger")

    # 2. Access the table the Pi is writing to
    table = inst.getTable("SenseHat")
    yaw_topic = table.getDoubleTopic("yaw")
    rate_topic = table.getDoubleTopic("rate_z")

    # 3. Create subscribers
    yaw_sub = yaw_topic.subscribe(0.0)
    rate_sub = rate_topic.subscribe(0.0)

    print("Listening for Pi data... (Press Ctrl+C to stop)")
    
    try:
        while True:
            # Check if we are actually connected to the server
            if not inst.isConnected():
                print("Waiting for connection...", end="\r")
            else:
                yaw = yaw_sub.get()
                rate = rate_sub.get()
                print(f"NETWORK DATA RECEIVED -> Yaw: {yaw:8.2f} | Rate: {rate:8.2f}  ", end="\r")
            
            time.sleep(0.05)
    except KeyboardInterrupt:
        print("\nStopped.")

if __name__ == "__main__":
    main()