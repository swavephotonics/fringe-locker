# **Differential Fringe Locker**

A Python-based active stabilization system for optical interferometers. This software locks interference fringes in place by reading a live feed from a Basler camera and sending feedback voltage to a piezo-mounted mirror via a Thorlabs MDT693B controller.  
It uses a **Differential Locking** algorithm (Balanced Detection) to reject common-mode noise (like laser power fluctuations) and creates a robust, drift-free lock.

## 

## **Features**

* **Differential Locking:** Uses two Regions of Interest (ROIs) on opposite slopes of a fringe to calculate a normalized error signal $(A-B)/(A+B)$.  
* **Dynamic Setpoint:** "Locks where it sits." The system captures the current fringe position as the target when you engage the lock, preventing jumps.  
* **Real-Time Tuning:** Adjust P (Proportional) and I (Integral) gains on the fly via the GUI.  
* **High-Speed Loop:** Optimized "Fire-and-Forget" serial commands to maximize bandwidth.  
* **Hardware Safety:** Includes bandwidth throttling to prevent serial buffer overflows and controller freezes.

## 

## **Hardware Requirements**

1. **Piezo Controller:** Thorlabs MDT693B (Open-Loop Voltage Control).  
2. **Camera:** Basler USB3 Camera (using Pylon drivers).  
3. **Actuator:** Piezo-mounted mirror in one arm of the interferometer.  
4. **Computer:** Windows/Linux with Python 3.x.

## 

## **Installation**

### **1\. Install Drivers**

* **Basler Camera:** Install the [pylon Camera Software Suite](https://www.baslerweb.com/en-us/downloads/software/).  
* **Thorlabs Controller:** Ensure the MDT693B is connected via USB and shows up as a COM port in Device Manager.

### **2\. Install Python Dependencies**

Run the following command to install the required libraries:

```
pip install pypylon pyserial opencv-python numpy pyqt5 pyqtgraph
```

## **Configuration**

Before running the script, open the code and check the **Main Execution** block at the bottom:

1. **COM Port:** Update the port string to match your system.  
```
  piezo = MDT693BController(port='COM3') # Change 'COM3' to your COM port
```

2. **Exposure Time:** Adjust camera exposure time in ms.  

```
  cam.set_exposure(10) # Set to 10 ms
```

## **Usage Guide**

### **1\. Running the System**

If running from a terminal:

```
python fringe_locker.py
```

*Note: If running in Jupyter Notebook, ensure you include %gui qt at the top of the first cell to prevent the window from freezing.*

### **2\. The Workflow**

1. **Preview:** Click the PREVIEW button. The camera feed will appear.  
2. **Align ROIs:**  
   * Drag the **Green Box (+)** to the "rising" slope of a dark fringe (mid-gray).  
   * Drag the **Red Box (-)** to the "falling" slope of the same (or adjacent) fringe.  
   * *Tip:* Try to place them so they have roughly equal intensity.  
3. **Engage Lock:** Click LOCK.  
   * The system measures the current balance.  
   * A **Green Dashed Line** will appear on the graph indicating the setpoint.  
   * The PID loop engages to hold the signal at that line.  
4. **Tune:**  
   * Adjust **Kp** (Proportional) until the lock reacts fast enough to vibrations.  
   * Adjust **Ki** (Integral) to remove slow drifts.  
   * *Start small:* Kp \= \-0.05, Ki \= \-0.005.

## **Troubleshooting**

| Issue | Cause | Solution |
| :---- | :---- | :---- |
| **GUI Freezes** | Serial buffer overflow. | The code has a built-in rate limiter (2ms). Do not decrease self.write\_interval below 0.002. |
| **"White Flash" / Fly-away** | Camera buffer backlog. | The start\_lock routine automatically flushes the buffer. Wait 0.5s after clicking lock for it to settle. |
| **Piezo Rails (0V or 150V)** | Drift is too large. | Click RESET to center the piezo at 75V, re-align your optical fringes manually, then lock again. |

## **Code Structure**

* **MDT693BController:** Handles serial communication. Implements "Fire-and-Forget" logic and truncates floats to save USB bandwidth.  
* **PylonCamera:** Wraps the Basler API. Uses a Hardware ROI (Region of Interest) to center the image and increase frame rate.  
* **DifferentialLockerWindow:** The PyQt5 GUI class. Handles plotting and ROI interaction.  
* **DifferentialLocker:** The PID logic core. Runs the main control loop.
