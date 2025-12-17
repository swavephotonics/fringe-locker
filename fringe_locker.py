import sys
import time
import serial
import numpy as np
import pyqtgraph as pg
from PyQt5 import QtWidgets, QtCore
from pypylon import pylon

# ==========================================
# 1. HELPER FUNCTIONS
# ==========================================

def get_qt_app():
    """Ensures only one instance of QApplication exists."""
    app = QtWidgets.QApplication.instance()
    if app is None:
        app = QtWidgets.QApplication(sys.argv)
    return app

# ==========================================
# 2. HARDWARE DRIVERS
# ==========================================

class MDT693BController:
    """
    Driver for Thorlabs MDT693B Piezo Controller.
    Includes BANDWIDTH PROTECTION (Rate Limiting + Truncation).
    """
    def __init__(self, port, timeout=0.1):
        self.port = port
        self.baudrate = 115200
        self.timeout = timeout
        self.ser = None
        self.last_write_time = 0.0
        self.write_interval = 0.002 # Max 500 commands/sec to prevent freeze

        try:
            self.ser = serial.Serial(
                port=self.port,
                baudrate=self.baudrate,
                bytesize=serial.EIGHTBITS,
                parity=serial.PARITY_NONE,
                stopbits=serial.STOPBITS_ONE,
                timeout=self.timeout,
                write_timeout=0
            )
            self.ser.reset_input_buffer()
            self.ser.reset_output_buffer()
            print(f"[{self.port}] Connected to MDT693B Controller.")
        except serial.SerialException as e:
            print(f"[{self.port}] Connection Error: {e}")

    def _send_command(self, command, expect_response=False):
        if not self.ser or not self.ser.is_open: return None
        
        # 1. Rate Limiter
        now = time.time()
        if not expect_response and (now - self.last_write_time) < self.write_interval:
            return None
        self.last_write_time = now

        # 2. Input Flush (Prevent buffer clog)
        if not expect_response: self.ser.reset_input_buffer()
        
        # 3. Write
        try:
            full_command = f"{command}\r"
            self.ser.write(full_command.encode('ascii'))
        except serial.SerialTimeoutException:
            self.ser.reset_output_buffer()
            return None

        if not expect_response: return None

        # 4. Read (Only if needed)
        response = b""; start_t = time.time()
        while (time.time() - start_t) < self.timeout:
            if self.ser.in_waiting:
                char = self.ser.read()
                if char == b'>': break
                response += char
        return response.decode('ascii').strip()

    def set_voltage(self, axis, voltage):
        # Truncate to 2 decimals to save USB bandwidth
        self._send_command(f"{axis}voltage={voltage:.2f}", expect_response=False)

    def close(self):
        if self.ser and self.ser.is_open:
            self.ser.close()
            print("Piezo Controller Disconnected.")


class PylonCamera:
    """Wrapper for Basler Camera with Hardware ROI."""
    def __init__(self):
        self.camera = None; self.is_open = False; self.is_grabbing = False

    def open(self, roi_size=(1000, 1000)):
        try:
            tl_factory = pylon.TlFactory.GetInstance()
            if not tl_factory.EnumerateDevices(): print("Error: No camera found."); return False

            self.camera = pylon.InstantCamera(tl_factory.CreateFirstDevice())
            self.camera.Open(); self.is_open = True
            
            self.camera.OffsetX.SetValue(0); self.camera.OffsetY.SetValue(0)
            self.camera.Width.SetValue(self.camera.Width.GetMax())
            self.camera.Height.SetValue(self.camera.Height.GetMax())

            self._configure_hardware_roi(*roi_size)
            self.camera.StartGrabbing(pylon.GrabStrategy_LatestImageOnly)
            self.is_grabbing = True
            print(f"Camera Connected: {self.camera.GetDeviceInfo().GetModelName()}")
            return True
        except Exception as e:
            print(f"Camera Error: {e}"); self.is_open = False; return False

    def _configure_hardware_roi(self, w, h):
        try:
            max_w = self.camera.Width.GetMax(); max_h = self.camera.Height.GetMax()
            w = (min(w, max_w) // 16) * 16; h = (min(h, max_h) // 16) * 16
            off_x = ((max_w - w) // 2 // 4) * 4; off_y = ((max_h - h) // 2 // 4) * 4
            self.camera.Width.SetValue(w); self.camera.Height.SetValue(h)
            self.camera.OffsetX.SetValue(off_x); self.camera.OffsetY.SetValue(off_y)
            print(f"ROI Set: {w}x{h}")
        except Exception as e: print(f"ROI Config Failed: {e}")

    def set_exposure(self, exp_ms):
        if self.is_open:
            try:
                self.camera.ExposureAuto.SetValue("Off")
                self.camera.ExposureTime.SetValue(float(exp_ms * 1000))
            except: pass

    def take_image(self, timeout_ms=500):
        if not self.is_grabbing: return None
        try:
            res = self.camera.RetrieveResult(timeout_ms, pylon.TimeoutHandling_Return)
            if res.GrabSucceeded():
                img = res.GetArray(); res.Release(); return img
            res.Release()
        except: pass
        return None

    def close(self):
        if self.is_grabbing: self.camera.StopGrabbing()
        if self.is_open: self.camera.Close()
        print("Camera Closed.")


# ==========================================
# 3. GUI CLASS (UPDATED FOR SETPOINT LINE)
# ==========================================

class DifferentialLockerWindow:
    def __init__(self):
        self.app = get_qt_app()
        self.win = QtWidgets.QMainWindow()
        self.win.setWindowTitle("Differential Lock Command Center")
        self.win.resize(1000, 900)
        
        self.shutdown_callback = None
        self.win.closeEvent = self.closeEvent
        
        cw = QtWidgets.QWidget(); self.win.setCentralWidget(cw)
        self.layout = QtWidgets.QVBoxLayout(); cw.setLayout(self.layout)
        
        self.glw = pg.GraphicsLayoutWidget(); self.layout.addWidget(self.glw)
        
        # Plot 1: Camera
        self.p_img = self.glw.addPlot(row=0, col=0)
        self.p_img.setAspectLocked(True); self.p_img.invertY(True)
        self.img_item = pg.ImageItem(); self.p_img.addItem(self.img_item)
        
        self.roi_plus = pg.RectROI([100, 100], [80, 80], pen=pg.mkPen('g', width=3))
        self.roi_plus.addScaleHandle([1, 1], [0, 0]); self.p_img.addItem(self.roi_plus)
        
        self.roi_minus = pg.RectROI([300, 100], [80, 80], pen=pg.mkPen('r', width=3))
        self.roi_minus.addScaleHandle([1, 1], [0, 0]); self.p_img.addItem(self.roi_minus)

        # Plot 2: Balance Error
        self.glw.nextRow()
        self.p_line = self.glw.addPlot(row=1, col=0)
        self.p_line.setLabel('left', "Error (%)"); self.p_line.setYRange(-50, 50)
        self.p_line.showGrid(x=True, y=True, alpha=0.5)
        
        self.trace_curve = self.p_line.plot(pen='y')
        
        # --- MOVABLE TARGET LINE ---
        self.target_line = pg.InfiniteLine(angle=0, movable=False, 
                                           pen=pg.mkPen('g', width=2, style=QtCore.Qt.DashLine),
                                           label='Target', 
                                           labelOpts={'position':0.1, 'color': (0,255,0), 'movable': True})
        self.p_line.addItem(self.target_line)
        self.win.show()

    def set_target_line(self, val):
        """Updates the visual green line to the new setpoint."""
        self.target_line.setPos(val)
        if hasattr(self.target_line, 'label') and self.target_line.label:
            self.target_line.label.setText(f"Target: {val:.1f}%")

    def closeEvent(self, event):
        if self.shutdown_callback: self.shutdown_callback()
        event.accept(); self.app.quit()

    def get_balance_error(self, frame):
        """Returns Balance %: (G - R) / Total"""
        if frame is None: return 0.0
        self.img_item.image = frame.transpose((1, 0))
        s_plus = np.sum(self.roi_plus.getArrayRegion(frame, img=self.img_item))
        s_minus = np.sum(self.roi_minus.getArrayRegion(frame, img=self.img_item))
        total = s_plus + s_minus
        if total == 0: return 0.0
        return ((s_plus - s_minus) / total) * 100.0

    def update_display(self, frame, trace_data):
        if frame is not None: self.img_item.setImage(frame.transpose((1, 0)))
        if trace_data is not None: self.trace_curve.setData(trace_data)
        self.app.processEvents()


# ==========================================
# 4. LOGIC CLASS (DYNAMIC SETPOINT)
# ==========================================

class DifferentialLocker:
    def __init__(self, controller, gui, camera_callback):
        self.ctl = controller; self.gui = gui; self.get_frame = camera_callback
        self.active = False; self.previewing = False; self.buffer = []
        self.Kp = -0.05; self.Ki = -0.005; self.integral_limit = 1000
        self.v_limit = 150.0; self.curr_v = 75.0
        self.target_val = 0.0 # Stores the setpoint

    def set_volts(self, v):
        self.curr_v = max(0, min(v, self.v_limit))
        self.ctl.set_voltage('x', self.curr_v) 

    def start_preview(self):
        self.previewing = True; self.active = False; print("Previewing...")
        while self.previewing:
            if not self.gui.win.isVisible(): self.previewing = False; break
            frame = self.get_frame()
            if frame is not None:
                err = self.gui.get_balance_error(frame)
                self.buffer.append(err)
                if len(self.buffer) > 1000: self.buffer.pop(0)
                self.gui.update_display(frame, self.buffer)
            QtWidgets.QApplication.processEvents()

    def start_lock(self):
        self.active = True; self.previewing = False
        self.set_volts(75.0); time.sleep(0.25)
        
        print("Flushing buffer..."); t0 = time.time()
        while (time.time() - t0) < 0.25: self.get_frame()
        
        # --- CAPTURE CURRENT STATE AS TARGET ---
        # We take one frame to see where the fringe is right now.
        initial_frame = self.get_frame()
        if initial_frame is not None:
            self.target_val = self.gui.get_balance_error(initial_frame)
        else:
            self.target_val = 0.0
            
        print(f"--- LOCK ENGAGED | Target: {self.target_val:.2f}% ---")
        self.gui.set_target_line(self.target_val) # Update Green Line
        
        integral = 0.0
        
        while self.active:
            if not self.gui.win.isVisible(): self.active = False; break
            frame = self.get_frame()
            if frame is None: continue
            
            # Get Current Balance
            current_val = self.gui.get_balance_error(frame)
            
            # Error = Current - Target
            # (If Kp is negative, and Current > Target, we want negative correction to reduce voltage)
            error = current_val - self.target_val
            
            integral += error
            integral = max(-self.integral_limit, min(integral, self.integral_limit))
            correction = (error * self.Kp) + (integral * self.Ki)
            
            self.set_volts(self.curr_v + correction)
            
            self.buffer.append(current_val) # Plot the value, not the error, to see it track target
            if len(self.buffer) > 200: self.buffer.pop(0)
            self.gui.update_display(frame, self.buffer)

    def stop(self):
        self.active = False; self.previewing = False; print("Stopped.")


# ==========================================
# 5. MAIN EXECUTION
# ==========================================

if __name__ == "__main__":
    piezo = MDT693BController(port='COM3')
    piezo.set_voltage('x', 75)
    cam = PylonCamera(); cam.open(); cam.set_exposure(10)

    gui = DifferentialLockerWindow()
    locker = DifferentialLocker(piezo, gui, cam.take_image)
    gui.shutdown_callback = locker.stop

    ctrl_panel = QtWidgets.QWidget(); ctrl_layout = QtWidgets.QHBoxLayout()
    ctrl_panel.setLayout(ctrl_layout); gui.layout.addWidget(ctrl_panel)

    for name, func in [("RESET (75V)", lambda: locker.set_volts(75.0)),
                       ("PREVIEW", locker.start_preview),
                       ("LOCK", locker.start_lock),
                       ("STOP", locker.stop)]:
        btn = QtWidgets.QPushButton(name); btn.clicked.connect(func)
        ctrl_layout.addWidget(btn)

    tune_layout = QtWidgets.QVBoxLayout(); ctrl_layout.addLayout(tune_layout)
    
    # KP
    kp_row = QtWidgets.QHBoxLayout()
    kp_row.addWidget(QtWidgets.QLabel("Kp:"))
    spin_kp = QtWidgets.QDoubleSpinBox(); spin_kp.setRange(-5.0, 5.0)
    spin_kp.setSingleStep(0.001); spin_kp.setDecimals(4); spin_kp.setValue(locker.Kp)
    spin_kp.valueChanged.connect(lambda v: setattr(locker, 'Kp', v))
    kp_row.addWidget(spin_kp); tune_layout.addLayout(kp_row)

    # KI
    ki_row = QtWidgets.QHBoxLayout()
    ki_row.addWidget(QtWidgets.QLabel("Ki:"))
    spin_ki = QtWidgets.QDoubleSpinBox(); spin_ki.setRange(-1.0, 1.0)
    spin_ki.setSingleStep(0.0001); spin_ki.setDecimals(5); spin_ki.setValue(locker.Ki)
    spin_ki.valueChanged.connect(lambda v: setattr(locker, 'Ki', v))
    ki_row.addWidget(spin_ki); tune_layout.addLayout(ki_row)

    print("Running... Close window to stop.")
    if (sys.flags.interactive != 1): QtWidgets.QApplication.instance().exec_()