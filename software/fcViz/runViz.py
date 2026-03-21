import sys
import struct
import collections
import serial
import serial.tools.list_ports
from PyQt6 import QtWidgets, QtCore
import pyqtgraph as pg
import pyqtgraph.opengl as gl
import numpy as np

# Import custom gauges
from gauges import (
    ArtificialHorizon,
    AirspeedIndicator,
    Altimeter,
    HeadingIndicator,
    TurnCoordinator,
    VerticalSpeedIndicator
)

# --- Protocol Constants ---
SYNC_1 = 0x55
SYNC_2 = 0xAA
HEADER_SIZE = 4

# Packet Types
LOG_TYPE_ATTITUDE = 0x01
LOG_TYPE_COMMANDS = 0x02
LOG_TYPE_MAG_RAW = 0x03
LOG_TYPE_CAL_STATUS = 0x04
LOG_TYPE_SYSTEM_STATUS = 0x05

# State Map
STATE_MAP = {
    0: "BOOT",
    1: "IDLE",
    2: "FLIGHT",
    3: "FAILSAFE",
    4: "CALIBRATE"
}


# --- Calibration Logic (Ported from MATLAB) ---
class MagnetometerCalibrator:
    @staticmethod
    def fit(points):
        """
        Fits an ellipsoid to the points and returns the calibration parameters.
        Ported from ellipsoid_fit2magnetic_data.m
        """
        if len(points) < 9:
            return None, None, None

        X = np.array(points)
        x, y, z = X[:, 0], X[:, 1], X[:, 2]

        # 1. Solve for Ellipsoid Center (Hard Iron)
        D = np.array([
            x ** 2, y ** 2, z ** 2,
            2 * x * y, 2 * x * z, 2 * y * z,
            2 * x, 2 * y, 2 * z
        ]).T

        v, _, _, _ = np.linalg.lstsq(D, np.ones(len(x)), rcond=None)

        A = np.array([
            [v[0], v[3], v[4], v[6]],
            [v[3], v[1], v[5], v[7]],
            [v[4], v[5], v[2], v[8]],
            [v[6], v[7], v[8], -1]
        ])

        center_rhs = -np.array([v[6], v[7], v[8]])
        offset = np.linalg.solve(A[:3, :3], center_rhs)

        # 2. Solve for Shape (Soft Iron)
        xc = x - offset[0]
        yc = y - offset[1]
        zc = z - offset[2]

        K = np.array([
            xc ** 2, yc ** 2, zc ** 2,
            2 * xc * yc, 2 * xc * zc, 2 * yc * zc
        ]).T

        p, _, _, _ = np.linalg.lstsq(K, np.ones(len(xc)), rcond=None)

        AA = np.array([
            [p[0], p[3], p[4]],
            [p[3], p[1], p[5]],
            [p[4], p[5], p[2]]
        ])

        evals, evecs = np.linalg.eig(AA)
        radii = np.sqrt(1.0 / evals)
        Bfield = np.power(np.prod(radii), 1.0 / 3.0)

        D_sqrt = np.diag(np.sqrt(evals))
        W_inv = evecs @ D_sqrt @ evecs.T * Bfield

        return offset, W_inv, Bfield


class TelemetryParser(QtCore.QObject):
    attitude_received = QtCore.pyqtSignal(float, float, float)
    commands_received = QtCore.pyqtSignal(int, int, int, int, int, int)
    mag_received = QtCore.pyqtSignal(float, float, float)
    cal_status_received = QtCore.pyqtSignal(int, float)
    system_status_received = QtCore.pyqtSignal(int, int, int)
    connection_status = QtCore.pyqtSignal(bool)

    def __init__(self, port, baudrate=57600):
        super().__init__()
        self.port = port
        self.baudrate = baudrate
        self.running = False
        self.serial_conn = None

    def start(self):
        self.running = True
        try:
            self.serial_conn = serial.Serial(self.port, self.baudrate, timeout=0.1, dsrdtr=False, rtscts=False)
            self.connection_status.emit(True)
            print(f"Connected to {self.port}")
            self._read_loop()
        except serial.SerialException as e:
            print(f"Serial Error: {e}")
            self.connection_status.emit(False)

    def stop(self):
        self.running = False
        if self.serial_conn and self.serial_conn.is_open:
            self.serial_conn.close()
            print("Serial closed")

    def _read_loop(self):
        buffer = bytearray()
        while self.running and self.serial_conn.is_open:
            try:
                if self.serial_conn.in_waiting > 0:
                    buffer.extend(self.serial_conn.read(self.serial_conn.in_waiting))

                while len(buffer) >= HEADER_SIZE:
                    if buffer[0] != SYNC_1 or buffer[1] != SYNC_2:
                        del buffer[0]
                        continue

                    msg_type, payload_len = buffer[2], buffer[3]

                    # Size check: Att(12), Cmd(12), Mag(12), Cal(5), Sys(4)
                    if payload_len not in [8, 12, 5, 4]:
                        del buffer[:2]
                        continue

                    packet_size = HEADER_SIZE + payload_len + 1
                    if len(buffer) < packet_size: break

                    payload = buffer[HEADER_SIZE: HEADER_SIZE + payload_len]
                    if self._calc_checksum(payload) == buffer[HEADER_SIZE + payload_len]:
                        self._parse_payload(msg_type, payload)
                        del buffer[:packet_size]
                    else:
                        print(f"Checksum Fail: {msg_type}")
                        del buffer[:2]
            except Exception as e:
                print(f"Parser Loop Error: {e}")
                buffer.clear()

    def _calc_checksum(self, payload):
        chk = 0
        for b in payload: chk ^= b
        return chk

    def _parse_payload(self, msg_type, data):
        try:
            if msg_type == LOG_TYPE_ATTITUDE:
                self.attitude_received.emit(*struct.unpack('<fff', data))
            elif msg_type == LOG_TYPE_COMMANDS:
                self.commands_received.emit(*struct.unpack('<HHHHHH', data))
            elif msg_type == LOG_TYPE_MAG_RAW:
                self.mag_received.emit(*struct.unpack('<fff', data))
            elif msg_type == LOG_TYPE_CAL_STATUS:
                self.cal_status_received.emit(*struct.unpack('<Bf', data))
            elif msg_type == LOG_TYPE_SYSTEM_STATUS:
                self.system_status_received.emit(*struct.unpack('<BBH', data))
        except Exception:
            pass


class MainWindow(QtWidgets.QMainWindow):
    def __init__(self):
        super().__init__()
        self.setWindowTitle("Flight Telemetry & Calibration")
        self.resize(1200, 800)

        self.tabs = QtWidgets.QTabWidget()
        self.setCentralWidget(self.tabs)

        self.dash_tab = QtWidgets.QWidget()
        self.setup_dashboard(self.dash_tab)
        self.tabs.addTab(self.dash_tab, "Dashboard")

        self.cal_tab = QtWidgets.QWidget()
        self.setup_calibration(self.cal_tab)
        self.tabs.addTab(self.cal_tab, "Mag Calibration 3D")

        self.thread = QtCore.QThread()
        self.worker = None

        self.mag_points_raw = []

        # Graph Buffers
        self.att_data = {k: collections.deque([0] * 200, maxlen=200) for k in ['r', 'p', 'y']}
        self.cmd_data = {k: collections.deque([1500] * 200, maxlen=200) for k in ['a', 'e', 'r', 't', 'x1', 'x2']}

    def setup_dashboard(self, widget):
        layout = QtWidgets.QVBoxLayout(widget)

        # Bar
        bar = QtWidgets.QHBoxLayout()
        self.combo = QtWidgets.QComboBox()
        self.refresh_ports()
        self.btn_conn = QtWidgets.QPushButton("Connect")
        self.btn_conn.clicked.connect(self.toggle_conn)

        self.lbl_state = QtWidgets.QLabel("STATE: DISCONNECTED")
        self.style_label(self.lbl_state, "gray")

        # Add CPU Load Label
        self.lbl_cpu = QtWidgets.QLabel("CPU: ---%")
        self.lbl_cpu.setStyleSheet(
            "font-weight: bold; color: gray; border: 2px solid gray; padding: 5px; border-radius: 4px;")

        bar.addWidget(QtWidgets.QLabel("Port:"))
        bar.addWidget(self.combo)
        bar.addWidget(self.btn_conn)
        bar.addSpacing(20)
        bar.addWidget(self.lbl_state)
        bar.addSpacing(10)
        bar.addWidget(self.lbl_cpu)  # Add to the top bar
        bar.addStretch()
        layout.addLayout(bar)

        # Content
        content = QtWidgets.QHBoxLayout()
        layout.addLayout(content)

        # Inst
        inst_col = QtWidgets.QWidget()
        inst_col.setFixedWidth(350)
        grid = QtWidgets.QGridLayout(inst_col)
        self.asi = AirspeedIndicator()
        self.ai = ArtificialHorizon()
        self.alt = Altimeter()
        self.hi = HeadingIndicator()
        self.tc = TurnCoordinator()
        self.vsi = VerticalSpeedIndicator()
        grid.addWidget(self.asi, 0, 0)
        grid.addWidget(self.ai, 0, 1)
        grid.addWidget(self.alt, 1, 0)
        grid.addWidget(self.hi, 1, 1)
        grid.addWidget(self.tc, 2, 0)
        grid.addWidget(self.vsi, 2, 1)
        content.addWidget(inst_col)

        # Graphs
        self.glw = pg.GraphicsLayoutWidget()
        content.addWidget(self.glw, 1)

        # Attitude Plot
        p1 = self.glw.addPlot(title="Attitude")
        p1.showGrid(x=True, y=True)
        p1.setYRange(-45, 45)
        p1.addLegend()
        self.c_roll = p1.plot(pen='r', name="Roll")
        self.c_pitch = p1.plot(pen='g', name="Pitch")
        self.c_yaw = p1.plot(pen='b', name="Yaw")

        self.glw.nextRow()

        # Commands Plot
        p2 = self.glw.addPlot(title="RC Inputs")
        p2.showGrid(x=True, y=True)
        p2.setYRange(900, 2100)
        p2.addLegend()
        self.c_ail = p2.plot(pen='r', name="Ail")
        self.c_ele = p2.plot(pen='g', name="Ele")
        self.c_rud = p2.plot(pen='b', name="Rud")
        self.c_thr = p2.plot(pen='y', name="Thr")
        self.c_aux1 = p2.plot(pen='c', name="Aux1")
        self.c_aux2 = p2.plot(pen='m', name="Aux2")

    def setup_calibration(self, widget):
        # Main Layout: 3 Columns (Left Panel, Visualizer, Right Panel)
        main_layout = QtWidgets.QHBoxLayout(widget)
        main_layout.setContentsMargins(10, 10, 10, 10)
        main_layout.setSpacing(10)

        # --- LEFT PANEL: Actions & Status ---
        left_panel = QtWidgets.QFrame()
        left_panel.setFrameShape(QtWidgets.QFrame.Shape.StyledPanel)
        left_panel.setFixedWidth(200)
        left_layout = QtWidgets.QVBoxLayout(left_panel)

        # Actions Group
        grp_actions = QtWidgets.QGroupBox("Actions")
        act_layout = QtWidgets.QVBoxLayout()

        self.btn_clear = QtWidgets.QPushButton("Clear Data")
        self.btn_clear.clicked.connect(self.clear_cal_data)
        self.btn_clear.setMinimumHeight(30)

        self.btn_cal = QtWidgets.QPushButton("Calculate / Send")
        self.btn_cal.clicked.connect(self.run_calibration)
        self.btn_cal.setMinimumHeight(40)
        self.btn_cal.setStyleSheet("background-color: #ddd; font-weight: bold;")

        act_layout.addWidget(self.btn_clear)
        act_layout.addWidget(self.btn_cal)
        grp_actions.setLayout(act_layout)

        # Status Group
        grp_status = QtWidgets.QGroupBox("Status")
        stat_layout = QtWidgets.QVBoxLayout()

        # Increased size to 100x100 to fit text, adjusted radius to 50px
        self.lbl_cal_state = QtWidgets.QLabel("DISCONNECTED")
        self.lbl_cal_state.setAlignment(QtCore.Qt.AlignmentFlag.AlignCenter)
        self.lbl_cal_state.setWordWrap(True)
        self.lbl_cal_state.setStyleSheet(
            "border: 5px solid gray; border-radius: 50px; "
            "min-width: 100px; min-height: 100px; "
            "max-width: 100px; max-height: 100px; "
            "font-weight: bold; color: gray; font-size: 10px;"
        )

        # Container to center the circle horizontally
        stat_container = QtWidgets.QWidget()
        stat_cont_layout = QtWidgets.QHBoxLayout(stat_container)
        stat_cont_layout.addStretch()
        stat_cont_layout.addWidget(self.lbl_cal_state)
        stat_cont_layout.addStretch()

        stat_layout.addWidget(stat_container)
        grp_status.setLayout(stat_layout)

        left_layout.addWidget(grp_actions)
        left_layout.addSpacing(10)
        left_layout.addWidget(grp_status)
        left_layout.addStretch()

        # --- CENTER PANEL: Visualization ---
        center_panel = QtWidgets.QWidget()
        center_layout = QtWidgets.QVBoxLayout(center_panel)
        center_layout.setContentsMargins(0, 0, 0, 0)

        lbl_title = QtWidgets.QLabel("Ideal calibration is a perfectly centered sphere")
        lbl_title.setAlignment(QtCore.Qt.AlignmentFlag.AlignCenter)
        lbl_title.setStyleSheet("font-size: 10pt; color: #777; margin-bottom: 5px;")

        self.gl_view = gl.GLViewWidget()
        self.gl_view.setCameraPosition(distance=10, elevation=30, azimuth=45)
        self.gl_view.setSizePolicy(QtWidgets.QSizePolicy.Policy.Expanding, QtWidgets.QSizePolicy.Policy.Expanding)

        g = gl.GLGridItem()
        g.scale(2, 2, 1)
        self.gl_view.addItem(g)

        # Raw Data (Red)
        self.scatter_raw = gl.GLScatterPlotItem(pos=np.array([[0, 0, 0]]), color=(1, 0, 0, 0.6), size=6, pxMode=True)
        self.gl_view.addItem(self.scatter_raw)

        # Calibrated Data (Green)
        self.scatter_cal = gl.GLScatterPlotItem(pos=np.array([[0, 0, 0]]), color=(0, 1, 0, 0.8), size=6, pxMode=True)
        self.gl_view.addItem(self.scatter_cal)

        center_layout.addWidget(lbl_title)
        center_layout.addWidget(self.gl_view, stretch=1)

        # --- RIGHT PANEL: Results ---
        right_panel = QtWidgets.QFrame()
        right_panel.setFrameShape(QtWidgets.QFrame.Shape.StyledPanel)
        right_panel.setFixedWidth(250)
        right_layout = QtWidgets.QVBoxLayout(right_panel)

        lbl_res_header = QtWidgets.QLabel("Calibration Results")
        lbl_res_header.setStyleSheet("font-weight: bold; font-size: 14px;")
        right_layout.addWidget(lbl_res_header)
        right_layout.addSpacing(10)

        # 1. Magnetic Offset
        right_layout.addWidget(QtWidgets.QLabel("Magnetic Offset"))
        self.res_offsets = []
        for _ in range(3):
            l = QtWidgets.QLabel("0.000")
            l.setStyleSheet("font-family: Monospace; margin-left: 10px;")
            self.res_offsets.append(l)
            right_layout.addWidget(l)

        right_layout.addSpacing(15)

        # 2. Magnetic Mapping (Matrix)
        right_layout.addWidget(QtWidgets.QLabel("Magnetic Mapping"))
        map_grid = QtWidgets.QGridLayout()
        self.res_matrix = []
        for r in range(3):
            row_lbls = []
            for c in range(3):
                l = QtWidgets.QLabel("+0.000")
                l.setStyleSheet("font-family: Monospace;")
                map_grid.addWidget(l, r, c)
                row_lbls.append(l)
            self.res_matrix.append(row_lbls)
        right_layout.addLayout(map_grid)

        right_layout.addSpacing(15)

        # 3. Magnetic Field
        right_layout.addWidget(QtWidgets.QLabel("Magnetic Field"))
        self.res_field = QtWidgets.QLabel("0.00")
        self.res_field.setStyleSheet("font-family: Monospace; font-weight: bold; margin-left: 10px;")
        right_layout.addWidget(self.res_field)

        right_layout.addStretch()

        # Add panels to main layout
        main_layout.addWidget(left_panel)
        main_layout.addWidget(center_panel, stretch=1)
        main_layout.addWidget(right_panel)

    def refresh_ports(self):
        self.combo.clear()
        for p in serial.tools.list_ports.comports(): self.combo.addItem(p.device)

    def toggle_conn(self):
        if not self.worker:
            port = self.combo.currentText()
            if not port: return
            self.worker = TelemetryParser(port)
            self.worker.moveToThread(self.thread)
            self.thread.started.connect(self.worker.start)
            self.worker.attitude_received.connect(self.on_att)
            self.worker.commands_received.connect(self.on_cmd)
            self.worker.mag_received.connect(self.on_mag)
            self.worker.system_status_received.connect(self.on_status)
            self.thread.start()
            self.btn_conn.setText("Disconnect")
        else:
            self.worker.stop()
            self.thread.quit()
            self.thread.wait()
            self.worker = None
            self.btn_conn.setText("Connect")
            self.style_label(self.lbl_state, "gray", "DISCONNECTED")
            if hasattr(self, 'lbl_cal_state'):
                self.lbl_cal_state.setText("DISCONN")
                self.lbl_cal_state.setStyleSheet(
                    "border: 5px solid gray; border-radius: 35px; min-width: 70px; min-height: 70px; max-width: 70px; max-height: 70px; font-weight: bold; color: gray;")

    def on_att(self, r, p, y):
        self.att_data['r'].append(r)
        self.att_data['p'].append(p)
        self.att_data['y'].append(y)
        self.c_roll.setData(self.att_data['r'])
        self.c_pitch.setData(self.att_data['p'])
        self.c_yaw.setData(self.att_data['y'])
        self.ai.setAttitude(r, p)
        self.hi.setValue(y)

    def on_cmd(self, a, e, r, t, x1, x2):
        self.cmd_data['a'].append(a)
        self.cmd_data['e'].append(e)
        self.cmd_data['r'].append(r)
        self.cmd_data['t'].append(t)
        self.cmd_data['x1'].append(x1)
        self.cmd_data['x2'].append(x2)

        self.c_ail.setData(self.cmd_data['a'])
        self.c_ele.setData(self.cmd_data['e'])
        self.c_rud.setData(self.cmd_data['r'])
        self.c_thr.setData(self.cmd_data['t'])
        self.c_aux1.setData(self.cmd_data['x1'])
        self.c_aux2.setData(self.cmd_data['x2'])

    def on_mag(self, x, y, z):
        self.mag_points_raw.append([x, y, z])
        if len(self.mag_points_raw) % 5 == 0:
            self.scatter_raw.setData(pos=np.array(self.mag_points_raw))

    def on_status(self, state, reserved, cpu):
        name = STATE_MAP.get(state, f"UNK({state})")
        color_map = {0: "orange", 1: "#00AA00", 2: "red", 3: "magenta", 4: "#00AAFF"}
        color = color_map.get(state, "gray")

        self.style_label(self.lbl_state, color, name)

        # Update CPU Label with dynamic coloring
        cpu_color = "red" if cpu < 60 else "orange" if cpu < 85 else "lime"
        self.lbl_cpu.setText(f"CPU IDLE: {cpu}%")
        self.lbl_cpu.setStyleSheet(
            f"font-weight: bold; color: {cpu_color}; border: 2px solid {cpu_color}; padding: 5px; border-radius: 4px;")

        # Update the visual circle on Cal tab
        if hasattr(self, 'lbl_cal_state'):
            self.lbl_cal_state.setText(name)
            self.lbl_cal_state.setStyleSheet(
                f"border: 5px solid {color}; border-radius: 50px; "
                f"min-width: 100px; min-height: 100px; "
                f"max-width: 100px; max-height: 100px; "
                f"font-weight: bold; color: {color}; font-size: 10px;"
            )

    def style_label(self, lbl, color, text=None):
        if text: lbl.setText(f"STATE: {text}")
        lbl.setStyleSheet(
            f"font-weight: bold; color: {color}; border: 2px solid {color}; padding: 5px; border-radius: 4px;")

    def clear_cal_data(self):
        self.mag_points_raw = []
        self.scatter_raw.setData(pos=np.array([[0, 0, 0]]))
        self.scatter_cal.setData(pos=np.array([[0, 0, 0]]))
        # Reset labels
        for l in self.res_offsets: l.setText("0.000")
        for row in self.res_matrix:
            for l in row: l.setText("+0.000")
        self.res_field.setText("0.00")

    def run_calibration(self):
        if len(self.mag_points_raw) < 50:
            QtWidgets.QMessageBox.warning(self, "Error", "Not enough data points. Rotate drone more!")
            return

        print("Running Ellipsoid Fit...")
        offset, soft_iron, field_strength = MagnetometerCalibrator.fit(self.mag_points_raw)

        if offset is None:
            print("Calibration Failed (singular matrix?)")
            return

        # Apply correction to visualize
        raw = np.array(self.mag_points_raw)
        corrected = (raw - offset) @ soft_iron.T
        self.scatter_cal.setData(pos=corrected)

        # --- UPDATE UI LABELS ---
        # 1. Update Offsets
        for i in range(3):
            self.res_offsets[i].setText(f"{offset[i]:.3f}")

        # 2. Update Matrix
        for r in range(3):
            for c in range(3):
                val = soft_iron[r, c]
                self.res_matrix[r][c].setText(f"{val:+.3f}")

        # 3. Update Field
        self.res_field.setText(f"{field_strength:.2f}")

        # Console Output (Keep for C++ copying)
        print("\n" + "=" * 40)
        print(" CALIBRATION RESULTS (Copy to C++)")
        print("=" * 40)
        print(f"// Field Strength: {field_strength:.2f}")
        print(f"static constexpr float kFactoryHardIron[3] = {{ {offset[0]:.4f}f, {offset[1]:.4f}f, {offset[2]:.4f}f }};")
        print("static constexpr float kFactorySoftIron[9] = {")
        print(f"    {soft_iron[0, 0]:.4f}f, {soft_iron[0, 1]:.4f}f, {soft_iron[0, 2]:.4f}f,")
        print(f"    {soft_iron[1, 0]:.4f}f, {soft_iron[1, 1]:.4f}f, {soft_iron[1, 2]:.4f}f,")
        print(f"    {soft_iron[2, 0]:.4f}f, {soft_iron[2, 1]:.4f}f, {soft_iron[2, 2]:.4f}f")
        print("};")
        print("=" * 40 + "\n")

        QtWidgets.QMessageBox.information(self, "Success", "Calibration Complete!")


if __name__ == "__main__":
    app = QtWidgets.QApplication(sys.argv)
    win = MainWindow()
    win.show()
    sys.exit(app.exec())