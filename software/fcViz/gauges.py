import math
from PyQt6 import QtWidgets, QtCore, QtGui

class InstrumentWidget(QtWidgets.QWidget):
    """Base class for all circular flight instruments."""
    def __init__(self, parent=None):
        super().__init__(parent)
        self.setMinimumSize(100, 100)
        self.value = 0
    
    def paintEvent(self, event):
        if self.width() < 10 or self.height() < 10:
            return

        painter = QtGui.QPainter()
        if not painter.begin(self):
            return

        try:
            painter.setRenderHint(QtGui.QPainter.RenderHint.Antialiasing)
            painter.setRenderHint(QtGui.QPainter.RenderHint.TextAntialiasing)

            w = self.width()
            h = self.height()
            self.cx = w / 2
            self.cy = h / 2
            self.size = min(w, h) - 20
            self.r = self.size / 2

            # Draw Bezel
            self.draw_bezel(painter)

            # Draw Instrument Face (implemented by subclasses)
            self.draw_face(painter)

        except Exception as e:
            print(f"Paint Error in {self.__class__.__name__}: {e}")
        finally:
            painter.end()

    def draw_bezel(self, painter):
        painter.setPen(QtCore.Qt.PenStyle.NoPen)
        painter.setBrush(QtGui.QColor(40, 40, 40))
        painter.drawEllipse(QtCore.QPointF(self.cx, self.cy), self.r + 5, self.r + 5)
        
    def draw_face(self, painter):
        pass


class ArtificialHorizon(InstrumentWidget):
    def __init__(self, parent=None):
        super().__init__(parent)
        self.roll = 0.0
        self.pitch = 0.0

    def setAttitude(self, roll, pitch):
        self.roll = roll
        self.pitch = pitch
        self.update()

    def draw_face(self, painter):
        # Clip to circle
        path = QtGui.QPainterPath()
        path.addEllipse(QtCore.QPointF(self.cx, self.cy), self.r, self.r)
        painter.setClipPath(path)
        
        pitch_scale = self.r / 25.0 

        # --- Sky/Ground & Pitch Ladder ---
        painter.save()
        painter.translate(self.cx, self.cy)
        painter.rotate(-self.roll)
        painter.translate(0, self.pitch * pitch_scale)

        diag = self.r * 4 
        
        # Sky
        sky_grad = QtGui.QLinearGradient(0, -diag, 0, 0)
        sky_grad.setColorAt(0, QtGui.QColor(0, 100, 200))
        sky_grad.setColorAt(1, QtGui.QColor(135, 206, 235))
        painter.setBrush(sky_grad)
        painter.setPen(QtCore.Qt.PenStyle.NoPen)
        painter.drawRect(QtCore.QRectF(-diag, -diag, 2*diag, diag))

        # Ground
        gnd_grad = QtGui.QLinearGradient(0, 0, 0, diag)
        gnd_grad.setColorAt(0, QtGui.QColor(160, 82, 45))
        gnd_grad.setColorAt(1, QtGui.QColor(80, 40, 20))
        painter.setBrush(gnd_grad)
        painter.drawRect(QtCore.QRectF(-diag, 0, 2*diag, diag))
        
        # Horizon Line
        painter.setPen(QtGui.QPen(QtCore.Qt.GlobalColor.white, 2))
        painter.drawLine(QtCore.QLineF(-diag, 0, diag, 0))

        # Pitch Ladder
        font = painter.font()
        font.setPixelSize(int(self.r * 0.1))
        font.setBold(True)
        painter.setFont(font)
        
        for i in range(-90, 91, 10):
            if i == 0: continue
            y_pos = -i * pitch_scale
            width_factor = self.r * 0.4 if i % 20 == 0 else self.r * 0.15
            
            painter.setPen(QtGui.QPen(QtCore.Qt.GlobalColor.white, 2))
            painter.drawLine(QtCore.QLineF(-width_factor, y_pos, width_factor, y_pos))
            
            if i % 10 == 0:
                text = str(abs(i))
                fm = QtGui.QFontMetrics(font)
                tw = fm.horizontalAdvance(text)
                th = fm.height()
                painter.drawText(QtCore.QRectF(-width_factor - tw - 5, y_pos - th/2, tw, th), QtCore.Qt.AlignmentFlag.AlignCenter, text)
                painter.drawText(QtCore.QRectF(width_factor + 5, y_pos - th/2, tw, th), QtCore.Qt.AlignmentFlag.AlignCenter, text)

        painter.restore()

        # --- Bank Scale ---
        painter.save()
        painter.translate(self.cx, self.cy)
        painter.rotate(-self.roll)
        
        painter.setPen(QtGui.QPen(QtCore.Qt.GlobalColor.white, 2))
        scale_r = self.r * 0.9
        tick_angles = [-60, -45, -30, -20, -10, 0, 10, 20, 30, 45, 60]
        
        for angle in tick_angles:
            painter.save()
            painter.rotate(angle) 
            painter.drawLine(QtCore.QPointF(0, -scale_r), QtCore.QPointF(0, -scale_r + 10))
            painter.restore()
        painter.restore() 

        # --- Aircraft Reference ---
        painter.translate(self.cx, self.cy)
        painter.setPen(QtGui.QPen(QtCore.Qt.GlobalColor.yellow, 3))
        # Use QLineF explicitly
        painter.drawLine(QtCore.QLineF(-40, 0, -10, 0))
        painter.drawLine(QtCore.QLineF(10, 0, 40, 0))
        painter.drawLine(QtCore.QLineF(0, -10, 0, 10))


class AirspeedIndicator(InstrumentWidget):
    def setValue(self, knots):
        self.value = knots
        self.update()

    def draw_face(self, painter):
        painter.setBrush(QtCore.Qt.GlobalColor.black)
        painter.drawEllipse(QtCore.QPointF(self.cx, self.cy), self.r, self.r)
        
        painter.translate(self.cx, self.cy)
        
        # Scale: 0 to 200 knots, 300 degrees range
        def draw_arc(start_v, end_v, color, width=5):
            # Qt angles: 0 is 3 oclock, + is CCW.
            # We map:
            # 0 knots -> 225 deg (7:30 oclock)
            # 200 knots -> -45 deg (4:30 oclock)
            
            def v_to_qt(v):
                return 225 - (v / 200) * 270

            a1 = v_to_qt(start_v)
            a2 = v_to_qt(end_v)
            span = a2 - a1
            
            # Qt drawArc uses 1/16th of a degree
            rect = QtCore.QRectF(-self.r*0.9, -self.r*0.9, self.r*1.8, self.r*1.8)
            painter.setPen(QtGui.QPen(color, width))
            painter.drawArc(rect, int(a1 * 16), int(span * 16))

        draw_arc(40, 85, QtCore.Qt.GlobalColor.white, 8)
        draw_arc(48, 129, QtCore.Qt.GlobalColor.green, 8)
        draw_arc(129, 163, QtCore.Qt.GlobalColor.yellow, 8)
        draw_arc(163, 165, QtCore.Qt.GlobalColor.red, 8)

        # Ticks
        painter.setPen(QtGui.QPen(QtCore.Qt.GlobalColor.white, 2))
        font = painter.font()
        font.setPixelSize(int(self.r * 0.15))
        painter.setFont(font)
        
        for v in range(0, 201, 10):
            ang = 225 - (v / 200) * 270
            rad = math.radians(ang)
            
            # Use QPointF explicitly
            p1 = QtCore.QPointF(math.cos(rad) * self.r * 0.9, -math.sin(rad) * self.r * 0.9)
            p2 = QtCore.QPointF(math.cos(rad) * self.r * 0.8, -math.sin(rad) * self.r * 0.8)
            painter.drawLine(p1, p2)
            
            if v % 20 == 0:
                p_text = QtCore.QPointF(math.cos(rad) * self.r * 0.65, -math.sin(rad) * self.r * 0.65)
                rect = QtCore.QRectF(p_text.x()-20, p_text.y()-10, 40, 20)
                painter.drawText(rect, QtCore.Qt.AlignmentFlag.AlignCenter, str(v))

        # Needle
        painter.save()
        needle_ang = 225 - (self.value / 200) * 270
        painter.rotate(-needle_ang)
        painter.setBrush(QtCore.Qt.GlobalColor.white)
        painter.setPen(QtCore.Qt.PenStyle.NoPen)
        painter.drawPolygon([
            QtCore.QPointF(0, -2),
            QtCore.QPointF(self.r * 0.85, 0),
            QtCore.QPointF(0, 2)
        ])
        painter.restore()
        
        painter.setBrush(QtCore.Qt.GlobalColor.gray)
        painter.drawEllipse(QtCore.QPointF(0,0), 5, 5)


class Altimeter(InstrumentWidget):
    def setValue(self, feet):
        self.value = feet
        self.update()

    def draw_face(self, painter):
        painter.setBrush(QtCore.Qt.GlobalColor.black)
        painter.drawEllipse(QtCore.QPointF(self.cx, self.cy), self.r, self.r)
        
        painter.translate(self.cx, self.cy)
        
        painter.setPen(QtGui.QPen(QtCore.Qt.GlobalColor.white, 2))
        font = painter.font()
        font.setPixelSize(int(self.r * 0.15))
        painter.setFont(font)
        
        for i in range(0, 10):
            ang = 90 - (i / 10) * 360
            rad = math.radians(ang)
            
            p1 = QtCore.QPointF(math.cos(rad) * self.r * 0.9, -math.sin(rad) * self.r * 0.9)
            p2 = QtCore.QPointF(math.cos(rad) * self.r * 0.8, -math.sin(rad) * self.r * 0.8)
            painter.drawLine(p1, p2)
            
            p_text = QtCore.QPointF(math.cos(rad) * self.r * 0.65, -math.sin(rad) * self.r * 0.65)
            rect = QtCore.QRectF(p_text.x()-15, p_text.y()-10, 30, 20)
            painter.drawText(rect, QtCore.Qt.AlignmentFlag.AlignCenter, str(i))

        # Digital
        rect_dig = QtCore.QRectF(-30, 10, 60, 20)
        painter.setPen(QtCore.Qt.GlobalColor.white)
        painter.drawText(rect_dig, QtCore.Qt.AlignmentFlag.AlignCenter, f"{int(self.value)}")

        # Needles
        # 10k
        val_10k = (self.value / 10000) % 10
        ang_10k = 90 - (val_10k / 10) * 360
        painter.save()
        painter.rotate(-ang_10k)
        painter.setPen(QtGui.QPen(QtCore.Qt.GlobalColor.white, 2))
        painter.drawLine(QtCore.QPointF(0,0), QtCore.QPointF(self.r * 0.5, 0)) # Fixed length float
        painter.drawText(QtCore.QRectF(self.r * 0.55, -10, 30, 20), QtCore.Qt.AlignmentFlag.AlignLeft, "10k")
        painter.restore()

        # 1k
        val_1k = (self.value / 1000) % 10
        ang_1k = 90 - (val_1k / 10) * 360
        painter.save()
        painter.rotate(-ang_1k)
        painter.setBrush(QtCore.Qt.GlobalColor.white)
        painter.setPen(QtCore.Qt.PenStyle.NoPen)
        painter.drawPolygon([QtCore.QPointF(0,-3), QtCore.QPointF(self.r * 0.5, 0), QtCore.QPointF(0,3)])
        painter.restore()

        # 100
        val_100 = (self.value / 100) % 10
        ang_100 = 90 - (val_100 / 10) * 360
        painter.save()
        painter.rotate(-ang_100)
        painter.setBrush(QtCore.Qt.GlobalColor.white)
        painter.setPen(QtCore.Qt.PenStyle.NoPen)
        painter.drawPolygon([QtCore.QPointF(0,-2), QtCore.QPointF(self.r * 0.85, 0), QtCore.QPointF(0,2)])
        painter.restore()


class HeadingIndicator(InstrumentWidget):
    def setValue(self, heading):
        self.value = heading # Degrees
        self.update()

    def draw_face(self, painter):
        path = QtGui.QPainterPath()
        path.addEllipse(QtCore.QPointF(self.cx, self.cy), self.r, self.r)
        painter.setClipPath(path)

        painter.setBrush(QtCore.Qt.GlobalColor.black)
        painter.drawRect(QtCore.QRectF(0,0, self.cx*2, self.cy*2)) 

        # Rotating Compass Card
        painter.translate(self.cx, self.cy)
        painter.rotate(-self.value) 

        painter.setPen(QtGui.QPen(QtCore.Qt.GlobalColor.white, 2))
        font = painter.font()
        font.setPixelSize(int(self.r * 0.15))
        painter.setFont(font)

        labels = {0: 'N', 90: 'E', 180: 'S', 270: 'W'}
        for i in range(0, 360, 10):
            ang = i - 90 
            rad = math.radians(ang)
            
            p1 = QtCore.QPointF(math.cos(rad) * self.r * 0.9, math.sin(rad) * self.r * 0.9)
            p2 = QtCore.QPointF(math.cos(rad) * self.r * 0.8, math.sin(rad) * self.r * 0.8)
            painter.drawLine(p1, p2)
            
            if i % 90 == 0:
                lbl = labels[i]
                p_t = QtCore.QPointF(math.cos(rad) * self.r * 0.65, math.sin(rad) * self.r * 0.65)
                rect = QtCore.QRectF(p_t.x()-15, p_t.y()-15, 30, 30)
                painter.drawText(rect, QtCore.Qt.AlignmentFlag.AlignCenter, lbl)
            elif i % 30 == 0:
                lbl = str(int(i/10))
                p_t = QtCore.QPointF(math.cos(rad) * self.r * 0.65, math.sin(rad) * self.r * 0.65)
                rect = QtCore.QRectF(p_t.x()-15, p_t.y()-15, 30, 30)
                painter.drawText(rect, QtCore.Qt.AlignmentFlag.AlignCenter, lbl)

        # Fixed Aircraft Reference 
        painter.resetTransform()
        painter.translate(self.cx, self.cy)
        
        painter.setPen(QtGui.QPen(QtCore.Qt.GlobalColor.yellow, 3))
        # Top tick
        painter.drawLine(QtCore.QLineF(0, -self.r, 0, -self.r + 20))
        
        # Airplane
        painter.setPen(QtCore.Qt.PenStyle.NoPen)
        painter.setBrush(QtCore.Qt.GlobalColor.white) 
        painter.drawPolygon([
            QtCore.QPointF(0, -10),
            QtCore.QPointF(-10, 10),
            QtCore.QPointF(0, 5),
            QtCore.QPointF(10, 10)
        ])


class TurnCoordinator(InstrumentWidget):
    def __init__(self, parent=None):
        super().__init__(parent)
        self.rate = 0.0
        self.slip = 0.0

    def setValue(self, rate, slip):
        self.rate = rate 
        self.slip = slip 
        self.update()

    def draw_face(self, painter):
        painter.setBrush(QtCore.Qt.GlobalColor.black)
        painter.drawEllipse(QtCore.QPointF(self.cx, self.cy), self.r, self.r)
        
        painter.translate(self.cx, self.cy)
        
        tilt = self.rate * 20 
        
        # Wings
        painter.save()
        painter.rotate(tilt)
        painter.setPen(QtGui.QPen(QtCore.Qt.GlobalColor.white, 3))
        # Use QLineF
        painter.drawLine(QtCore.QLineF(-40, 0, 40, 0))
        painter.drawLine(QtCore.QLineF(0, -10, 0, 10))
        painter.restore()
        
        # Marks
        painter.setPen(QtGui.QPen(QtCore.Qt.GlobalColor.white, 2))
        painter.drawLine(QtCore.QLineF(-45, 15, -45, 25))
        painter.drawLine(QtCore.QLineF(45, 15, 45, 25))

        # Ball
        rect_tube = QtCore.QRectF(-40, self.r * 0.5, 80, 20)
        painter.setPen(QtGui.QPen(QtCore.Qt.GlobalColor.white, 2))
        # Correct Enum Usage
        painter.setBrush(QtCore.Qt.BrushStyle.NoBrush)
        painter.drawRoundedRect(rect_tube, 10, 10)
        
        ball_x = self.slip * 35 
        painter.setBrush(QtCore.Qt.GlobalColor.white)
        painter.setPen(QtCore.Qt.PenStyle.NoPen)
        painter.drawEllipse(QtCore.QPointF(ball_x, self.r * 0.5 + 10), 8, 8)


class VerticalSpeedIndicator(InstrumentWidget):
    def setValue(self, fpm):
        self.value = fpm
        self.update()

    def draw_face(self, painter):
        painter.setBrush(QtCore.Qt.GlobalColor.black)
        painter.drawEllipse(QtCore.QPointF(self.cx, self.cy), self.r, self.r)
        
        painter.translate(self.cx, self.cy)
        
        def val_to_angle(v):
            clamped = max(-2000, min(2000, v))
            pct = clamped / 2000.0 
            return 180 - pct * 170

        # Draw Scale
        painter.setPen(QtGui.QPen(QtCore.Qt.GlobalColor.white, 2))
        font = painter.font()
        font.setPixelSize(int(self.r * 0.15))
        painter.setFont(font)
        
        for v in range(-20, 21, 5): 
            fpm_val = v * 100
            ang = val_to_angle(fpm_val)
            rad = math.radians(ang)
            
            tick_len = 0.15 if v % 10 == 0 else 0.1
            p1 = QtCore.QPointF(math.cos(rad) * self.r * 0.9, -math.sin(rad) * self.r * 0.9)
            p2 = QtCore.QPointF(math.cos(rad) * self.r * (0.9-tick_len), -math.sin(rad) * self.r * (0.9-tick_len))
            painter.drawLine(p1, p2)
            
            if v != 0 and v % 5 == 0:
                lbl = str(abs(v))
                p_t = QtCore.QPointF(math.cos(rad) * self.r * 0.65, -math.sin(rad) * self.r * 0.65)
                rect = QtCore.QRectF(p_t.x()-15, p_t.y()-10, 30, 20)
                painter.drawText(rect, QtCore.Qt.AlignmentFlag.AlignCenter, lbl)
                
        painter.drawText(QtCore.QRectF(-30, -30, 20, 20), QtCore.Qt.AlignmentFlag.AlignCenter, "UP")
        painter.drawText(QtCore.QRectF(-30, 10, 20, 20), QtCore.Qt.AlignmentFlag.AlignCenter, "DN")

        # Needle
        val = self.value
        ang = val_to_angle(val)
        painter.save()
        painter.rotate(-ang) 
        painter.setBrush(QtCore.Qt.GlobalColor.white)
        painter.setPen(QtCore.Qt.PenStyle.NoPen)
        painter.drawPolygon([
            QtCore.QPointF(0, -2),
            QtCore.QPointF(self.r * 0.85, 0),
            QtCore.QPointF(0, 2)
        ])
        painter.restore()
        
        painter.setBrush(QtCore.Qt.GlobalColor.gray)
        painter.drawEllipse(QtCore.QPointF(0,0), 5, 5)

