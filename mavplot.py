import time
import threading
from argparse import ArgumentParser

import numpy as np
import pyqtgraph as pg
from pymavlink import mavutil
from pyqtgraph.Qt import QtGui, QtCore
from PyQt5.QtCore import pyqtSignal, pyqtSlot

app = QtGui.QApplication([])

parser = ArgumentParser()
parser.add_argument('device')
parser.add_argument('--baud', type=int, default=115200)
args = parser.parse_args()

master = mavutil.mavlink_connection(args.device, baud=args.baud, robust_parsing=True)
master.wait_heartbeat()

mavlink_severity_names = 'EMERGENCY ALERT CRITICAL ERROR WARNING NOTICE INFO DEBUG'.split()

ts, xs, ys, zs, vxs, vys, vzs = [], [], [], [], [], [], []
def readloop(callback, threadkill, limit=1000):
    att = lpos = opt_flow = None
    tlast = time.time()
    next_heartbeat_time = time.time()
    print('awaiting data')
    while not threadkill.is_set():
        m = master.recv_match(type=['LOCAL_POSITION_NED', 'ATTITUDE_QUATERNION', 'STATUSTEXT', 'OPTICAL_FLOW_RAD'], blocking=True)
        if att is not None and lpos is not None:
            print(f'\033[1A\r', end='', flush=True)
        if m.get_type() == 'LOCAL_POSITION_NED':
            ts.append(m.time_boot_ms*1e-3)
            xs.append(m.x)
            ys.append(m.y)
            zs.append(m.z)
            vxs.append(m.vx)
            vys.append(m.vy)
            vzs.append(m.vz)
            ts[:]  = ts[-limit:]
            xs[:]  = xs[-limit:]
            ys[:]  = ys[-limit:]
            zs[:]  = zs[-limit:]
            vxs[:] = vxs[-limit:]
            vys[:] = vys[-limit:]
            vzs[:] = vzs[-limit:]
            lpos = m
        elif m.get_type() == 'ATTITUDE_QUATERNION':
            att = m
        elif m.get_type() == 'STATUSTEXT':
            text = str(m.text[:m.text.index(0)], encoding='utf-8')
            print(f'\033[2K[{mavlink_severity_names[m.severity]:>10s}] {text}', file=sys.stderr)
        elif m.get_type() == 'OPTICAL_FLOW_RAD':
            opt_flow = m

        if att is not None and lpos is not None:
            siny = +2.0 * (att.q1*att.q4 + att.q2*att.q3)
            cosy = +1.0 - 2.0 * (att.q3**2 + att.q4**2)
            heading = np.arctan2(siny, cosy)
            print(f'{lpos.x: 7.5f} {lpos.y: 7.5f} {lpos.z: 7.5f} {lpos.vx: 7.5f} {lpos.vy: 7.5f} {lpos.vz: 7.5f}\n'
                  f'heading: {heading*180./np.pi:.2f}°', end='', flush=True)

        if time.time() - tlast > 0.125:
            callback((ts, xs, ys, zs, vxs, vys, vzs))
            tlast = time.time()

        heartbeat_time = time.time()
        if heartbeat_time > next_heartbeat_time:
            master.mav.heartbeat_send(mavutil.mavlink.MAV_TYPE_GCS,
                    mavutil.mavlink.MAV_AUTOPILOT_GENERIC, 0, 0, 0)
            next_heartbeat_time = heartbeat_time + 1

class InteractiveGraph(pg.GraphicsLayoutWidget):

    # Signal to indicate new data acquisition
    # Note: signals need to be defined inside a QObject class/subclass
    data_acquired = pyqtSignal(tuple)

    def __init__(self):
    
        super().__init__()

        #self.plot = self.addPlot()
        #self.spectrum = self.plot.plot()
        #self.plot.enableAutoRange(pg.ViewBox.XYAxes)
        self.p1 = self.addPlot(title='Position (NED)')
        self.p1.setLabel('left', 'Distance', units='m')
        self.p1.showLabel('bottom', show=False)
        self.plotx = self.p1.plot(name='X (North)', pen=(0, 2))
        self.ploty = self.p1.plot(name='Y (East)', pen=(1, 2))
        self.p2 = self.addPlot(title='Velocity (NED)')
        self.p2.setLabel('left', 'Speed', units='m/s')
        self.p2.showLabel('bottom', show=False)
        self.plotvx = self.p2.plot(name='VX (North)', pen=(0, 3))
        self.plotvy = self.p2.plot(name='VY (East)', pen=(1, 3))
        self.nextRow()
        self.p3 = self.addPlot()
        self.p3.setLabel('left', 'Distance', units='m')
        self.p3.setLabel('bottom', 'Time', units='s')
        self.plotz = self.p3.plot(name='Z (Down)', pen=(0, 1))
        self.p4 = self.addPlot()
        self.p4.setLabel('left', 'Speed', units='m/s')
        self.p4.setLabel('bottom', 'Time', units='s')
        self.plotvz = self.p4.plot(name='VZ (Down)', pen=(0, 1))
        self.p1.showGrid(x=True, y=True)
        self.p2.showGrid(x=True, y=True)
        self.p3.showGrid(x=True, y=True)
        self.p4.showGrid(x=True, y=True)

        # Connect the signal
        self.data_acquired.connect(self.update_data)

        # Make and start the background thread to acquire data
        # Pass it the signal.emit as the callback function
        self.threadkill = threading.Event()
        self.thread = threading.Thread(target=readloop, args=(self.data_acquired.emit, self.threadkill))
        self.thread.start()

    # Kill our data acquisition thread when shutting down
    def closeEvent(self, close_event):
        self.threadkill.set()

    # Slot to receive acquired data and update plot
    @pyqtSlot(tuple)
    def update_data(self, data):
        ts, xs, ys, zs, vxs, vys, vzs = data
        self.plotx.setData(ts, xs)
        self.ploty.setData(ts, ys)
        self.plotz.setData(ts, zs)
        self.plotvx.setData(ts, vxs)
        self.plotvy.setData(ts, vys)
        self.plotvz.setData(ts, vzs)

win = InteractiveGraph()
win.setWindowTitle('MAVLink Plotter')
win.show()

pg.setConfigOptions(antialias=True)

if __name__ == '__main__':
    import sys
    if (sys.flags.interactive != 1) or not hasattr(QtCore, 'PYQT_VERSION'):
        QtGui.QApplication.instance().exec_()
        print()
