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

px4_main_modes = ('?', 'MANUAL', 'ALTCTL', 'POSCTL', 'AUTO', 'ACRO',
                  'OFFBOARD', 'STABILIZED', 'RATTITUDE', 'SIMPLE')

px4_sub_modes = ('', 'READY', 'TAKEOFF', 'LOITER', 'MISSION', 'RTL', 'LAND',
                 'RTGS', 'FOLLOW_TARGET', 'PRECLAND')

def px4_mode_name(m):
    m >>= 16
    main, sub = m & 0xff, m >> 8
    if sub:
        return f'{px4_main_modes[main]} {px4_sub_modes[sub]}'
    else:
        return px4_main_modes[main]

ts, xs, ys, zs, vxs, vys, vzs = [], [], [], [], [], [], []
def readloop(callback, threadkill, limit=1000):
    atts, lposes, opt_flows = [], [], []
    heartbeat = None
    tlast = time.time()
    next_heartbeat_time = time.time()
    from pymavlink.mavutil import mavlink
    print('awaiting data')
    while not threadkill.is_set():
        m = master.recv_match(type=['LOCAL_POSITION_NED', 'ATTITUDE_QUATERNION', 'STATUSTEXT', 'OPTICAL_FLOW_RAD', 'HEARTBEAT'], blocking=True)

        if m.get_type() == 'LOCAL_POSITION_NED':
            lposes.append(m)
        elif m.get_type() == 'ATTITUDE_QUATERNION':
            atts.append(m)
        elif m.get_type() == 'OPTICAL_FLOW_RAD':
            opt_flows.append(m)
        elif m.get_type() == 'HEARTBEAT':
            heartbeat = m

        if atts and lposes and heartbeat:
            print(f'\033[1A\r', end='', flush=True)

        if m.get_type() == 'STATUSTEXT':
            text = str(m.text[:m.text.index(0)], encoding='utf-8')
            print(f'\033[2K[{mavlink_severity_names[m.severity]:>10s}] {text}', file=sys.stderr)

        if atts and lposes and heartbeat:
            att, lpos = atts[-1], lposes[-1]
            siny = +2.0 * (att.q1*att.q4 + att.q2*att.q3)
            cosy = +1.0 - 2.0 * (att.q3**2 + att.q4**2)
            heading = np.arctan2(siny, cosy)
            #type, autopilot, base_mode, custom_mode, system_status, mavlink_version
            status = mavlink.enums['MAV_STATE'][heartbeat.system_status].name[10:]
            vtype  = mavlink.enums['MAV_TYPE'][heartbeat.type].name[9:]
            if heartbeat.base_mode & 1:
                mode = px4_mode_name(heartbeat.custom_mode)
            else:
                mode = mavlink.enums['MAV_MODE_FLAG'][heartbeat.base_mode & 63].name
            print(f'\033[2K{lpos.x: 7.5f} {lpos.y: 7.5f} {lpos.z: 7.5f} {lpos.vx: 7.5f} {lpos.vy: 7.5f} {lpos.vz: 7.5f}\n'
                  f'\033[2Kheading: {heading*180./np.pi:.2f}°, {vtype} {status} mode: {mode}', end='', flush=True)

        if time.time() - tlast > 0.125:
            lposes = lposes[-limit:]
            atts = atts[-limit:]
            opt_flows = opt_flows[-limit:]
            callback((atts[:], lposes[:], opt_flows)[:])
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
        atts, lposes, opt_flows = data
        self._update_lpos_plot(lposes)

    def _update_lpos_plot(self, lposes):
        ts = [lpos.time_boot_ms*1e-3 for lpos in lposes]
        self.plotx.setData(ts,  [lpos.x for lpos in lposes])
        self.ploty.setData(ts,  [lpos.y for lpos in lposes])
        self.plotz.setData(ts,  [lpos.z for lpos in lposes])
        self.plotvx.setData(ts, [lpos.vx for lpos in lposes])
        self.plotvy.setData(ts, [lpos.vy for lpos in lposes])
        self.plotvz.setData(ts, [lpos.vz for lpos in lposes])

pg.setConfigOptions(antialias=True)
win = InteractiveGraph()
win.setWindowTitle('MAVLink Plotter')
win.show()

if __name__ == '__main__':
    import sys
    if (sys.flags.interactive != 1) or not hasattr(QtCore, 'PYQT_VERSION'):
        QtGui.QApplication.instance().exec_()
        print()
