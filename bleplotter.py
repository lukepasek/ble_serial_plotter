from pyqtgraph.Qt import QtGui, QtCore
import numpy as np
import pyqtgraph
import serial
import sys
from bleak import BleakClient, BleakError, exc
from qasync import QEventLoop
import signal
from collections import deque
import time
import asyncio
from aiohttp import web

enableOpenGl = False

if enableOpenGl:
    try:

        import OpenGL
        pyqtgraph.setConfigOption('useOpenGL', True)
        pyqtgraph.setConfigOption('enableExperimental', True)
    except Exception as e:
        print(
            f"Enabling OpenGL failed with {e}. Will result in slow rendering. Try installing PyOpenGL.")

SERIAL_SVC_UUID = "0000ffe0-0000-1000-8000-00805f9b34fb"
SERIAL_CHR_UUID = "0000ffe1-0000-1000-8000-00805f9b34fb"


# port = '/dev/ttyUSB0'
# port = 'ble://50:65:83:6D:F6:73'
port = 'ble://D0:B5:C2:94:3A:98'
port_speed = 115200
reconnect = True

app = QtGui.QApplication([])
app.setWindowIcon(QtGui.QIcon("icon3.png"))
ble_client = None
close_event = None
ctrl_c_cnt = 0
parser = None
start_time = time.monotonic_ns()

num_samples = 30000
max_range = 600

# pyqtgraph.setConfigOption('antialias', True)
pyqtgraph.setConfigOption('background', "#202020")
p = pyqtgraph.plot()
p.setYRange(-20, 20, padding=0)
p.setXRange(0, max_range, padding=0)
p.showGrid(x=True, y=True, alpha=0.3)
p.resize(900, 900)

labels = ["vi", "vo", "io", "ib"]

colors = {
    "vi": 'r',
    "vo": 'y',
    "io": 'm',
    "ib": 'g',
    "pwr": 'w'
}

curves = {}
data = {}

serial_port = None
t = deque([])
cnt = -1
sample_frame = None

for l in labels:
    curve = pyqtgraph.PlotCurveItem(
        pen=({'color': colors[l], 'width': 2}), skipFiniteCheck=True, name=l)
    p.addItem(curve)
    curve.setPos(0, 0)
    curves[l] = curve
    data[l] = deque([])

inf1 = pyqtgraph.InfiniteLine(movable=True, angle=0, pen='y', bounds=[-2, 38], hoverPen=(0, 200, 0), label='{value:0.2f}',
                              labelOpts={'color': 'y', 'movable': True, 'fill': (0, 0, 200, 100)})
inf1.setPos([0, 0])
# vout_high = pg.InfiniteLine(movable=True, angle=0, pen='y', bounds=[-2, 38], hoverPen=(0, 200, 0), label='Absorbtion/Bulk out: {value:0.2f}V',
#                             labelOpts={'color': 'y', 'movable': True, 'fill': (0, 0, 200, 100)})
# vout_high.setPos([0, 14.7])
# iout_max = pg.InfiniteLine(movable=True, angle=0, pen='r', bounds=[-2, 38], hoverPen=(0, 200, 0), label='Current limit: {value:0.2f}A',
#                            labelOpts={'color': 'r', 'movable': True, 'fill': (0, 0, 200, 100)})
# iout_max.setPos([0, 2.5])

p.addItem(inf1)
# p.addItem(vout_high)
# p.addItem(iout_max)


def plot(values):
    global cnt, t, sample_frame

    cnt += 1
    try:
        ts = values["t"]
    except KeyError:
        ts = (time.monotonic_ns() - start_time)/1000000
    if cnt > num_samples:
        t.rotate(-1)
        t[-1] = ts
    else:
        t.append(ts)

    # start = p.range.right()
    # left = p.range.left()
    # if ts > left:
    #     p.setXRange(ts-p.range.width(), ts)

    empty = []
    for l in labels:
        if l in values:
            c = curves[l]
            d = data[l]
            if cnt > num_samples:
                d.rotate(-1)
                d[-1] = values[l]
            else:
                d.append(values[l])
            try:
                c.setData(x=np.array(t, copy=False), y=np.array(d, copy=False))
            except Exception as e:
                print(e)
        else:
            empty.append(l)
    # update_ui()


class StreamParser:
    eol = b'\n'
    line_buffer = bytearray()
    line_callback = None

    def set_line_callback(self, callback):
        if (callable(callback)):
            self.line_callback = callback

    def __init__(self, callback):
        self.set_line_callback(callback)

    def parse_line(self, line):
        global sample_frame
        values = {}
        tag_parse = True
        tag = bytearray()
        val = bytearray()
        for c in line:
            if tag_parse:
                if c == 58:
                    tag_parse = False
                elif c != 32 or c != 9:
                    tag.append(c)
            else:
                if c == 32 or c == 9 or c == 10 or c == 13:
                    values[tag.decode('ascii')] = float(
                        str(val.decode('ascii')))
                    tag_parse = True
                    tag.clear()
                    val.clear()
                elif c == 45 or c == 46 or c >= 48 and c <= 57:
                    val.append(c)
        if (len(tag) > 0 and len(val) > 0):
            values[tag.decode('ascii')] = float(str(val.decode('ascii')))
        sample_frame = {}
        sample_frame["__ts"] = time.monotonic_ns()
        # for index, (key, value) in enumerate(values):
        for key, value in values.items():
            sample_frame[key] = value
        return values

    def append(self, data):
        sys.stdout.buffer.write(data)
        sys.stdout.buffer.flush()
        self.line_buffer.extend(data)
        line_src = self.line_buffer
        sol_ptr = 0
        eol_ptr = line_src.find(self.eol, sol_ptr)
        while eol_ptr > -1:
            line = line_src[sol_ptr:eol_ptr]
            if len(line) > 0 and line[0] != 35:
                try:
                    values = self.parse_line(line)
                    if not self.line_callback == None:
                        self.line_callback(values)
                except ValueError as e:
                    print("-- Value error - line skipped:", line, e)
            sol_ptr = eol_ptr + 1
            eol_ptr = line_src.find(self.eol, sol_ptr)
        if sol_ptr:
            self.line_buffer = line_src[sol_ptr:]


def update_ui():
    # app.processEvents()
    pass


def poll_serial():
    global serial_port
    if not serial_port:
        serial_port = serial.Serial(port, port_speed)

    parser.append(serial_port.readline())
    # update_ui()


async def ble_serial_close():
    if ble_client != None:
        print("--- Closing BLE connection")
        try:
            await ble_client.stop_notify(SERIAL_CHR_UUID)
            await ble_client.disconnect()
        except BleakError as err:
            print("--- BLE connection error:", err)
    else:
        print("--- BLE connection not open")


def ble_disconnect_handler():
    print("--- BLE connection closed")
    if reconnect:
        close_event.set()


async def ble_serial_open(address):
    global close_event, ble_client
    close_event = asyncio.Event()

    print("--- Connecting to BLE device", address)
    try_cnt = 0
    while not close_event.is_set() and try_cnt < 50:
        try:
            ble_client = BleakClient(address)
            ble_client.set_disconnected_callback(ble_disconnect_handler)
            if await ble_client.connect(timeout=3):
                await ble_client.start_notify(SERIAL_CHR_UUID, lambda i, d: parser.append(d))
                print("--- BLE connection open")
                try_cnt = 0
                await close_event.wait()
                if reconnect:
                    close_event = asyncio.Event()
            else:
                try_cnt += 1
        except BleakError as err:
            print("--- BLE connect error:", err)
            try_cnt += 1
        except TimeoutError:
            print("--- BLE connect timeout")
            try_cnt += 1
        except GeneratorExit as err:
            pass
        except asyncio.CancelledError:
            pass
        finally:
            if (ble_client.is_connected == True):
                await ble_serial_close()


def signalHandler(sig, frame):
    global ctrl_c_cnt
    try:
        print()
        if sig == signal.SIGINT:
            print("--- Exiting on keyboard interrupt")
        else:
            print("--- Exiting on signal", sig)
        if ctrl_c_cnt == 0:
            app.quit()
        ctrl_c_cnt += 1
        if ctrl_c_cnt > 2:
            print("--- Terminating on repeted signal")
            sys.exit(1)
    except:
        sys.exit(1)


async def handleHttp(request):
    text = ""
    if sample_frame != None:
        ts = sample_frame["__ts"]
        if ts+2000000000 > time.monotonic_ns():
            for key, value in sample_frame.items():
                if key != "__ts":
                    text += "value{name=\""+key+"\", port=\"" + \
                        port+"\"} " + str(value) + "\n"
        else:
            text = "### Data is stale"+"\n"
    else:
        text = "### No valid data sampled or parsed"+"\n"
    return web.Response(text=text)


def http_server(loop):
    app = web.Application()
    app.router.add_route('GET', '/', handleHttp)
    handler = app.make_handler()
    f = loop.create_server(handler, '0.0.0.0', 8585)
    srv = loop.run_until_complete(f)
    return srv


def main():
    global close, parser
    print()

    parser = StreamParser(None)

    # def plot_init(values):
    #     try:
    #         ts = values["t"]
    #         p.setXRange(ts, ts+max_range, padding=0)
    #         print("--- Plot range start moved to ", ts)
    #         parser.set_callback(plot)
    #     except AttributeError:
    #         pass

    # parser.set_callback(plot_init)

    if port.startswith('ble://'):
        address = port[6:]
        p.setWindowTitle('Serial plotter on BLE device '+address)
        print("--- Serial plotter on ", port, "---")
        print("--- Quit: Ctrl-C | Toggle parser debug: Ctrl-D ---")
        loop = QEventLoop(app)
        srv = http_server(loop)
        print("--- Http server serving on", srv.sockets[0].getsockname())
        start_task = loop.create_task(ble_serial_open(address))

        def close():
            global reconnect
            reconnect = False
            srv.close()
            start_task.cancel()
            if close_event != None:
                close_event.set()

        app.aboutToQuit.connect(close)
        loop.run_forever()
        app.aboutToQuit.disconnect(close)
        if not start_task.done():
            loop.run_until_complete(ble_serial_close())

    else:
        p.setWindowTitle('Serial plotter on '+port+' at '+str(port_speed))

        def close():
            global reconnect
            reconnect = False
            if serial_port != None and serial_port.is_open:
                print("--- Closing serial port")
                serial_port.cancel_read()
                serial_port.cancel_write()
                serial_port.close()

        app.aboutToQuit.disconnect(close)
        timer = QtCore.QTimer()
        timer.timeout.connect(poll_serial)
        timer.start(10)
        app.exec_()


if __name__ == '__main__':
    signal.signal(signal.SIGINT, signalHandler)
    try:
        signal.signal(signal.SIGQUIT, signalHandler)
    except AttributeError:
        pass
    # signal.signal(signal.SIGKILL, signalHandler)
    signal.signal(signal.SIGTERM, signalHandler)
    main()
