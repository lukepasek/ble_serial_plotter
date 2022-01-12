from logging import shutdown
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

port = '/dev/ttyUSB0'
# port = 'ble://50:65:83:6D:F6:73'
# port = 'ble://D0:B5:C2:94:3A:98'
# port = "test"
port_speed = 115200
reconnect = True
ble_client = None
close_event = None
ctrl_c_cnt = 0
parser = None
start_time = time.monotonic_ns()
num_samples = 3600
max_range = 1200
serial_port = None
t = deque([])
cnt = -1
sample_frame = None
labels = ["vi", "vo", "io", "ib"]
label_map = {"vi": "vi", "cr": "ib", "mcl": "io"}
colors = {
    "vi": 'r',
    "vo": 'y',
    "io": 'm',
    "ib": 'g',
    "pwr": 'w'
}
curves = {}
data = {}
plotWidget = None
app = None
data_changed = False


def initQtApp():
    global plotWidget
    QtGui.QApplication.setAttribute(QtCore.Qt.AA_EnableHighDpiScaling)
    QtGui.QApplication.setAttribute(QtCore.Qt.AA_UseHighDpiPixmaps)
    app = QtGui.QApplication([])
    app.setWindowIcon(QtGui.QIcon("icon.png"))
    # pyqtgraph.setConfigOption('antialias', True)
    pyqtgraph.setConfigOption('background', "#202020")
    plotWidget = pyqtgraph.plot()
    plotWidget.clipToView = True
    plotWidget.autoDownsample = True
    plotWidget.skipFiniteCheck = True
    plotWidget.setYRange(-20, 20, padding=0)
    plotWidget.setXRange(0, max_range, padding=0)
    plotWidget.showGrid(x=True, y=True, alpha=0.3)
    plotWidget.resize(800, 600)

    for l in labels:
        curve = pyqtgraph.PlotCurveItem(
            pen=({'color': colors[l], 'width': 2}), skipFiniteCheck=True, name=l)
        plotWidget.addItem(curve)
        curve.setPos(0, 0)
        curves[l] = curve
        d = deque([])
        data[l] = d
        curve.setData(x=np.array(t, copy=False), y=np.array(d, copy=False))

    inf1 = pyqtgraph.InfiniteLine(movable=True, angle=0, pen='y', hoverPen=(0, 200, 0), label='{value:0.2f}',
                                  labelOpts={'color': 'y', 'movable': True, 'fill': (0, 0, 200, 100)})
    inf1.setPos([0, 0])
    # vout_high = pg.InfiniteLine(movable=True, angle=0, pen='y', bounds=[-2, 38], hoverPen=(0, 200, 0), label='Absorbtion/Bulk out: {value:0.2f}V',
    #                             labelOpts={'color': 'y', 'movable': True, 'fill': (0, 0, 200, 100)})
    # vout_high.setPos([0, 14.7])
    # iout_max = pg.InfiniteLine(movable=True, angle=0, pen='r', bounds=[-2, 38], hoverPen=(0, 200, 0), label='Current limit: {value:0.2f}A',
    #                            labelOpts={'color': 'r', 'movable': True, 'fill': (0, 0, 200, 100)})
    # iout_max.setPos([0, 2.5])

    plotWidget.addItem(inf1)
    # p.addItem(vout_high)
    # p.addItem(iout_max)
    return app


def plot(values):
    global cnt, t, sample_frame, data_changed

    cnt += 1
    data_changed = True
    try:
        ts = values["t"]
    except KeyError:
        ts = (time.monotonic_ns() - start_time)/100000000
    if cnt >= num_samples:
        t.rotate(-1)
        t[-1] = ts
    else:
        t.append(ts)

    for l in labels:
        val = None
        d = data[l]
        if l in values:
            val = values[l]
        else:
            val = 0
        if cnt >= num_samples:
            d.rotate(-1)
            d[-1] = val
        else:
            d.append(val)


class StreamParser:
    EOL = b'\r\n'
    line_buffer = bytearray(1024*8)
    buffer_mem = memoryview(line_buffer)
    buffer_ptr = 0
    buffer_offset = 0
    line_callback = None

    def set_line_callback(self, callback):
        if (callable(callback)):
            self.line_callback = callback

    def __init__(self, callback):
        self.set_line_callback(callback)

    def parse_line(self, line):
        global sample_frame
        values = {}
        tag = bytearray()
        val = bytearray()
        tag_parse = True
        for c in line:
            if tag_parse:
                if c == 58:
                    tag_parse = False
                elif c != 32 or c != 9:
                    tag.append(c)
            else:
                if c == 45 or c == 46 or c >= 48 and c <= 57:
                    val.append(c)
                elif c == 32 or c == 9 or c == 10 or c == 13:
                    if len(val) > 0:
                        try:
                            values[tag.decode('ascii')] = float(
                                val.decode('ascii'))
                        except ValueError as e:
                            print("-- Value error - key/value skipped:",
                                  tag, val, line, e)
                    tag.clear()
                    val.clear()
                    tag_parse = True

        if (len(tag) > 0 and len(val) > 0):
            values[tag.decode('ascii')] = float(val.decode('ascii'))
        sample_frame = {}
        sample_frame["__ts"] = time.monotonic_ns()
        # for index, (key, value) in enumerate(values):
        mapped_values = {}
        for key, value in values.items():
            mapped_key = label_map.get(key)
            if mapped_key != None:
                mapped_values[mapped_key] = value
            else:
                mapped_values[key] = value
            sample_frame[key] = value
        return mapped_values

    def process_data(self, data):
        eol = self.EOL
        buffer = self.line_buffer
        data_len = len(data)
        mem_offset = self.buffer_ptr
        self.buffer_ptr = self.buffer_ptr+data_len
        dst_mem = self.buffer_mem[mem_offset:self.buffer_ptr]
        dst_mem[:] = data
        if buffer.rfind(eol, self.buffer_offset, self.buffer_ptr) > -1:
            ls = self.buffer_offset
            le = buffer.find(eol, ls, self.buffer_ptr)
            while le > -1:
                if buffer[ls] != 35 and le > ls:
                    values = self.parse_line(self.buffer_mem[ls:le])
                    if not self.line_callback == None:
                        self.line_callback(values)
                ls = le + 2
                le = buffer.find(eol, ls, self.buffer_ptr)
            if ls == self.buffer_ptr:
                self.buffer_ptr = 0
                self.buffer_offset = 0
            else:
                if self.buffer_ptr > (1024*6):
                    left = self.buffer_ptr-self.buffer_offset
                    dst_mem = self.buffer_mem[0:left]
                    dst_mem[:] = self.buffer_mem[self.buffer_offset:self.buffer_ptr]
                    self.buffer_ptr = left
                    self.buffer_offset = 0
                else:
                    self.buffer_offset = ls

    # def append(self, data):
    #     sys.stdout.buffer.write(data)
    #     sys.stdout.buffer.flush()
    #     self.line_buffer.extend(data)
    #     line_src = self.line_buffer
    #     sol_ptr = 0
    #     eol_ptr = line_src.find(self.eol, sol_ptr)
    #     while eol_ptr > -1:
    #         line = line_src[sol_ptr:eol_ptr]
    #         if len(line) > 0 and line[0] != 35:
    #             try:
    #                 values = self.parse_line(line)
    #                 if not self.line_callback == None:
    #                     self.line_callback(values)
    #             except ValueError as e:
    #                 print("-- Value error - line skipped:", line, e)
    #         sol_ptr = eol_ptr + 1
    #         eol_ptr = line_src.find(self.eol, sol_ptr)
    #     if sol_ptr:
    #         self.line_buffer = line_src[sol_ptr:]

    # def append2(self, data):
    #     buffer = self.line_buffer
    #     buffer.extend(data)
    #     if data.rfind(EOL) > -1:
    #         ls = 0
    #         le = buffer.find(EOL)
    #         while le > -1:
    #             if buffer[ls] != 35 and le > ls:
    #                 values = parse_line(buffer, ls, le)
    #                 if not self.line_callback == None:
    #                     self.line_callback(values)
    #             ls = le + 2
    #             le = buffer.find(EOL, ls)
    #         buffer = buffer[ls:]


def poll_serial():
    global serial_port
    if not serial_port and reconnect:
        serial_port = serial.Serial(port, port_speed, timeout=0)
        print("--- Serial port open:", port)
    data = serial_port.read_all()
    while data and len(data) > 0:
        parser.process_data(data)
        data = serial_port.read_all()


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


def ble_disconnect_handler(client):
    print("--- BLE connection disconected")
    if reconnect:
        print("--- BLE connection reconnecting")
        close_event.set()


async def ble_serial_open(address):
    global close_event, ble_client
    close_event = asyncio.Event()

    print("--- Connecting to BLE device", address)
    while not close_event.is_set():
        try:
            ble_client = BleakClient(address)
            ble_client.set_disconnected_callback(ble_disconnect_handler)
            if await ble_client.connect(timeout=3):
                await ble_client.start_notify(SERIAL_CHR_UUID, lambda i, d: parser.append(d))
                print("--- BLE connection open")
                await close_event.wait()
                if reconnect:
                    close_event = asyncio.Event()
        except BleakError as err:
            print("--- BLE connect error:", err)
        except TimeoutError:
            print("--- BLE connect timeout")
        except GeneratorExit as err:
            pass
        except asyncio.CancelledError:
            break
        finally:
            if (ble_client.is_connected == True):
                await ble_serial_close()

close_app = None
close = None


def signalHandler(sig, frame):
    global ctrl_c_cnt
    try:
        print()
        if sig == signal.SIGINT:
            print("--- Exiting on keyboard interrupt")
        else:
            print("--- Exiting on signal", sig)
        if ctrl_c_cnt == 0:
            if close_app != None:
                close_app()
            else:
                close()
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


def initHttpServer(loop):
    http_server = web.Application()
    http_server.router.add_route('GET', '/', handleHttp)
    handler = http_server.make_handler()
    f = loop.create_server(handler, '0.0.0.0', 8585)
    srv = loop.run_until_complete(f)
    return srv


def update_plot():
    global data_changed
    if data_changed:
        data_changed = False
        plotWidget.setUpdatesEnabled(False)
        start = t[0]
        end = t[-1]
        if end > max_range:
            plotWidget.setXRange(end-max_range, end, padding=0)
        else:
            plotWidget.setXRange(start, start+max_range, padding=0)
        for l in labels:
            c = curves[l]
            d = data[l]
            c.setData(x=np.array(t, copy=False), y=np.array(d, copy=False))
        plotWidget.setUpdatesEnabled(True)


def main():
    global parser, app
    print()
    parser = StreamParser(None)
    app = initQtApp()
    timer2 = QtCore.QTimer()
    timer2.timeout.connect(update_plot)
    timer2.start(100)

    # def plot_init(values):
    #     try:
    #         ts = values["t"]
    #         p.setXRange(ts, ts+max_range, padding=0)
    #         print("--- Plot range start moved to ", ts)
    #         parser.set_line_callback(plot)
    #     except AttributeError:
    #         pass

    # parser.set_line_callback(plot_init)

    parser.set_line_callback(plot)

    if port == "test":
        import random
        tick = 0
        print("--- Serial plotter with test data ---")

        def poll_test_data():
            nonlocal tick
            import math
            vi = 12 + random.randint(0, 100)/100
            vo = 12 + random.randint(0, 100)/100
            data = "t:"+str(tick)+" vi:"+str(vi)+" vo:"+str(vo) + \
                " ib:"+str(math.sin(tick/100)*5)+"\r\n"
            tick += 1
            parser.append(data.encode())
            # app.processEvents()

        timer1 = QtCore.QTimer()
        timer1.timeout.connect(poll_test_data)
        timer1.start(100)
        app.exec_()

    elif port.startswith('ble://'):
        address = port[6:]
        # p.setWindowTitle('Serial plotter on BLE device '+address)
        print("--- Serial plotter on ", port, "---")
        print("--- Quit: Ctrl-C | Toggle parser debug: Ctrl-D ---")
        loop = QEventLoop(app)
        srv = initHttpServer(loop)
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
        # p.setWindowTitle('Serial plotter on '+port+' at '+str(port_speed))
        def close():
            global reconnect
            reconnect = False
            if serial_port != None and serial_port.is_open:
                print("--- Closing serial port")
                serial_port.cancel_read()
                serial_port.cancel_write()
                serial_port.close()

        timer = QtCore.QTimer()
        timer.timeout.connect(poll_serial)
        timer.start(10)
        app.aboutToQuit.connect(close)
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
