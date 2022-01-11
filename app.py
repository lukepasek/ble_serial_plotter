
from kivy.properties import StringProperty
from kivy.uix.widget import Widget
from kivy.app import App

import asyncio
import serial
import queue

from kivy.config import Config
Config.set('kivy', 'log_level', 'error')


eol = b'\r\n'

root = None
cnt = 0
ser_dev = None
port = '/dev/ttyUSB0'
port_speed = 115200
buffer = bytearray()


def parse(data, start, end):
    values = {}
    tag_parse = True
    tag = bytearray()
    val = bytearray()

    for c in memoryview(data)[start:end]:
        if c == 10 or c == 13:
            break
        if tag_parse:
            if c == 58:
                tag_parse = False
            else:
                tag.append(c)
        else:
            if c == 32:
                values[tag.decode()] = float(str(val.decode()))
                tag_parse = True
                tag.clear()
                val.clear()
            else:
                val.append(c)
    try:
        values[tag.decode()] = float(str(val.decode()))
    except ValueError:
        print('ValueError: Missing value for:', tag.decode())
    return values


def process_data(data):
    global buffer
    buffer.extend(data)
    if data.rfind(eol) > -1:
        ls = 0
        le = buffer.find(eol)
        while le > -1:
            if buffer[ls] != 35 and le > ls:
                values = parse(buffer, ls, le)
                root.set_values(values)
            ls = le + 2
            le = buffer.find(eol, ls)
        buffer = buffer[ls:]


class InputChunkProtocol(asyncio.Protocol):

    def connection_made(self, transport):
        self.transport = transport

    def data_received(self, data):
        process_data(data)


class ValueDisplayWidget(Widget):
    value0 = StringProperty()
    value1 = StringProperty()
    value2 = StringProperty()


class ValueDisplayApp(App):
    widget = None
    voltage_avg = 0
    current_avg = 0
    samples = 10
    vq = queue.Queue(maxsize=samples)
    cq = queue.Queue(maxsize=samples)

    def build(self):
        self.widget = ValueDisplayWidget()
        return self.widget

    def moving_avg(self, q, v, a):
        s = v/self.samples
        if q.empty():
            a = v
        while not q.full():
            q.put(s)
        return a - q.get() + s

    def set_values(self, values):
        if values and 'vi' in values and 'cr' in values:
            root.set_voltage(values['vi'])
            root.set_current(values['cr'])

    def set_voltage(self, v):
        self.voltage_avg = self.moving_avg(self.vq, v, self.voltage_avg)

    def set_current(self, v):
        self.current_avg = self.moving_avg(self.cq, v, self.current_avg)

    def update(self):
        if self.widget:
            self.widget.value1 = str("%.3f" % round(self.voltage_avg, 3))
            self.widget.value2 = str("%.2f" % round(self.current_avg, 2))


def poll_serial():
    global ser_dev, port, port_speed, buffer
    if not ser_dev:
        ser_dev = serial.Serial(port, port_speed)
    if ser_dev.in_waiting == 0:
        return
    data = ser_dev.read_all()
    process_data(data)


async def run_app(root, cancelable):
    print("App start")
    await App.async_run(root, async_lib='asyncio')
    print("App finished")
    for c in cancelable:
        c.cancel()


async def read_serial():
    print("serial data polling start")
    # loop = asyncio.get_event_loop()
    # transport, protocol = await serial_asyncio.create_serial_connection(loop, InputChunkProtocol, port, baudrate=port_speed)
    try:
        while True:
            poll_serial()
            await asyncio.sleep(0.05)
    except asyncio.CancelledError:
        print("serial data polling canceled")
    print("serial data polling end")


async def update_values():
    print("value update start")
    try:
        while True:
            root.update()
            await asyncio.sleep(0.5)
    except asyncio.CancelledError:
        print("value update canceled")
    print("value update end")

if __name__ == '__main__':
    def root_func():
        global root
        root = ValueDisplayApp()
        read_serial_task = asyncio.ensure_future(read_serial())
        update_task = asyncio.ensure_future(update_values())
        return asyncio.gather(run_app(root, [read_serial_task, update_task]), read_serial_task, update_task)

    loop = asyncio.get_event_loop()
    loop.run_until_complete(root_func())
    loop.close()
