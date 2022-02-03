from asyncore import loop
import sys
from bleak import BleakClient, BleakError, BleakScanner
import signal
import time
import asyncio
import os

SERIAL_SVC_UUID = "0000ffe0-0000-1000-8000-00805f9b34fb"
SERIAL_CHR_UUID = "0000ffe1-0000-1000-8000-00805f9b34fb"

port = 'ble://D0:B5:C2:94:3A:98'
# port = 'ble://50:65:83:6D:F6:73'
reconnect = True
close_event = None
ctrl_c_cnt = 0

pipe = None


def close():
    print("close")
    if main_future:
        main_future.cancel()


async def ble_serial_close(ble_client):
    if ble_client:
        #  and ble_client.is_connected == True:
        print("--- Closing BLE connection: ", ble_client)
        try:
            await ble_client.stop_notify(SERIAL_CHR_UUID)
            await ble_client.disconnect()
        except BleakError as err:
            print("--- BLE connection close error:", err)
    else:
        print("--- BLE connection not open: ", ble_client)


def ble_disconnect_handler(client):
    print("--- BLE connection disconected")
    if reconnect:
        close_event.set()


async def ble_serial_open(address, infd, outfd):
    global close_event
    close_event = asyncio.Event()

    def process_ble_data(char, data):
        nonlocal outfd
        # print('btx:', data)
        try:
            os.write(outfd, data)
        except OSError:
            pass

    print("--- Connecting to BLE device", address)
    ble_client = None
    while not close_event.is_set():
        try:
            print("--- BLE discovery...")
            devices = await BleakScanner.discover(timeout=3)
            print("--- BLE devices discovered:")
            target_dev = None
            for d in devices:
                print("---  ", d.address, '(', d.name, ")")
                if d.address == address:
                    target_dev = d
            if not target_dev:
                print("--- BLE device not found/scan timeout")
                await asyncio.sleep(3)
                continue

            ble_client = BleakClient(target_dev)
            print("--- Ble client/GATT server", ble_client)
            ble_client.set_disconnected_callback(ble_disconnect_handler)
            if await ble_client.connect(timeout=3):
                print("--- BLE device connected")
                await ble_client.start_notify(SERIAL_CHR_UUID, process_ble_data)
                print("--- BLE connection open")
                print("--- Reading serial data")
                while not close_event.is_set():
                    await asyncio.sleep(0.001)
                    # try:
                    #     # b = os.read(infd, 1024)
                    #     # if len(b) > 0:
                    #     # print('stx:', b)
                    #     await ble_client.write_gatt_char(SERIAL_CHR_UUID, b)
                    # except BlockingIOError:
                    #     await asyncio.sleep(0.001)
                # await close_event.wait()
                print("read loop end")
                print("close event:", close_event)
                if reconnect and close_event.is_set():
                    close_event = asyncio.Event()
            else:
                print("--- BLE connection timeout, reconnecting...")
        except BleakError as err:
            print("--- BLE connect error:", err)
        except TimeoutError:
            print("--- BLE connect timeout")
            print("close event:", close_event)
        except asyncio.CancelledError:
            print("--- BLE connect/read loop canceled")
            break
        finally:
            print("finally")
            await ble_serial_close(ble_client)


def signalHandler(sig, frame):
    global ctrl_c_cnt, reconnect
    try:
        print()
        if sig == signal.SIGINT:
            print("--- Exiting on keyboard interrupt")
        else:
            print("--- Exiting on signal", sig)
        reconnect = False
        if ctrl_c_cnt == 0:
            close()
        ctrl_c_cnt += 1
        if ctrl_c_cnt > 2:
            print("--- Terminating on repeted signal")
            sys.exit(1)
    except:
        sys.exit(1)


async def main():
    address = port[6:]
    # path = '/dev/tnt0'
    # print('Opening target device:', path)
    outfd = os.dup(sys.stdout.fileno())
    infd = os.dup(sys.stdin.fileno())
    os.set_blocking(infd, False)

    # outfd = os.open(sys.stdout.fileno(), os.O_WRONLY | os.O_NONBLOCK)
    # infd = os.open(sys.stdin.fileno(), os.O_RDONLY | os.O_NONBLOCK)
    # os.O_NOCTTY |
    await ble_serial_open(address, infd, outfd)
    os.close(outfd)
    os.close(infd)


if __name__ == '__main__':
    global main_future
    signal.signal(signal.SIGINT, signalHandler)
    try:
        signal.signal(signal.SIGQUIT, signalHandler)
    except AttributeError:
        pass
    # signal.signal(signal.SIGKILL, signalHandler)
    signal.signal(signal.SIGTERM, signalHandler)
    loop = asyncio.get_event_loop()
    main_future = asyncio.ensure_future(main())
    loop.run_until_complete(main_future)
    # asyncio.run(main())
