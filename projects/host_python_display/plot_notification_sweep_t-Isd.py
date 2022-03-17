# -*- coding: utf-8 -*-
"""
Notifications
-------------

Example showing how to add notifications to a characteristic and handle the responses.

Updated on 2019-07-03 by hbldh <henrik.blidh@gmail.com>

"""

import sys
import asyncio
import platform

from bleak import BleakClient
from bleak.uuids import uuid16_dict

import numpy as np
import matplotlib.pyplot as plt
from matplotlib.widgets import Button
# from matplotlib import animation

import struct
import csv
from datetime import datetime

uuid16_dict = {v: k for k, v in uuid16_dict.items()}

#DISS Service
SYSTEM_ID_UUID = "0000{0:x}-0000-1000-8000-00805f9b34fb".format(
    uuid16_dict.get("System ID")
)
MODEL_NBR_UUID = "0000{0:x}-0000-1000-8000-00805f9b34fb".format(
    uuid16_dict.get("Model Number String")
)
DEVICE_NAME_UUID = "0000{0:x}-0000-1000-8000-00805f9b34fb".format(
    uuid16_dict.get("Device Name")
)
FIRMWARE_REV_UUID = "0000{0:x}-0000-1000-8000-00805f9b34fb".format(
    uuid16_dict.get("Firmware Revision String")
)
HARDWARE_REV_UUID = "0000{0:x}-0000-1000-8000-00805f9b34fb".format(
    uuid16_dict.get("Hardware Revision String")
)
SOFTWARE_REV_UUID = "0000{0:x}-0000-1000-8000-00805f9b34fb".format(
    uuid16_dict.get("Software Revision String")
)
MANUFACTURER_NAME_UUID = "0000{0:x}-0000-1000-8000-00805f9b34fb".format(
    uuid16_dict.get("Manufacturer Name String")
)

# you can change these to match your device or override them from the command line
ADDRESS = "CE:C6:E3:FA:1D:59"
CHAR_CONTROL_POINT_UUID = "2d86686a-53dc-25b3-0c4a-f0e10c8dee20"
CHAR_ADC_VAL_1_UUID = "15005991-b131-3396-014c-664c9867b917"
CHAR_LED_STATE_UUID = "5a87b4ef-3bfa-76a8-e642-92933c31434f"
CHAR_DAC_DATA_UUID = "772ae377-b3d2-4f8e-4042-5481d1e0098c"
filename = "sensor_Isd-t.csv"

NUM_DATA = 4
NUM_CHANNEL = 4
LOG_INTERVAL = 64
# Chnl_read = [8, 9, 10, 11]
Rs = 100
Vd = 0.5
# Vds = -0.4 # real sensor
Vds = -1.4   # fake sensor
Vgs = 0.1
Vs = Vd - Vds
Vg = Vgs + Vs
f_exit = False
plt.ion()

counter = 0
prevCounter = 0
ADCConversionFinishFlg = False
scatter = []
tdata = []


def onclick_button_exit(event):
    global f_exit
    f_exit = True


def notification_handler(sender, data):
    global fig, ax, scatter, xdata, ydata, tdata
    global counter, ADCConversionFinishFlg
    """Simple notification handler which prints the data received."""
    time_now = datetime.now().time()
    print("{0}: {1}".format(sender, data))

    # https://stackoverflow.com/questions/10944621/dynamically-updating-plot-in-matplotlib
    floatADC = struct.unpack_from('<ffffffffffffIIII', data)
    print(floatADC)

    ADCConversionFinishFlg = True
    rx_time = counter
    tdata.append(time_now)

    counter += 1

    # ydata = np.append(ydata[0:], floatADC[9])
    # xdata = np.append(xdata[0:], rx_time)
    # line.set_xdata(xdata)
    # line.set_ydata(ydata)

    ydata_new = np.asarray(floatADC[8:12]).reshape((4, 1))   #   Channel 11 - 8
    ydata = np.concatenate((ydata, ydata_new), axis=1)

    xdata_new = np.asarray([rx_time] * 4).reshape((4, 1))
    xdata = np.concatenate((xdata, xdata_new), axis=1)

    if (counter % LOG_INTERVAL) == LOG_INTERVAL-1:
        with open(filename, mode='a', newline='') as csv_file:
            csv_writer = csv.writer(csv_file)
            for i in range(counter-LOG_INTERVAL, counter):
                row = []
                row.append(tdata[i])
                [row.append(ydata[ch, i]) for ch in range(NUM_CHANNEL)]
                csv_writer.writerow(row)

    for ch in range(0, NUM_CHANNEL):
        scatter[ch].set_offsets(np.column_stack([xdata[ch, :], ydata[ch, :]/Rs*1000]))

    ax.set_xlim(0, rx_time+1)
    # ax.set_ylim(np.min(floatADC[:12])-1, np.max(floatADC[:12])+1)
    ax.figure.canvas.draw_idle()


def convertDACVoltageToDACCode(voltage: float):
    return int(voltage * (2 ** 14 - 1) / 2.5) << 2


async def main(address, char_uuid):
    global fig, ax, scatter, xdata, ydata, f_exit
    global counter, DACCode_Vg0, ADCConversionFinishFlg, DAC_Vg0
    async with BleakClient(address, timeout=10.0) as client:
        print(f"Connected: {client.is_connected}")
        # client._assign_mtu = 67
        # print("The MTU size is {}".format(client.mtu_size))

        system_id = await client.read_gatt_char(SYSTEM_ID_UUID)
        print(
            "System ID: {0}".format(
                ":".join(["{:02x}".format(x) for x in system_id[::-1]])
            )
        )

        model_number = await client.read_gatt_char(MODEL_NBR_UUID)
        print("Model Number: {0}".format("".join(map(chr, model_number))))

        try:
            device_name = await client.read_gatt_char(DEVICE_NAME_UUID)
            print("Device Name: {0}".format("".join(map(chr, device_name))))
        except Exception:
            pass

        manufacturer_name = await client.read_gatt_char(MANUFACTURER_NAME_UUID)
        print("Manufacturer Name: {0}".format("".join(map(chr, manufacturer_name))))

        firmware_revision = await client.read_gatt_char(FIRMWARE_REV_UUID)
        print("Firmware Revision: {0}".format("".join(map(chr, firmware_revision))))

#        hardware_revision = await client.read_gatt_char(HARDWARE_REV_UUID)
#        print("Hardware Revision: {0}".format("".join(map(chr, hardware_revision))))

        software_revision = await client.read_gatt_char(SOFTWARE_REV_UUID)
        print("Software Revision: {0}".format("".join(map(chr, software_revision))))

        await client.write_gatt_char(CHAR_CONTROL_POINT_UUID, b'\x01', True)
        print("Control point characteristic written")

        await client.write_gatt_char(CHAR_LED_STATE_UUID, b'\x01', True)

        await client.start_notify(char_uuid, notification_handler)
        print("ADC value 1 characteristic notification enabled")

        DAC_Vs0 = Vs
        DAC_Vg0 = Vg
        DACCode_Vs0 = convertDACVoltageToDACCode(DAC_Vs0)
        DACCode_Vg0 = convertDACVoltageToDACCode(DAC_Vg0)
        write_value = bytearray(struct.pack('HHHHHHHH', DACCode_Vs0, DACCode_Vs0, DACCode_Vs0, DACCode_Vs0, DACCode_Vg0, DACCode_Vg0, DACCode_Vg0, DACCode_Vg0))
        # value = await client.read_gatt_char(CHAR_DAC_DATA_UUID)
        # print("I/O Data Pre-Write Value: {0}".format(value))
        await client.write_gatt_char(CHAR_DAC_DATA_UUID, write_value, True)
        # value = await client.read_gatt_char(CHAR_DAC_DATA_UUID)
        # print("I/O Data Post-Write Value: {0}".format(value))

        fig, ax = plt.subplots()    # figsize=(16, 10)
        xdata = np.empty((0,))
        ydata = np.empty((0,))
        ydata[:] = np.NaN    # np.NaN
        # line, = plt.plot(xdata, ydata)
        xdata = np.tile(xdata, (4, 1))
        ydata = np.tile(ydata, (4, 1))
        for ch in range(0, NUM_CHANNEL):
            scatter_ch = ax.scatter(xdata[ch], ydata[ch], s=5)
            scatter.append(scatter_ch)
        ax.set_xlim(0, 16)
        ax.set_ylim(0.0, 0.7)
        plt.xlabel('Samples')
        plt.ylabel('Isd (mA)')
        ax_exit = plt.axes([0.77, 0.77, 0.1, 0.075])
        button_exit = Button(ax_exit, 'Exit')
        button_exit.on_clicked(onclick_button_exit)

        with open(filename, mode='a', newline='') as csv_file:
            csv_writer = csv.writer(csv_file)
            row = ['time']
            [row.append('Isd_CH{:d}'.format(ch)) for ch in range(0, NUM_CHANNEL)]
            csv_writer.writerow(row)

        await asyncio.sleep(3.0)
        while (f_exit == False):
            # if ADCConversionFinishFlg:
            #     # write_value = bytearray(struct.pack('HHHHHHHH', DACCode_Vs0, DACCode_Vs0, DACCode_Vs0, DACCode_Vs0, DACCode_Vg0, DACCode_Vg0, DACCode_Vg0, DACCode_Vg0))
            #     # print("DAC[4] = {}".format(DACCode_Vg0))
            #     # await client.write_gatt_char(CHAR_DAC_DATA_UUID, write_value, True)
            #     ADCConversionFinishFlg = False
            await asyncio.sleep(0.33)
            ax.figure.canvas.flush_events()
        await client.stop_notify(char_uuid)


if __name__ == "__main__":
    asyncio.run(
        main(
            sys.argv[1] if len(sys.argv) > 1 else ADDRESS,
            sys.argv[2] if len(sys.argv) > 2 else CHAR_ADC_VAL_1_UUID,
        )
    )
