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

NUM_DATA = 4
Rs = 100
Vgs_start = -0.5
Vgs_step = 0.25
Vgs_stop = 1.0
Vgs = Vgs_start
Vd = 0.5
f_exit = False
f_finish = False
# fig, ax = None, None
# line = None
# xdata, ydata = None, None
plt.ion()

counter = 0
prevCounter = 0
DACCode = 0xFF1F
ADCConversionFinishFlg = False
sineSignal = []
# colors = ['#1f77b4', '#ff7f0e', '#2ca02c', '#d62728', '#9467bd', '#8c564b', '#e377c2', '#7f7f7f', '#bcbd22', '#17becf']
# colors = [0.1, 0.2, 0.3, 0.4]

def generateSineWave():
    N = NUM_DATA/2
    ix = np.arange(N)
    signal = np.sin(2 * np.pi * ix / float(N / 2)) * 1 + 1.5
    time = ix / 96e3
    return signal, time

def onclick_button_exit(event):
    global f_exit
    f_exit = True

def notification_handler(sender, data):
    global fig, ax, line, xdata, ydata, cdata
    global counter, prevCounter, ADCConversionFinishFlg, DAC_Vs0, DAC_Vg0, Vgs, f_finish
    """Simple notification handler which prints the data received."""
    print("{0}: {1}".format(sender, data))

    # https://stackoverflow.com/questions/10944621/dynamically-updating-plot-in-matplotlib
    # ydata_new = int.from_bytes(data, byteorder='little', signed=False)
    floatADC = struct.unpack_from('<ffffffffffffIIII', data)
    # floatADC = list(floatADC)
    # floatADC.reverse()
    print(floatADC)

    ADCConversionFinishFlg = True
    DACTargetVoltage_Channel0 = DAC_Vs0[prevCounter]
    DACTargetVoltage_Channel4 = DAC_Vg0
    # if (prevCounter == 0): # and (Vgs == Vgs_start):
    #     # ydata = np.reshape(ydata, (4, -1))
    #     # xdata = np.reshape(xdata, (4, -1))
    #     # with open('sensor2.csv', mode='a', newline='') as csv_file:
    #     #     csv_writer = csv.writer(csv_file)
    #     #     for x0, y0, y1, y2, y3 in zip(xdata[0, :], ydata[0, :], ydata[1, :], ydata[2, :], ydata[3, :]):
    #     #         if not np.isnan(y0):
    #     #             csv_writer.writerow([x0, y0, y1, y2, y3])
    #     #         else:
    #     #             csv_writer.writerow(['Vds', 'Vgs', 'Ids'])
    #     #             break
    #     ydata = np.empty((0,))
    #     xdata = np.empty((0,))

    prevCounter = counter
    if counter == len(DAC_Vs0) - 1:
        counter = 0
        Vgs += Vgs_step
        if Vgs > Vgs_stop:
            f_finish = True
            Vgs = Vgs_start
    else:
        counter += 1

    ydata = np.append(ydata[0:], floatADC[11])
    xdata = np.append(xdata[0:], DACTargetVoltage_Channel0)

    # line.set_xdata(xdata)
    # line.set_ydata(ydata)

    # ydata = np.reshape(ydata, (4, -1))
    # ydata_new = np.asarray(floatADC[8:12]).reshape((4, 1))   #   Channel 11 - 8
    # ydata = np.concatenate((ydata, ydata_new), axis=1)
    # ydata = np.reshape(ydata, (-1,))
    #
    # xdata = np.reshape(xdata, (4, -1))
    # xdata_new = np.asarray([DACTargetVoltage_Channel0] * 4).reshape((4, 1))
    # xdata = np.concatenate((xdata, xdata_new), axis=1)
    # xdata = np.reshape(xdata, (-1,))

    # cdata = np.reshape(cdata, (4, -1))
    # cdata_new = np.asarray(colors[0:4]).reshape((4, 1))
    # cdata = np.concatenate((cdata, cdata_new), axis=1)
    # cdata = np.reshape(cdata, (-1,))

    line.set_offsets(np.column_stack([Vd - xdata, ydata/Rs*1000]))
    # line.set_array(cdata)

    # ax.set_ylim(np.min(floatADC[:12])-1, np.max(floatADC[:12])+1)
    # ax.set_ylim(np.min(sineSignal) - 1, np.max(sineSignal) + 1)
    # plt.draw()
    # plt.pause(0.04)
    line.figure.canvas.draw_idle()


def convertDACVoltageToDACCode(voltage: float):
    return int(voltage * (2 ** 14 - 1) / 2.5) << 2


async def main(address, char_uuid):
    global fig, ax, line, xdata, ydata, cdata, f_exit
    global counter, prevCounter, ADCConversionFinishFlg, DAC_Vs0, DAC_Vg0, Vgs, f_finish
    async with BleakClient(address, timeout=10.0) as client:
        print(f"Connected: {client.is_connected}")
        # client._assign_mtu = 67
        # print("The MTU size is {}".format(client.mtu_size))

        # generate Sine waveform
        sineSignal, time = generateSineWave()

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

        await client.write_gatt_char(CHAR_CONTROL_POINT_UUID, b'\x00', True)
        print("Control point characteristic written")

        await client.write_gatt_char(CHAR_LED_STATE_UUID, b'\x01', True)

        DACDefaultVoltage = 0.3
        # DAC_Vs0 = 0.9
        # DAC_Vg0 = np.arange(1.3, 0.29, -0.05)
        # Vd = 0.5
        # Vds [-0.8, 0.0]
        # Vs = Vd - Vds
        # DAC_Vs0 = np.concatenate([np.arange(1.0, 2.01, 0.05), np.arange(2.0, 0.99, -0.05)])
        Vs_start = Vd - 0.0
        Vs_stop = Vd - (-0.8)
        Vs_step = 0.02
        DAC_Vs0 = np.concatenate([np.arange(Vs_start, Vs_stop+0.001, Vs_step), np.arange(Vs_stop, Vs_start-0.001, -Vs_step)])
        DAC_Vg0 = DAC_Vs0[counter] + Vgs
        # DAC_Vg0 = np.ones(20) * 0.5
        DACCode_Vs0 = convertDACVoltageToDACCode(DAC_Vs0[counter])
        DACCode_Vg0 = convertDACVoltageToDACCode(DAC_Vg0)
        # DACCode = convertDACVoltageToDACCode(DACDefaultVoltage)
        counter += 1
        # DAC Channel 0-3: Vs, 4-7: Vg
        write_value = bytearray(struct.pack('HHHHHHHH', DACCode_Vs0, DACCode_Vs0, DACCode_Vs0, DACCode_Vs0, DACCode_Vg0, DACCode_Vg0, DACCode_Vg0, DACCode_Vg0))
        # value = await client.read_gatt_char(CHAR_DAC_DATA_UUID)
        # print("I/O Data Pre-Write Value: {0}".format(value))
        await client.write_gatt_char(CHAR_DAC_DATA_UUID, write_value, True)

        # value = await client.read_gatt_char(CHAR_DAC_DATA_UUID)
        # print("I/O Data Post-Write Value: {0}".format(value))

        fig, ax = plt.subplots()    # figsize=(16, 10)
        xdata = sineSignal[0] * np.ones(NUM_DATA)
        ydata = np.empty((NUM_DATA,))
        ydata[:] = np.NaN    # np.NaN
        # line, = plt.plot(xdata, ydata)
        # xdata = np.tile(xdata, 4)
        # ydata = np.tile(ydata, 4)
        # cdata = [colors[0]] * NUM_DATA + [colors[1]] * NUM_DATA + [colors[2]] * NUM_DATA + [colors[3]] * NUM_DATA
        line = plt.scatter(xdata, ydata, s=5)
        ax.set_xlim(-0.9, 0.1)
        ax.set_ylim(-0.2, 1.0)
        plt.xlabel('Vds sweep voltage (V)')
        plt.ylabel('Isd (mA)')
        ax_exit = plt.axes([0.77, 0.77, 0.1, 0.075])
        button_exit = Button(ax_exit, 'Exit')
        button_exit.on_clicked(onclick_button_exit)

        value = await client.read_gatt_char(CHAR_ADC_VAL_1_UUID)
        print("ADC1 Value is: {0}".format(value))
        adc_data_int = int.from_bytes(value, byteorder='little', signed=False)

        await client.start_notify(char_uuid, notification_handler)
        print("ADC value 1 characteristic notification enabled")
        await asyncio.sleep(3.0)
        while ((f_exit == False) and (f_finish == False)):
            if ADCConversionFinishFlg:
                DACCode_Vs0 = convertDACVoltageToDACCode(DAC_Vs0[counter])
                DAC_Vg0 = DAC_Vs0[counter] + Vgs
                DACCode_Vg0 = convertDACVoltageToDACCode(DAC_Vg0)
                write_value = bytearray(struct.pack('HHHHHHHH', DACCode_Vs0, DACCode_Vs0, DACCode_Vs0, DACCode_Vs0, DACCode_Vg0, DACCode_Vg0, DACCode_Vg0, DACCode_Vg0))
                print("DAC[0] = {}".format(DACCode_Vs0))
                await client.write_gatt_char(CHAR_DAC_DATA_UUID, write_value, True)
                ADCConversionFinishFlg = False

            await asyncio.sleep(0.033)
            line.figure.canvas.flush_events()
        await client.stop_notify(char_uuid)
        if (f_finish):
            while (f_exit == False):
                await asyncio.sleep(0.033)
                line.figure.canvas.flush_events()


if __name__ == "__main__":
    asyncio.run(
        main(
            sys.argv[1] if len(sys.argv) > 1 else ADDRESS,
            sys.argv[2] if len(sys.argv) > 2 else CHAR_ADC_VAL_1_UUID,
        )
    )
