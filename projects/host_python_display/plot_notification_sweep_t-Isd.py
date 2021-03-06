# -*- coding: utf-8 -*-
"""
Sweep Isd-t
-------------

"""

import sys
import asyncio
import platform

from bleak import BleakClient
from bleak.uuids import uuid16_dict

import numpy as np
import matplotlib.pyplot as plt
from matplotlib.widgets import Button, CheckButtons
# from matplotlib import animation

import struct
import csv
from datetime import datetime


uuid16_dict = {v: k for k, v in uuid16_dict.items()}

# DISS Service
SYSTEM_ID_UUID = "0000{0:x}-0000-1000-8000-00805f9b34fb".format(uuid16_dict.get("System ID"))
DISS_CHAR_NAMES = ["Model Number String", "Device Name", "Firmware Revision String",
                   # "Hardware Revision String",
                   "Software Revision String", "Manufacturer Name String"]

# you can change these to match your device or override them from the command line
ADDRESS = "CE:C6:E3:FA:1D:59"
CHAR_CONTROL_POINT_UUID = "2d86686a-53dc-25b3-0c4a-f0e10c8dee20"
CHAR_ADC_VAL_1_UUID = "15005991-b131-3396-014c-664c9867b917"
CHAR_LED_STATE_UUID = "5a87b4ef-3bfa-76a8-e642-92933c31434f"
CHAR_DAC_DATA_UUID = "772ae377-b3d2-4f8e-4042-5481d1e0098c"

FILENAME = "sensor_Isd-t_" + datetime.now().strftime("%Y%m%d_%H%M%S") + ".csv"
SAVE_CSV = True
PLOT_TYPE = 'line'      # 'line', 'scatter'

NUM_CONNECT_TRIAL = 3
NUM_CHANNEL = 4
LOG_INTERVAL = 64

Vdac_min = 0.0
Vdac_max = 2.4
Rs = 100
Vd = 0.5
# DEFAULT VALUE Vds
Vds = -0.4
# DEFAULT VALUE Vgs
Vgs = 0.1
Vs = Vd - Vds
Vg = Vgs + Vs
# DEFAULT VALUE y_min, y_max
y_min, y_max = -0.1, 1.5

counter = 0
ADC_NTF_cnt = 0
errCounter = 0
f_pause = False
ADCConversionFinishFlg = False
lines = []
tdata = []

plt.ion()


def convertDACVoltageToDACCode(voltage: float):
    return int(voltage * (2 ** 14 - 1) / 2.5) << 2


def onclick_button_pause(event):
    global button_pause, f_pause
    if f_pause:
        f_pause = False
        button_pause.label.set_text('Pause')
    else:
        f_pause = True
        button_pause.label.set_text('Continue')


def onclick_check_buttons(label):
    global chkbtn_labels, lines
    index = chkbtn_labels.index(label)
    lines[index].set_visible(not lines[index].get_visible())


def on_enter(event):
    global ax_button_pause
    ax_button_pause.set_visible(True)


def on_leave(event):
    global ax_button_pause
    ax_button_pause.set_visible(False)


def notification_handler(sender, data):
    global SAVE_CSV, PLOT_TYPE
    global fig, ax, lines, xdata, ydata, tdata, counter, ADCConversionFinishFlg, f_pause
    """Simple notification handler which prints the data received."""
    time_now = datetime.now().time()
    print("{0}: {1}".format(sender, data))

    # https://stackoverflow.com/questions/10944621/dynamically-updating-plot-in-matplotlib
    floatADC = struct.unpack_from('<ffffffffffffIIII', data)
    print(floatADC)
    ADCConversionFinishFlg = True
    
    if not f_pause:
        rx_time = counter
        tdata.append(time_now)

        ydata_new = np.asarray(floatADC[11:7:-1]).reshape((4, 1))   #   Channel 11 - 8
        ydata = np.concatenate((ydata, ydata_new), axis=1)

        xdata_new = np.asarray([rx_time] * 4).reshape((4, 1))
        xdata = np.concatenate((xdata, xdata_new), axis=1)

        for ch in range(0, NUM_CHANNEL):
            xdata_ch = xdata[ch]
            ydata_ch = ydata[ch]/Rs*1000    # Isd
            if PLOT_TYPE == 'line':
                lines[ch].set_xdata(xdata_ch)
                lines[ch].set_ydata(ydata_ch)
            else:
                lines[ch].set_offsets(np.column_stack([xdata_ch, ydata_ch]))
        ax.set_xlim(0, rx_time+1)
        # ax.set_ylim(np.min(floatADC[:12])-1, np.max(floatADC[:12])+1)

        counter += 1

        if SAVE_CSV:
            if (counter % LOG_INTERVAL) == LOG_INTERVAL-1:
                with open(FILENAME, mode='a', newline='') as csv_file:
                    csv_writer = csv.writer(csv_file)
                    for i in range(counter-LOG_INTERVAL, counter):
                        row = []
                        row.append(tdata[i])
                        [row.append(ydata[ch, i]/Rs*1000) for ch in range(NUM_CHANNEL)]
                        csv_writer.writerow(row)
    
    ax.figure.canvas.draw_idle()


async def main(address, save_csv, plot_type):
    global SAVE_CSV, PLOT_TYPE
    global fig, ax, lines, xdata, ydata, f_exit, counter, ADC_NTF_cnt, errCounter, ADCConversionFinishFlg, f_pause
    global ax_button_pause, button_pause, chkbtn_labels
    SAVE_CSV, PLOT_TYPE = save_csv, plot_type
    
    print("Connecting to BLE device ({})...".format(address))
    client = BleakClient(address, timeout=10.0)
    for connect_cnt in range(1, NUM_CONNECT_TRIAL+1):
        try:
            await client.connect()
        except Exception as exception:
            print("[WARNING!] BLE CONNECT TRIAL #{:d}: {}".format(connect_cnt, exception))
            if connect_cnt >= NUM_CONNECT_TRIAL:
                print("[ERROR!] BLE connection failed!")
                return
            await asyncio.sleep(3.0)
        else:
            if client.is_connected: break
    
    print(f"Connected: {client.is_connected}")
    # client._assign_mtu = 67
    # print("The MTU size is {}".format(client.mtu_size))

    system_id = await client.read_gatt_char(SYSTEM_ID_UUID)
    print("System ID: {0}".format(":".join(["{:02x}".format(x) for x in system_id[::-1]])))

    for char_name in DISS_CHAR_NAMES:
        char_uuid = "0000{0:x}-0000-1000-8000-00805f9b34fb".format(uuid16_dict.get(char_name))
        char_value = await client.read_gatt_char(char_uuid)
        print("{0}: {1}".format(char_name, "".join(map(chr, char_value))))
    
    await client.write_gatt_char(CHAR_CONTROL_POINT_UUID, b'\x01', True)
    print("Control point characteristic written (1)")

    await client.write_gatt_char(CHAR_LED_STATE_UUID, b'\x01', True)

    await client.start_notify(CHAR_ADC_VAL_1_UUID, notification_handler)
    print("ADC value 1 characteristic notification enabled")

    DAC_Vs0 = Vs
    DAC_Vg0 = Vg
    DACCode_Vs0 = convertDACVoltageToDACCode(DAC_Vs0)
    DACCode_Vg0 = convertDACVoltageToDACCode(DAC_Vg0)
    write_value = bytearray(struct.pack('HHHHHHHH', DACCode_Vs0, DACCode_Vs0, DACCode_Vs0, DACCode_Vs0, DACCode_Vg0, DACCode_Vg0, DACCode_Vg0, DACCode_Vg0))
    await client.write_gatt_char(CHAR_DAC_DATA_UUID, write_value, True)

    if SAVE_CSV:
        with open(FILENAME, mode='a', newline='') as csv_file:
            csv_writer = csv.writer(csv_file)
            row = ['time']
            [row.append('Isd_CH{:d} (mA)'.format(ch)) for ch in range(0, NUM_CHANNEL)]
            csv_writer.writerow(row)

    fig, ax = plt.subplots()    # figsize=(16, 10)
    xdata = np.empty((4, 0))
    ydata = np.empty((4, 0))
    for ch in range(0, NUM_CHANNEL):
        if PLOT_TYPE == 'line':
            line_ch = ax.plot(xdata[ch], ydata[ch], marker='.')[0]
        else:
            line_ch = ax.scatter(xdata[ch], ydata[ch], s=5)
        lines.append(line_ch)
    # DEFAULT VALUE x-axis range
    ax.set_xlim(0, 16)
    # DEFAULT VALUE y-axis range
    ax.set_ylim(y_min, y_max)
    plt.xlabel('Samples')
    plt.ylabel('Isd (mA)')
    ax_button_pause = plt.axes([0.77, 0.77, 0.1, 0.075])
    button_pause = Button(ax_button_pause, 'Pause')
    button_pause.on_clicked(onclick_button_pause)
    ax_button_pause.set_visible(False)
    plt.connect("axes_enter_event", on_enter)
    plt.connect("axes_leave_event", on_leave)

    rax = plt.axes([0.9, 0.4, 0.1, 0.2])
    chkbtn_labels = ['CH0', 'CH1', 'CH2', 'CH3']
    actives = [True] * 4
    chk_btns = CheckButtons(rax, chkbtn_labels, actives)
    colors = ['C0', 'C1', 'C2', 'C3']
    for rect, line, label, color in zip(chk_btns.rectangles, chk_btns.lines, chk_btns.labels, colors):
        rect.set_edgecolor(color)
        [sub_line.set_color(color) for sub_line in line]
        label.set_color(color)
    chk_btns.on_clicked(onclick_check_buttons)

    # await asyncio.sleep(3.0)
    while (plt.fignum_exists(fig.number)):
        if not f_pause:
            if ADCConversionFinishFlg:
                # write_value = bytearray(struct.pack('HHHHHHHH', DACCode_Vs0, DACCode_Vs0, DACCode_Vs0, DACCode_Vs0, DACCode_Vg0, DACCode_Vg0, DACCode_Vg0, DACCode_Vg0))
                # print("DAC[4] = {}".format(DACCode_Vg0))
                # await client.write_gatt_char(CHAR_DAC_DATA_UUID, write_value, True)
                ADCConversionFinishFlg = False
                ADC_NTF_cnt = 0
            else:
                ADC_NTF_cnt += 1
                if (ADC_NTF_cnt >= 9):
                    errCounter += 1
                    print("[WARNING!] ADC Notification Missed ({:d}) !".format(errCounter))
                    ADCConversionFinishFlg = True
                    ADC_NTF_cnt = 0
                    while (not client.is_connected):
                        print("Trying to reconnect ...")
                        try:
                            await client.connect()
                        except Exception as exception:
                            print("[WARNING!] BLE CONNECT TRIAL #{:d}: {}".format(connect_cnt, exception))
                            await asyncio.sleep(3.0)
                        else:
                            if client.is_connected:
                                await client.write_gatt_char(CHAR_CONTROL_POINT_UUID, b'\x01', True)
                                print("Control point characteristic written (1)")
                                await client.start_notify(CHAR_ADC_VAL_1_UUID, notification_handler)
                                print("ADC value 1 characteristic notification enabled")
                                await client.write_gatt_char(CHAR_DAC_DATA_UUID, write_value, True)
                                break
        
        await asyncio.sleep(0.11)
        ax.figure.canvas.flush_events()
    
    await client.stop_notify(CHAR_ADC_VAL_1_UUID)
    print("ADC value 1 characteristic notification disabled")
    await client.write_gatt_char(CHAR_CONTROL_POINT_UUID, b'\x00', True)
    print("Control point characteristic written (0)")
    if errCounter:
        print("[WARNING!] Total ADC notification missed: {:d}".format(errCounter))
    await client.disconnect()
    print("BLE device ({:}) disconnected.".format(address))


if __name__ == "__main__":
    
    # Default parameters
    params = [Vds, Vgs, y_min, y_max]
    params_name = ['Vds', 'Vgs', 'axis_y_min', 'axis_y_max']
    
    print("Default parameters:")
    for param, param_name in zip(params, params_name):
        print("{} = {}".format(param_name, param))
    
    input_str = input("Use default parameters? [Y/n] ")
    while input_str not in ["", "Y", "y", "N", "n"]:
        print("Please type \'Y\' or \'N\'.")
        input_str = input("Use default parameters? [Y/n] ")
    if input_str in ["", "Y", "y"]:
        pass
    else:
        print("Setting custom parameters...")
        
        # Input Vds
        Vds_min = Vd - Vdac_max
        Vds_max = Vd - Vdac_min
        input_Vds = input("Vds (range {} .. {}, default to {}) = ".format(Vds_min, Vds_max, Vds))
        if input_Vds == "":
            print("Use default value {}".format(Vds))
        else:
            input_Vds = float(input_Vds)
            while not (Vds_min <= input_Vds <= Vds_max):
                print("Out of range! Please specify again..")
                input_Vds = float(input("Vds (range {} .. {}, default to {}) = ".format(Vds_min, Vds_max, Vds)))
            Vds = input_Vds
        
        # Input Vgs
        Vs = Vd - Vds
        Vgs_min = Vdac_min - Vs
        Vgs_max = Vdac_max - Vs
        input_Vgs = input("Vgs (range {} .. {}, default to {}) = ".format(Vgs_min, Vgs_max, Vgs))
        if input_Vgs == "":
            print("Use default value {}".format(Vgs))
        else:
            input_Vgs = float(input_Vgs)
            while not (Vgs_min <= input_Vgs <= Vgs_max):
                print("Out of range! Please specify again..")
                input_Vgs = float(input("Vgs (range {} .. {}, default to {}) = ".format(Vgs_min, Vgs_max, Vgs)))
            Vgs = input_Vgs
        
        # Input axis_y_min
        input_y_min = input("axis_y_min (default to {}) = ".format(y_min))
        if input_y_min == "":
            print("Use default value {}".format(y_min))
        else:
            y_min = float(input_y_min)

        # Input axis_y_max
        input_y_max = input("axis_y_max (default to {}) = ".format(y_max))
        if input_y_max == "":
            print("Use default value {}".format(y_max))
        else:
            y_max = float(input_y_max)
    
    # Run main function
    asyncio.run(
        main(
            sys.argv[1] if len(sys.argv) > 1 else ADDRESS,
            sys.argv[2] if len(sys.argv) > 2 else SAVE_CSV,
            sys.argv[3] if len(sys.argv) > 3 else PLOT_TYPE
        )
    )
