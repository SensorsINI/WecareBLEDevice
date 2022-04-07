# -*- coding: utf-8 -*-
"""
Sweep Isd-Vds
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

FILENAME = "sensor_Isd-Vds_" + datetime.now().strftime("%Y%m%d_%H%M%S") + ".csv"
SAVE_CSV = True
PLOT_TYPE = 'line'      # 'line', 'scatter'

NUM_CONNECT_TRIAL = 3
NUM_CHANNEL = 4
LOG_INTERVAL = 64

Vdac_min = 0.0
Vdac_max = 2.4
Rs = 100
Vd = 0.5
# Vgs in [-0.5, 1.1]
# Vds in [-1.9, 0.5]
# DEFAULT VALUE Vgs range
Vgs_start = -0.5
Vgs_stop = 1.0
Vgs_step = 0.25
# DEFAULT VALUE Vds range
Vds_start = 0.0
Vds_stop = -0.8
Vds_step = -0.02
# DEFAULT VALUE oneway_sweep
oneway_sweep = False
# DEFAULT VALUE y_min, y_max
y_min, y_max = -0.2, 1.5
# DEFAULT VALUE x_min, x_max
x_min, x_max = -0.9, 0.1

counter = 0
ADC_NTF_cnt = 0
errCounter = 0
prev_idx = 0
f_finish = False
ADCConversionFinishFlg = False
lines = []
tdata = []

plt.ion()


def convertDACVoltageToDACCode(voltage: float):
    return int(voltage * (2 ** 14 - 1) / 2.5) << 2


def onclick_button_restart(event):
    global f_finish
    if f_finish: f_finish = False


def onclick_check_buttons(label):
    global chkbtn_labels, lines
    index = chkbtn_labels.index(label)
    lines[index].set_visible(not lines[index].get_visible())


def on_enter(event):
    global ax_button_restart
    ax_button_restart.set_visible(True)


def on_leave(event):
    global ax_button_restart
    ax_button_restart.set_visible(False)


def notification_handler(sender, data):
    global SAVE_CSV, PLOT_TYPE
    global fig, ax, lines, xdata, ydata, tdata, counter, ADCConversionFinishFlg, f_finish
    global DAC_Vs0, DAC_Vg0, Vgs, Vgs_start, Vgs_step, Vgs_stop, prev_idx
    """Simple notification handler which prints the data received."""
    # time_now = datetime.now().time()
    print("{0}: {1}".format(sender, data))

    # https://stackoverflow.com/questions/10944621/dynamically-updating-plot-in-matplotlib
    floatADC = struct.unpack_from('<ffffffffffffIIII', data)
    print(floatADC)
    ADCConversionFinishFlg = True
    
    if not f_finish:
        DACTargetVoltage_Channel0 = DAC_Vs0[counter]
        # DACTargetVoltage_Channel4 = DAC_Vg0
        # tdata.append(time_now)

        ydata_new = np.asarray(floatADC[11:7:-1]).reshape((4, 1))   #   Channel 11 - 8
        ydata = np.concatenate((ydata, ydata_new), axis=1)

        xdata_new = np.asarray([DACTargetVoltage_Channel0] * 4).reshape((4, 1))
        xdata = np.concatenate((xdata, xdata_new), axis=1)

        for ch in range(0, NUM_CHANNEL):
            xdata_ch = Vd - xdata[ch]       # Vds
            ydata_ch = ydata[ch]/Rs*1000    # Isd
            if PLOT_TYPE == 'line':
                lines[ch].set_xdata(xdata_ch)
                lines[ch].set_ydata(ydata_ch)
            else:
                lines[ch].set_offsets(np.column_stack([xdata_ch, ydata_ch]))
        # ax.set_ylim(np.min(floatADC[:12])-1, np.max(floatADC[:12])+1)

        counter += 1
        if counter == len(DAC_Vs0):
            counter = 0
            
            if SAVE_CSV:
                with open(FILENAME, mode='a', newline='') as csv_file:
                    csv_writer = csv.writer(csv_file)
                    for i in range(prev_idx, xdata.shape[1]):   # len(DAC_Vs0)
                        row = [Vgs]
                        row.append(Vd - xdata[0, i])
                        [row.append(ydata[ch, i]/Rs*1000) for ch in range(NUM_CHANNEL)]
                        csv_writer.writerow(row)
            
            line_sep = np.empty((4, 1)) # Break line into segments with NaNs
            line_sep[:] = np.NaN
            xdata = np.concatenate((xdata, line_sep), axis=1)
            ydata = np.concatenate((ydata, line_sep), axis=1)
            prev_idx = xdata.shape[1]
            
            Vgs += Vgs_step
            if Vgs > Vgs_stop:
                Vgs = Vgs_start
                f_finish = True
                print("[Info] Sweep cycle finished!")
                print("[Info] Data saved to file {:}".format(FILENAME))
                print("Press \"Restart\" button to start a new sweep cycle...")
                xdata = np.empty((4, 0))
                ydata = np.empty((4, 0))
                prev_idx = 0
    
    ax.figure.canvas.draw_idle()


async def main(address, save_csv, plot_type):
    global SAVE_CSV, PLOT_TYPE
    global fig, ax, lines, xdata, ydata, counter, ADC_NTF_cnt, errCounter, ADCConversionFinishFlg, f_finish
    global ax_button_restart, chkbtn_labels, DAC_Vs0, DAC_Vg0, Vgs
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
    
    await client.write_gatt_char(CHAR_CONTROL_POINT_UUID, b'\x00', True)
    print("Control point characteristic written (0)")

    await client.write_gatt_char(CHAR_LED_STATE_UUID, b'\x01', True)

    await client.start_notify(CHAR_ADC_VAL_1_UUID, notification_handler)
    print("ADC value 1 characteristic notification enabled")

    x_min = min(Vds_start, Vds_stop) - 0.1
    x_max = max(Vds_start, Vds_stop) + 0.1
    # Vs_start = Vd - 0.0
    # Vs_stop = Vd - (-0.8)
    # Vs_step = 0.02
    Vds = np.arange(Vds_start, Vds_stop+Vds_step, Vds_step)
    DAC_Vs0 = Vd - Vds
    if not oneway_sweep:
        DAC_Vs0 = np.concatenate([DAC_Vs0, np.flip(DAC_Vs0)])
    Vgs = Vgs_start
    # DAC_Vs0 = np.concatenate([np.arange(Vs_start, Vs_stop+0.001, Vs_step), np.arange(Vs_stop, Vs_start-0.001, -Vs_step)])
    DAC_Vg0 = DAC_Vs0[counter] + Vgs
    print(DAC_Vs0, DAC_Vg0)
    DACCode_Vs0 = convertDACVoltageToDACCode(DAC_Vs0[counter])
    DACCode_Vg0 = convertDACVoltageToDACCode(DAC_Vg0)
    write_value = bytearray(struct.pack('HHHHHHHH', DACCode_Vs0, DACCode_Vs0, DACCode_Vs0, DACCode_Vs0, DACCode_Vg0, DACCode_Vg0, DACCode_Vg0, DACCode_Vg0))
    await client.write_gatt_char(CHAR_DAC_DATA_UUID, write_value, True)

    if SAVE_CSV:
        with open(FILENAME, mode='a', newline='') as csv_file:
            csv_writer = csv.writer(csv_file)
            row = ['Vgs (V)']
            row.append('Vds (V)')
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
    ax.set_xlim(x_min, x_max)  # Vds voltage sweep range
    ax.set_ylim(y_min, y_max)
    plt.xlabel('Vds sweep voltage (V)')
    plt.ylabel('Isd (mA)')
    
    ax_button_restart = plt.axes([0.77, 0.77, 0.1, 0.075])
    button_restart = Button(ax_button_restart, 'Restart')
    button_restart.on_clicked(onclick_button_restart)
    ax_button_restart.set_visible(False)
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
        if not f_finish:
            if ADCConversionFinishFlg:
                DACCode_Vs0 = convertDACVoltageToDACCode(DAC_Vs0[counter])
                DAC_Vg0 = DAC_Vs0[counter] + Vgs
                DACCode_Vg0 = convertDACVoltageToDACCode(DAC_Vg0)
                write_value = bytearray(struct.pack('HHHHHHHH', DACCode_Vs0, DACCode_Vs0, DACCode_Vs0, DACCode_Vs0, DACCode_Vg0, DACCode_Vg0, DACCode_Vg0, DACCode_Vg0))
                # print("DAC: Vs0 = {}, Vg0 = {}".format(DACCode_Vs0, DACCode_Vg0))
                await client.write_gatt_char(CHAR_DAC_DATA_UUID, write_value, True)
                ADCConversionFinishFlg = False
                ADC_NTF_cnt = 0
            else:
                ADC_NTF_cnt += 1
                if (ADC_NTF_cnt >= 7):
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
                                await client.write_gatt_char(CHAR_CONTROL_POINT_UUID, b'\x00', True)
                                print("Control point characteristic written (0)")
                                await client.start_notify(CHAR_ADC_VAL_1_UUID, notification_handler)
                                print("ADC value 1 characteristic notification enabled")
                                await client.write_gatt_char(CHAR_DAC_DATA_UUID, write_value, True)
                                break

        await asyncio.sleep(0.11)
        ax.figure.canvas.flush_events()

    await client.stop_notify(CHAR_ADC_VAL_1_UUID)
    print("ADC value 1 characteristic notification disabled")
    if errCounter:
        print("[WARNING!] Total ADC notification missed: {:d}".format(errCounter))
    await client.disconnect()
    print("BLE device ({:}) disconnected.".format(address))


if __name__ == "__main__":
    
    # Default parameters
    params = [Vds_start, Vds_stop, Vds_step, Vgs_start, Vgs_stop, Vgs_step, oneway_sweep, y_min, y_max]
    params_name = ['Vds_start', 'Vds_stop', 'Vds_step', 'Vgs_start', 'Vgs_stop', 'Vgs_step', 'oneway_sweep', 'axis_y_min', 'axis_y_max']
    
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
        
        # Input Vds range
        Vds_min = Vd - Vdac_max
        Vds_max = Vd - Vdac_min
        
        input_Vds_start = input("Vds_start (range {} .. {}, default to {}) = ".format(Vds_min, Vds_max, Vds_start))
        if input_Vds_start == "":
            print("Use default value {}".format(Vds_start))
        else:
            input_Vds_start = float(input_Vds_start)
            while not (Vds_min <= input_Vds_start <= Vds_max):
                print("Out of range! Please specify again..")
                input_Vds_start = float(input("Vds_start (range {} .. {}, default to {}) = ".format(Vds_min, Vds_max, Vds_start)))
            Vds_start = input_Vds_start
        
        input_Vds_stop = input("Vds_stop (range {} .. {}, default to {}) = ".format(Vds_min, Vds_max, Vds_stop))
        if input_Vds_stop == "":
            print("Use default value {}".format(Vds_stop))
        else:
            input_Vds_stop = float(input_Vds_stop)
            while not (Vds_min <= input_Vds_stop <= Vds_max):
                print("Out of range! Please specify again..")
                input_Vds_stop = float(input("Vds_stop (range {} .. {}, default to {}) = ".format(Vds_min, Vds_max, Vds_stop)))
            Vds_stop = input_Vds_stop
        
        input_Vds_step = input("Vds_step (default to {}) = ".format(Vds_step))
        if input_Vds_step == "":
            print("Use default value {}".format(Vds_step))
        else:
            Vds_step = float(input_Vds_step)
        Vds_step = abs(Vds_step) if Vds_start < Vds_stop else -abs(Vds_step)
        
        # Input Vgs range
        Vs_min = Vd - max(Vds_start, Vds_stop)
        Vs_max = Vd - min(Vds_start, Vds_stop)
        Vgs_min = Vdac_min - Vs_min
        Vgs_max = Vdac_max - Vs_max
        
        input_Vgs_start = input("Vgs_start (range {} .. {}, default to {}) = ".format(Vgs_min, Vgs_max, Vgs_start))
        if input_Vgs_start == "":
            print("Use default value {}".format(Vgs_start))
        else:
            input_Vgs_start = float(input_Vgs_start)
            while not (Vgs_min <= input_Vgs_start <= Vgs_max):
                print("Out of range! Please specify again..")
                input_Vgs_start = float(input("Vgs_start (range {} .. {}, default to {}) = ".format(Vgs_min, Vgs_max, Vgs_start)))
            Vgs_start = input_Vgs_start
        
        input_Vgs_stop = input("Vgs_stop (range {} .. {}, default to {}) = ".format(Vgs_min, Vgs_max, Vgs_stop))
        if input_Vgs_stop == "":
            print("Use default value {}".format(Vgs_stop))
        else:
            input_Vgs_stop = float(input_Vgs_stop)
            while not (Vgs_min <= input_Vgs_stop <= Vgs_max):
                print("Out of range! Please specify again..")
                input_Vgs_stop = float(input("Vgs_stop (range {} .. {}, default to {}) = ".format(Vgs_min, Vgs_max, Vgs_stop)))
            Vgs_stop = input_Vgs_stop
        
        input_Vgs_step = input("Vgs_step (default to {}) = ".format(Vgs_step))
        if input_Vgs_step == "":
            print("Use default value {}".format(Vgs_step))
        else:
            Vgs_step = float(input_Vgs_step)
        Vgs_step = abs(Vgs_step) if Vgs_start < Vgs_stop else -abs(Vgs_step)
        
        # Input oneway_sweep
        input_oneway_sweep = input("oneway_sweep ('Y'=unidirectional / 'N'=bidirectional, default to {}) ? ".format('Y' if oneway_sweep else 'N'))
        while input_oneway_sweep not in ["", "Y", "y", "N", "n"]:
            print("Please type \'Y\' or \'N\'.")
            input_oneway_sweep = input("oneway_sweep ('Y'=unidirectional / 'N'=bidirectional, default to {}) ? ".format('Y' if oneway_sweep else 'N'))
        if input_oneway_sweep == "":
            print("Use default value {}".format('Y' if oneway_sweep else 'N'))
        else:
            oneway_sweep = True if input_oneway_sweep in ["Y", "y"] else False
        
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
