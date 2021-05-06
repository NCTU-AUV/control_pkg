"""
Rabbini API
"""
import os
import sys
import csv
import time
import pickle
import logging
import platform
import threading

import hid
import pygatt
import matplotlib.pyplot as plt

from binascii import hexlify
from typing import List
from dataclasses import dataclass
from matplotlib.lines import Line2D

from .custom_logger import CustomFormatter
from .constants import ACC_FSR_CHAR, GYRO_FSR_CHAR, DATA_RATE
from .utils import get_key_by_val

logger = logging.getLogger(__file__)
logger.setLevel(logging.DEBUG)

# create console handler with a higher log level
ch = logging.StreamHandler(sys.stdout)
ch.setLevel(logging.INFO)
ch.setFormatter(CustomFormatter())
fh = logging.FileHandler('./debug.log', encoding='utf-8')
fh.setLevel(level=logging.DEBUG)
fh.setFormatter(logging.Formatter('%(asctime)s - %(name)s - %(levelname)s - %(message)s'))

logger.addHandler(ch)
logger.addHandler(fh)

# logging.basicConfig()
# logging.getLogger('pygatt').setLevel(logging.DEBUG)

if platform.system() == 'Windows':
    BGAPIBackend = pygatt.BGAPIBackend
elif platform.system() == 'Linux':
    BGAPIBackend = pygatt.GATTToolBackend

curdir_path = os.path.dirname(os.path.realpath(__file__))


def convert_acc(acc, acc_scale):
    x = int(acc, 16)
    x = twos_comp(x, 16)
    x = float(x)
    # print('convert_acc', x, acc_scale)
    return x*(acc_scale)/32768  # x*16/32768


def convert_gyr(gyr, gyr_scale):
    x = int(gyr, 16)
    x = twos_comp(x, 16)
    x = float(x)
    # print('convert_gyr', x, gyr_scale)
    return x*gyr_scale/32768


def twos_comp(val, bits):
    """compute the 2's complement of int value val"""
    if (val & (1 << (bits - 1))) != 0:  # if sign bit is set e.g., 8bit: 128-255
        val = val - (1 << bits)        # compute negative value
    return val


@dataclass
class RabboniCmd:
    length: int = 0
    command: int = 0
    data: int = 0

    def __repr__(self):
        return f'<RabboniCmd length: {self.length}, command: {self.command}, data: {self.data}>'

    @property
    def payload(self):
        cmd_init = [0x00 for i in range(33)]
        cmd_init[1] = self.length
        cmd_init[2] = self.command
        data_len = len(self.data)
        cmd_init[3:3+data_len] = self.data
        return cmd_init

    def to_bytes(self):
        return bytes(self.payload)

    def from_bytes(self, bytes_string):
        # print('from_bytes', bytes_string)
        bytes_list = list(bytes_string)
        self.length = bytes_list[0]
        self.command = bytes_list[1]
        self.data = bytes_list[2:2+self.length-1]


@dataclass
class RabboniCmdResponse:
    length: int = 0
    command: int = 0
    data: int = 0

    def __repr__(self):
        return f'<RabboniCmdResponse length: {self.length}, command: {self.command}, data: {self.data}>'

    @classmethod
    def from_bytes(cls, bytes_string):
        # print('from_bytes', bytes_string)
        bytes_list = list(bytes_string)
        resp_length = bytes_list[0]
        bytes_hex_list = ["{:02x}".format(bl) for bl in bytes_list]
        resp = cls()
        resp.length = bytes_hex_list[0]
        # resp.command = bytes_hex_list[1]
        resp.data = bytes_hex_list[1:1+resp_length]
        return resp

    @property
    def payload(self):
        return [self.command] + self.data


class RabboniCmdHelper(object):

    def __init__(self, cmd=RabboniCmd()):
        self._cmd = cmd

    def _setup_cmd(self, length: int, command: int, data: List[int]):
        self._cmd.length = length
        self._cmd.command = command
        self._cmd.data = data

    @property
    def connect_30_cmd(self):
        self._setup_cmd(0x02, 0x30, [0x0A])
        return self._cmd

    @property
    def read_config_cmd(self):
        self._setup_cmd(0x01, 0x49, [])
        return self._cmd

    @property
    def set_config_cmd(self):
        return self._cmd

    @set_config_cmd.setter
    def set_config_cmd(self, data: List[int]):
        self._setup_cmd(0x0E, 0x45, data)

    @property
    def fetch_status_48_cmd(self):
        self._setup_cmd(0x02, 0x48, [0x0A])
        return self._cmd

    @property
    def fetch_status_32_cmd(self):
        self._setup_cmd(0x02, 0x32, [0x0A])
        return self._cmd

    @property
    def reset_current_count_cmd(self):
        self._setup_cmd(0x02, 0x38, [0x0A])
        return self._cmd

    @property
    def reset_stored_count_cmd(self):
        self._setup_cmd(0x02, 0x36, [0x0A])
        return self._cmd

    @property
    def stop_cmd(self):
        self._setup_cmd(0x02, 0x33, [0x0A])
        return self._cmd

    def format_response(self, resp):
        rabboni_resp = RabboniCmdResponse.from_bytes(resp)
        # print('format_response', rabboni_resp)
        return rabboni_resp


class Rabboni(object):

    def __init__(self, mode='USB'):
        self.mode = mode
        self.dev = None
        self.ble_adapter = None
        self.ble_devices = []
        self.acc_scale = 2
        self.gyr_scale = 250
        self.accx_list = []
        self.accy_list = []
        self.accz_list = []
        self.gyrx_list = []
        self.gyry_list = []
        self.gyrz_list = []
        self._cmd_helper = RabboniCmdHelper()
        self._read_temp = []
        self._polling_thread = threading.Thread(target=self.polling_status)
        muls_file = f'{curdir_path}/muls.pkl'
        with open(muls_file, 'rb') as f:
            self.muls = pickle.load(f)

    def scan(self, timeout=5):
        """ 掃描附近 Rabboni 裝置(Only BLE mode) """
        if self.mode == 'BLE':
            self.ble_adapter = pygatt.BGAPIBackend()
            self.ble_adapter.start()
            self.ble_devices = self.ble_adapter.scan(timeout)
            print("Scan BLE devices:")
            for dev in (self.ble_devices):
                print("Name : %s  MAC : %s" % (dev["name"], dev["address"]))
            print('=============================')
            return self.ble_devices
        else:
            # TODO:
            pass

    def connect(self, mac_address: str = None):
        """ 建立 Rabboni 裝置連線 """
        if self.mode == 'USB':
            self._connect_with_USB()
        elif self.mode == 'BLE':
            if mac_address in self.muls:
                try:
                    self.dev = self.ble_adapter.connect(mac_address, address_type=pygatt.BLEAddressType.random)
                    logger.info(f'Open BLE device {self.dev}')
                    self._discover_char_with_BLE()
                except pygatt.exceptions.NotConnectedError:
                    raise AssertionError(f'找不到 MAC 地址: {mac_address}，無法連線...')
            else:
                raise ValueError(f'{mac_address} 為不支援的 MAC 地址')
        else:
            raise ValueError('mode 僅支援 "USB" 或 "BLE"')

    def _connect_with_USB(self):
        self.dev = hid.Device(0x04d9, 0xb564)
        self.dev.send_feature_report(bytes([0, 1, 0, 194, 1, 0, 0, 0, 8]))
        self.dev.write(self._cmd_helper.connect_30_cmd.to_bytes())
        logger.info(f'Open HID device {self.dev.product}')

    def _discover_char_with_BLE(self):
        charcs = []
        for uuid in self.dev.discover_characteristics().keys():
            try:
                self.dev.char_read(uuid)
                charcs.append(
                    {'uuid': uuid, 'handle': self.dev.get_handle(uuid), 'readable': True})
            except Exception as e:
                if "unable to read" in str(e).lower():
                    charcs.append(
                        {'uuid': uuid, 'handle': self.dev.get_handle(uuid), 'readable': False})
                else:
                    raise e

        logger.debug("====== device.discover_characteristics() =====")
        for charc in charcs:
            logger.debug(charc)

    def disconnect(self):
        """ 斷開 Rabboni 裝置連線 """
        if self.mode == 'USB':
            self.dev.write(self._cmd_helper.stop_cmd.to_bytes())
            logger.info(f'Closing HID device {self.dev.product}...')
            self.dev.close()
        elif self.mode == 'BLE':
            self.dev.char_write("0000fff6-0000-1000-8000-00805f9b34fb",
                                bytearray([self._cmd_helper.stop_cmd.command]),
                                wait_for_response=True)
            logger.info(f'Disconnecting BLE device {self.dev.product}...')
            self.ble_adapter.stop()

    def read_sensor_config(self):
        """ 讀取裝置設定值 """
        if self.mode == 'USB':
            logger.debug("================== read_sensor_config start ===========================")
            self.dev.write(self._cmd_helper.read_config_cmd.to_bytes())
            # print('Send:', self._cmd_helper.read_config_cmd)

            time.sleep(3)

            read_data = self.dev.read(16)
            resp = RabboniCmdResponse.from_bytes(read_data)
            logger.debug(f'read_sensor_config {hexlify(read_data)}, resp: {resp}')
            self.acc_scale = get_key_by_val(ACC_FSR_CHAR, int(resp.data[1], 16))
            read_data = self.dev.read(16)
            resp = RabboniCmdResponse.from_bytes(read_data)
            logger.debug(f'read_sensor_config {hexlify(read_data)}, resp: {resp}')
            self.gyr_scale = get_key_by_val(GYRO_FSR_CHAR, int(resp.data[0], 16))
            logger.debug(f'Acc scale: {self.acc_scale}, Gyr scale: {self.gyr_scale}')

            ##################

            read_data = self.dev.read(16)
            resp = RabboniCmdResponse.from_bytes(read_data)
            logger.debug(f'read_sensor_config {hexlify(read_data)}, resp: {resp}')

            time.sleep(3)
            logger.debug("=================== read_sensor_config end ==========================")
        elif self.mode == 'BLE':
            self.dev.subscribe("0000fff7-0000-1000-8000-00805f9b34fb", self._read_sensor_config_callback_BLE)
            self.dev.char_write("0000fff6-0000-1000-8000-00805f9b34fb",
                                bytearray([self._cmd_helper.read_config_cmd.command]),
                                wait_for_response=True)
            # 需要這個才能讀到值
            read_data = self.dev.char_read('00001705-0000-1000-8000-00805f9b34fb')
            # print(read_data)
        # print(self._cmd_helper.read_config_cmd.payload)

    def set_sensor_config(self, acc_scale, gyr_scale, rate, threshold):
        """ 修改裝置設定值 """
        config_data = [0x00 for i in range(14)]
        config_data[0:2] = [ACC_FSR_CHAR[acc_scale], GYRO_FSR_CHAR[gyr_scale]]
        config_data[3:5] = [1, 1]
        config_data[8] = DATA_RATE[rate]
        config_data[12:14] = [threshold // 256, threshold % 256]  # 2500 => [9, 196]
        # print(config_data)
        self._cmd_helper.set_config_cmd = config_data
        # print(self._cmd_helper.set_config_cmd.payload)

        if self.mode == 'USB':
            logger.debug("================== set_sensor_config start ===========================")
            self.acc_scale = acc_scale
            self.gyr_scale = gyr_scale
            self.dev.write(self._cmd_helper.set_config_cmd.to_bytes())
            # print('Send:', self._cmd_helper.set_config_cmd)
            # set_cmd = RabboniCmd(length=0x0E, command=0x45, data=[0, 0, 0, 0, 1, 1, 0, 0, 1, 0, 0, 0, 0, 100])
            # self.dev.write(set_cmd.to_bytes())
            read_data = self.dev.read(16)
            resp = RabboniCmdResponse.from_bytes(read_data)
            logger.debug(f'set_sensor_config {hexlify(read_data)}, resp: {resp}')

            time.sleep(5)
            cmd = RabboniCmd(length=0x02, command=0x49, data=[0x0A])
            self.dev.write(cmd.to_bytes())
            read_data = self.dev.read(16)
            resp = RabboniCmdResponse.from_bytes(read_data)
            logger.debug(f'set_sensor_config {hexlify(read_data)}, resp: {resp}')
            logger.debug("================== set_sensor_config end ===========================")
        elif self.mode == 'BLE':
            # set_cmd = RabboniCmd(length=0x0E, command=0x45, data=[1, 2, 0, 0, 1, 1, 0, 0, 1, 0, 0, 0, 0, 100])
            # self.dev.char_write('0000fff6-0000-1000-8000-00805f9b34fb', bytearray([set_cmd.command] + set_cmd.data))
            self.dev.subscribe("0000fff7-0000-1000-8000-00805f9b34fb", self._set_sensor_config_callback_BLE)
            self.dev.char_write('0000fff6-0000-1000-8000-00805f9b34fb',
                                bytearray([self._cmd_helper.set_config_cmd.command] + self._cmd_helper.set_config_cmd.data))
            read_data = self.dev.char_read('00001705-0000-1000-8000-00805f9b34fb')
            # print(read_data)

    def reset_count(self, count_type=None):
        """ 重置紀錄數 """
        if self.mode == 'USB':
            if not count_type:
                self.dev.write(self._cmd_helper.reset_current_count_cmd.to_bytes())
                self.dev.write(self._cmd_helper.reset_stored_count_cmd.to_bytes())
            else:
                if count_type == 'current':
                    self.dev.write(self._cmd_helper.reset_current_count_cmd.to_bytes())
                elif count_type == 'stored':
                    self.dev.write(self._cmd_helper.reset_stored_count_cmd.to_bytes())
        elif self.mode == 'BLE':
            if not count_type:
                self.dev.char_write('0000fff6-0000-1000-8000-00805f9b34fb',
                                    bytearray([self._cmd_helper.reset_current_count_cmd.command]))
                self.dev.char_write('0000fff6-0000-1000-8000-00805f9b34fb',
                                    bytearray([self._cmd_helper.reset_stored_count_cmd.command]))
            else:
                if count_type == 'current':
                    self.dev.char_write('0000fff6-0000-1000-8000-00805f9b34fb',
                                        bytearray([self._cmd_helper.reset_current_count_cmd.command]))
                elif count_type == 'stored':
                    self.dev.char_write('0000fff6-0000-1000-8000-00805f9b34fb',
                                        bytearray([self._cmd_helper.reset_stored_count_cmd.command]))

    def start_fetching_status(self, custom_callback=None):
        """ 開始讀取 Rabboni 狀態資料 """
        self.custom_callback = custom_callback
        if self.mode == 'USB':
            self.dev.write(self._cmd_helper.fetch_status_32_cmd.to_bytes())
            self.dev.write(self._cmd_helper.fetch_status_48_cmd.to_bytes())
        elif self.mode == 'BLE':
            try:
                self.dev.subscribe("00001601-0000-1000-8000-00805f9b34fb", self._read_callback_BLE)
            except pygatt.backends.bgapi.exceptions.ExpectedResponseTimeout:
                pass

    def stop_fetching_status(self):
        """ 停止 Rabboni 狀態資料 """
        self.dev.write(self._cmd_helper.stop_cmd.to_bytes())

    def polling_status(self):
        """ 持續讀取 Rabboni 狀態資料 """
        print('============ polling_status ===============')
        if self.mode == 'USB':
            try:
                while True:
                    read_data = self.dev.read(33)
                    # time.sleep(0.5)
                    self._read_callback_USB(read_data)
            except KeyboardInterrupt:
                raise AssertionError()
        elif self.mode == 'BLE':
            try:
                while True:
                    read_data = self.dev.char_read('00001705-0000-1000-8000-00805f9b34fb')
                    # self._cmd_helper.format_response(read_data)
                    time.sleep(0.5)
            except KeyboardInterrupt:
                raise AssertionError()

    def export_csv(self, data: List = None, filename: str = 'rabboni-data'):
        """ 匯出 CSV 檔案，預設會匯出 Acc 和 Gyr 值的 CSV (rabboni-data-acc.csv, rabboni-data-gyr.csv) """
        if data is not None:
            filename = filename + '.csv'
            with open(filename, 'w', newline='') as csvfile:
                writer = csv.writer(csvfile)
                for d in data:
                    writer.writerow(d)
        else:  # 預設匯出 Acc 和 Gyr 值
            acc_filename = filename + '-acc.csv'
            with open(acc_filename, 'w', newline='') as csvfile:
                writer = csv.writer(csvfile)
                writer.writerow(['accx', 'accy', 'accz'])
                for index, accx in enumerate(self.accx_list):
                    # print(f'export csv write {accx}')
                    writer.writerow([accx, self.accy_list[index], self.accz_list[index]])

            gyr_filename = filename + '-gyr.csv'
            with open(gyr_filename, 'w', newline='') as csvfile:
                writer = csv.writer(csvfile)
                writer.writerow(['gyrx', 'gyry', 'gyrz'])
                for index, gyrx in enumerate(self.gyrx_list):
                    # print(f'export csv write {gyrx}')
                    writer.writerow([gyrx, self.gyry_list[index], self.gyrz_list[index]])

    def export_plot(self, data: List = None, filename: str = 'rabboni-data'):
        """ 匯出狀態折線圖，預設會匯出 Acc 和 Gyr 值的折線圖 (rabboni-data-acc.png, rabboni-data-gyr.png) """
        if data is not None:
            plt.plot(data)
            plt.savefig(filename)
            plt.show()
        else:
            acc_fig, acc_plt = plt.subplots()
            acc_x = list(range(len(self.accx_list)))
            acc_lines = [
                Line2D([0], [0], color="red", label="AccX"),
                Line2D([0], [0], color="green", label="AccY"),
                Line2D([0], [0], color="blue", label="AccZ"),
            ]
            acc_plt.plot(acc_x, self.accx_list, color="red", label="AccX")
            acc_plt.plot(acc_x, self.accy_list, color="green", label="AccY")
            acc_plt.plot(acc_x, self.accz_list, color="blue", label="AccZ")
            acc_plt.legend(acc_lines, ['AccX', 'AccY', 'AccZ'])
            acc_fig.suptitle('Acc')
            acc_fig.savefig(filename + '-acc')

            gyr_fig, gyr_plt = plt.subplots()
            gyr_x = list(range(len(self.gyrx_list)))
            gyr_lines = [
                Line2D([0], [0], color="red", label="GyrX"),
                Line2D([0], [0], color="green", label="GyrY"),
                Line2D([0], [0], color="blue", label="GyrZ"),
            ]
            gyr_plt.plot(gyr_x, self.gyrx_list, color="red", label="GyrX")
            gyr_plt.plot(gyr_x, self.gyry_list, color="green", label="GyrY")
            gyr_plt.plot(gyr_x, self.gyrz_list, color="blue", label="GyrZ")
            gyr_plt.legend(gyr_lines, ['GyrX', 'GyrY', 'GyrZ'])
            gyr_fig.suptitle('Gyr')
            gyr_fig.savefig(filename + '-gyr')
            plt.show()

    def _read_callback_USB(self, value):
        resp = self._cmd_helper.format_response(value)
        # print('_read_callback_USB', resp.data)
        self._read_temp.extend(resp.data)
        # print(self._read_temp)

        if '10' in self._read_temp:
            index_10 = self._read_temp.index('10')
            if len(self._read_temp) - index_10 >= 17:
                value_data = "".join(self._read_temp[index_10+1:index_10+17])
                del self._read_temp[index_10:index_10+17]
                logger.debug(f'value_data: {value_data}')
                self._parse_status(value_data)

    def _read_callback_BLE(self, handle, value):
        value_data = hexlify(value)
        logger.debug(f'value_data: {value_data}')
        self._parse_status(value_data)

    def _read_sensor_config_callback_BLE(self, handle, value):
        value_data = hexlify(value)
        logger.debug(f'_read_sensor_config_callback_BLE: {value_data}')
        acc_char = value_data[2:4]
        gyr_char = value_data[4:6]

        if acc_char != b'':
            self.acc_scale = get_key_by_val(ACC_FSR_CHAR, int(acc_char, 16))
            self.gyr_scale = get_key_by_val(GYRO_FSR_CHAR, int(gyr_char, 16))
            logger.debug(f'Acc scale: {self.acc_scale}, Gyr scale: {self.gyr_scale}')
        else:
            pass
        # self.dev.unsubscribe("0000fff7-0000-1000-8000-00805f9b34fb")
        # print('unsubscribe~!!!')

    def _set_sensor_config_callback_BLE(self, handle, value):
        value_data = hexlify(value)
        logger.debug(f'_set_sensor_config_callback_BLE: {value_data}')
        # acc_char = value_data[2:4]
        # gyr_char = value_data[4:6]

        # if acc_char != b'':
        #     self.acc_scale = get_key_by_val(ACC_FSR_CHAR, int(acc_char, 16))
        #     self.gyr_scale = get_key_by_val(GYRO_FSR_CHAR, int(gyr_char, 16))
        #     logger.debug(f'Acc scale: {self.acc_scale}, Gyr scale: {self.gyr_scale}')
        # else:
        #     pass

    def _parse_status(self, value_data):
        accx = convert_acc(value_data[:4], self.acc_scale)
        accy = convert_acc(value_data[4:8], self.acc_scale)
        accz = convert_acc(value_data[8:12], self.acc_scale)
        gyrx = convert_gyr(value_data[12:16], self.gyr_scale)
        gyry = convert_gyr(value_data[16:20], self.gyr_scale)
        gyrz = convert_gyr(value_data[20:24], self.gyr_scale)

        cur_count = int(value_data[24:28], 16)
        stored_count = int(value_data[28:], 16)
        if self.mode == 'USB':
            temp = value_data[26:28]
            temp_ = value_data[24:26]
            temp__ = temp+temp_
            cur_count = int(temp__, 16)  # byte由右至左看

            temp = value_data[26:28]
            temp_ = value_data[28:30]
            temp__ = temp+temp_
            stored_count = int(temp__, 16)  # byte由右至左看
        logger.debug(f'Acc: {accx} {accy} {accz}, Gyr: {gyrx} {gyry} {gyrz}')
        logger.debug(f'Current Count: {cur_count}, Stored Count: {stored_count}')

        self.accx_list.append(accx)
        self.accy_list.append(accy)
        self.accz_list.append(accz)
        self.gyrx_list.append(gyrx)
        self.gyry_list.append(gyry)
        self.gyrz_list.append(gyrz)

        # 將狀態數值丟給 custom_callback
        if self.custom_callback is not None:
            self.custom_callback(dict(Acc=(accx, accy, accz),
                                      Gyr=(gyrx, gyry, gyrz),
                                      Count=(cur_count, stored_count)))
