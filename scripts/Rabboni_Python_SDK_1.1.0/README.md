Rabboni Python SDK
========

Prerequisite
-----

### Linux
(建議: Ubuntu 20.04LTS)

* 安裝 libhidapi-hidraw0 (USB 模式)  
```
$ sudo apt install libhidapi-hidraw0
```
* 寫入 udev rules
  > 先將 rabboni-python-sdk-1.0.0 複製到  /usr/lib/python3/dist-packages
```
$ cd rabboni-python-sdk-1.0.0
$ sh generate-udev-rules.sh
```
* 安裝 python3 及相關  
```
$ sudo apt install python3 python3-pip python3-tk
```

### Windows

* 複製 hidapi.dll 到開發專案
```
> copy rabboni-python-sdk-1.0.0\hidapi.dll .
```

---

Installation
-----

```
$ cd rabboni-python-sdk-1.0.0
$ sudo python3 setup.py install
```

Usage
-----

### 與 Rabboni 建立連線/斷開連線

* 用 USB 連接
``` python
rab = Rabboni(mode='USB')
rab.connect()
```

* 用 BLE 連接
``` python
rab = Rabboni(mode='BLE')
rab.scan()
rab.connect(mac_address='E5:9F:B8:36:17:9A')
```

* 斷開連接
``` python
rab.disconnect()
```

### 讀取/修改設定值

``` python
rab.set_sensor_config(16, 2000, 40, 2500)
rab.read_sensor_config()
```

### 讀取 Rabboni 狀態數值

``` python
rab.start_fetching_status()
rab.polling_status()
```

### 重製紀錄數值

``` python
rab.reset_count()
```

### 匯出 CSV

``` python
rab.export_csv()
```

### 匯出折線圖

``` python
rab.export_plot()
```

---

Example
-----

* USB 模式

``` python
import traceback as tb

from rabboni import Rabboni

rab = Rabboni()
rab.connect()

rab.set_sensor_config(8, 500, 20, 100)


def usb_custom_callback(status):
    """
    參數 status 為一個 dictionary
    eg.
    Acc: (X,Y,Z), Gyr: (X,Y,Z), Count: (CurrentCount, StoredCount)
    {'Acc': (0.614013671875, -0.583984375, 0.58349609375), 'Gyr': (-92.437744140625, -99.212646484375, 0.0152587890625), 'Count': (98, 0)}
    """
    print(status)


try:
    rab.start_fetching_status(custom_callback=usb_custom_callback)
    rab.polling_status()
except AssertionError:  # 結束程式
    print('Bye~!!')
except Exception:
    tb.print_exc()
finally:
    rab.export_csv()
    rab.export_plot()
    rab.disconnect()
```

```
$ sudo python3 usb_rabboni.py
```

* BLE 模式

``` python
import traceback as tb

from rabboni import Rabboni

rab = Rabboni(mode='BLE')
rab.scan()
rab.connect(mac_address='E5:9F:B8:36:17:9A')

rab.read_sensor_config()
rab.set_sensor_config(4, 500, 20, 100)
rab.read_sensor_config()


def ble_custom_callback(status):
    print(status)


try:
    rab.start_fetching_status(ble_custom_callback)
    rab.polling_status()
except AssertionError:  # 結束程式
    print('Bye~!!')
except Exception:
    tb.print_exc()
finally:
    rab.export_csv()
    rab.export_plot()
    rab.disconnect()
```

```
$ sudo python3 ble_rabboni.py
```

* BLE 連線步驟

    1. 按下 Rabboni 右鍵，進入 BLE 連線模式(閃綠燈) 
    2. 執行 python 程式 (`python ble_example.py`)
    3. 成功連線訊息
```
Scan BLE devices:
Name :   MAC : 6C:EE:DB:1B:72:22
Name :   MAC : 3B:01:58:CD:DC:02
Name : RABBONI  MAC : E5:9F:B8:36:17:9A
Name :   MAC : 6F:C5:0A:44:24:38
=============================
2020-05-21 11:20:54,498 - E:\Code\dev-projects\NCTUTWTLab\rabboni-python-sdk\rabboni\rabboni.py - INFO - Open BLE device <pygatt.backends.bgapi.device.BGAPIBLEDevice object at 0x0000013971E88E48> (rabboni.py:226)
```
