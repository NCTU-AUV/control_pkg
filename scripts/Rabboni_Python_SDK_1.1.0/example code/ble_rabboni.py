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
