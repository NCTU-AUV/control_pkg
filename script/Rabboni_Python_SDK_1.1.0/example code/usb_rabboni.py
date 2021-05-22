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
