import time
import seeed_dht
import paho.mqtt.client as mqtt

def main():
    # for DHT11/DHT22
    sensor = seeed_dht.DHT("11", 12)
    # for DHT10
    # sensor = seeed_dht.DHT("10")

    # while True:
    humi, temp = sensor.read()

    #for x in range(1,50):
    var = ""
    if not humi is None:
        var = "DHT{0}, humidity {1:.1f}%, temperature {2:.1f}*".format(sensor.dht_type, humi, temp)
        # print('DHT{0}, humidity {1:.1f}%, temperature {2:.1f}*'.format(sensor.dht_type, humi, temp), file=open("./../logfiles/Temp_Hum.txt", "a"))
        print('DHT{0}, humidity {1:.1f}%, temperature {2:.1f}*'.format(sensor.dht_type, humi, temp))
    else:
        # print('DHT{0}, humidity & temperature: {1}'.format(sensor.dht_type, temp), file=open("./../logfiles/Temp_Hum.txt", "a"))
        var = "DHT{0}, humidity & temperature: {1}'.format(sensor.dht_type, temp)"
        print('DHT{0}, humidity & temperature: {1}'.format(sensor.dht_type, temp))
    # print(var)
    client.publish("topic/temp", var)
    # time.sleep(1)


if __name__ == '__main__':
    client = mqtt.Client()
    client.connect("172.16.116.133",1883,60)
    main()
    client.disconnect()
