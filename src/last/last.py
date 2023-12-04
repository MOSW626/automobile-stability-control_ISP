import serial
import csv

ser = serial.Serial('COM3', 115200)  # Replace 'COM3' with your Arduino's serial port

data_list = []

try:
    while True:
        data = ser.readline().decode('utf-8').strip()
        data_split = data.split(',')
        if len(data_split) == 3:
            time = data_split[0]
            angle = data_split[1]
            y_velocity = data_split[2]
            data_list.append([time, angle, y_velocity])
except KeyboardInterrupt:
    pass

ser.close()

with open('sensor_data.csv', 'w', newline='') as csvfile:
    csvwriter = csv.writer(csvfile)
    csvwriter.writerow(['Time', 'Angle', 'Y Velocity'])
    csvwriter.writerows(data_list)

print("Data saved to sensor_data.csv")
