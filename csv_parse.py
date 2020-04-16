# coding: UTF-8
import csv

with open('garbage_station.csv', 'r', encoding="utf_8") as read_file:
    reader = csv.reader(read_file)
    header = next(reader)  # ヘッダーを読み飛ばしたい時

    with open('test.csv','a') as write_file:
        writer = csv.writer(write_file)
        for row in reader:
            print(row)
            writer.writerow([row[1],float(row[4]),float(row[5]),row[7],row[8],row[9]])