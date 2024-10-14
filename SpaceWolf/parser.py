import requests
import time

url =  "http://localhost:8080/available"

is_Have_Job = False
# this code should be run for start of the pi to end of the pi make it happen
while is_Have_Job != True:
    response = requests.get(url)

    print(response.json())

    response_data = response.json()  

    station = response_data.get("station")

    if isinstance(station, int):
        print("data sent to the driver")
        is_Have_Job = True
    else:
        print("que is empty no location sent ")
        is_Have_Job = False

    print(station)
    time.sleep(5)




