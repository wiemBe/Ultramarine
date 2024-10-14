import requests
import time

url =  "http://localhost:8080/available"

is_Have_Job = False
at_Target = False
# this code should be run for start of the pi to end of the pi make it happen
while is_Have_Job != True and at_Target != True:
    response = requests.get(url)

    print(response.json())

    response_data = response.json()  

    station = response_data.get("station")

    if isinstance(station, int):
        print("data sent to the driver")
        is_Have_Job = True
        #TODO:After the setting up the ros handle here
        # need to send this value to arduino over GPIO and to the ros and find my location and return a value at_Target as true
        # bussines logic will go here 
    else:
        print("que is empty no location sent ")
        is_Have_Job = False

    print(station)
    
    #TODO:will be gone
    time.sleep(5) 
    




