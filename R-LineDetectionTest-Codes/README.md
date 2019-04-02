# Line Detection Test & Can Sweeping Test
This project is created to collect the sensor value continuously and periodically to acquire the graph of how the readings change. 

2 parts: 
1) LineDetectionTest
Exports 'linedata.csv' after robot acquires sensor data while driving in a square. 
It applies median filter to the sensor readings. 
14Hz data sampling rate. 
If you graph it using the values in CSV, you will see a triangular wave form. 


2) Can sweeping
Similarly but robot turns 360 degrees instead. 
Applies median filter. 
Exports usdata.csv