## Introduction   

This program shows that drone can be controlled through common commecical mobile connectivity such as wibro and LTE, and it does not need any RF receiver and transmitter as well.

You can get most of information related to this in [the site](http://tech.tigiminsight.com/2016/06/12/how-to-use-dronekit.html) even if those are written in Korean, because google translator can help you. :)  

## Open Source List

There are many open source used in this project 

- Raspberry Pi3 
- Pixhawk
- PyQT
- Dronekit
- Mavproxy

## How to test  

[This video](https://www.youtube.com/watch?v=6sRNNMlCmjM) shows how it works  

First, power up your [drone](https://pixhawk.org/platforms/multicopters/dji_flamewheel_450) with pixhawk annd push safety button before testing  

On drone, companion computer also powers up, and mavproxy runs as a bridge to both app and gcs like APM Planner2

	mavproxy.py  --udpout 127.0.0.1:14550 --tcpout 127.0.0.1:14550 

Also, in your laptop, you download this source code and run while gcs needs to connect, because it lets you know whole status of drone. Of course, environment settings are required as described [here](https://supertigim.github.io/2016/06/12/how-to-use-dronekit.html)

## License  
  
MIT License