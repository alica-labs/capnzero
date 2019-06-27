# CapnZero Utilities  
This are some tools to help debugging capnzero applications.  

## Installation
There is an capnz.bash file that wraps activating the included venv and the script.
If you choose to run the script directly you can do so. The python scripts can be run like bash scripts.
But if you run the scripts directly you have to make shure you installed the dependencies.  

## Dependencies
ZeroMQ: ```pip3 install --user pyzmq```  
CapnProto: ```pip3 install --user pycapnp```  

## Usage
If you sourced the capnz.bash file you can use these tools like this:  
```bash
capnzero echo "udp://224.0.0.2:5555" "msg-string" "test"
capnzero echo -u
capnzero echo -l
```
else you can run the echo script like this:
```bash
./echo.py "udp://224.0.0.2:5555" "msg-string" "test"
./echo.py -u
./echo.py -l
```