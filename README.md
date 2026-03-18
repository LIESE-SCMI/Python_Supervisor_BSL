# Python interface to reprogram the MSP430FR5969 microcontroller through the BSL
This program reads TI-TXT files from the `titxt_files` folder, parses them and then send that to the BSL of the Supervisor through the Mission Boss

### Dependencies
- PySerial==3.5

### Creating virtual env and installing dependencies
In order to use the script a virtual env must be created to install dependencies. To do so, open a terminal and run the following commands inside the project's folder (BSL_Supervisor):
```bash
# In ~/BSL_Supervisor

$ py -m venv .venv # Used to create a virtual env named .venv
$ ./.venv/Scripts/activate # Windows only
$ ./.venv/bin/activate # Linux/MacOS only
$ pip install -r requirements.txt # Install dependencies
```

## Running script
Before running the script, a COM port and TI-TXT files placed in the corresponding folder must be set. Once everything is ready, run in a terminal:
```bash
$ py msp430_bsl_programmer.py 
```