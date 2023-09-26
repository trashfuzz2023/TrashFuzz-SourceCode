# Prerequisites
1. A computer powerful enogh for running Apollo+LGSVL. 
(Or two computers: one for Apollo, one for LGSVL)
2. antlr4
3. rtamt
4. python3

## Install antlr4 for TrashFuzz2023
Make sure installation of version antlr-4.8(the latest version is not supported):
[Install By Package](https://www.antlr.org/download/antlr-4.8-complete.jar)


## Install RTAMT for TrashFuzz
Please refer to [the github page](https://github.com/nickovic/rtamt) for installation of RTAMT.

# Step by step

## Run Apollo with LGSVL
Please refer to [the detailed documentation](https://www.svlsimulator.com/docs/system-under-test/apollo-master-instructions/) for co-simulation of Apollo with LGSVL.
Set the LGSVL to API-Only mode.

## Setup our bridge.
1. Download and go to the root. Note that the source code should be downloaded and set up on the computer running Apollo.
	```bash
	git clone https://github.com/trashfuzz2023/TrashFuzz-SourceCode.git
	cd TrashFuzz-SourceCode
	```
2. Connect our bridge to the LGSVL and Apollo:
	Go the bridge in the folder:/TrashFuzz-SourceCode/bridge
	```bash
	cd /root_of_TrashFuzz-SourceCode/bridge
	```
	Find file: [bridge.py](https://github.com/trashfuzz2023/TrashFuzz-SourceCode/blob/main/bridge/bridge_attack.py).
	There is class `Server` in [bridge.py](https://github.com/trashfuzz2023/TrashFuzz-SourceCode/blob/main/bridge/bridge.py). 

	Modify the `SIMULATOR_HOST` and `SIMULATOR_PORT` of `Server` to your IP and port of LGSVL.
	Modify the `BRIDGE_HOST` and `BRIDGE_PORT` of `Server` to your IP and port of Apollo.
	
3. Test the parser:
	If the support for parser is properly installed, we can test it by running:
	```bash
	cd /root_of_TrashFuzz-SourceCode
	python3 monitor.py
	```
	If there is no errors and warnings, the parser is correct.


## Run our bridge.
Open a terminal on the computer running Apollo.
```bash
cd /root_of_TrashFuzz-SourceCode/bridge
python3 bridge_attack.py
```
Keep it Running.


## Run the Fuzzing Algorithm.
Open another terminal on the computer running Apollo.
```bash
cd /root_of_TrashFuzz-SourceCode
python3 greedy.py
```
If the brige is set properly, you will see the LGSVL and Apollo running. The results will be put into a folder: The_Results.

