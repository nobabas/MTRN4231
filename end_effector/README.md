Layout: The endeffector works through pin 14 of the teensy 4.1 connected to the data pin in the ur5e desktop through to the data pin of the soil sensor (yellow wire). The usb is still connected to the teensy from the desktop.

To run it you need to first have arduino with the teensy 4.1 library installed. The test file is Soil_test_for_ur5e.ino. This needs to be running before using it with brain.cpp. Make sure not to open terminal within arduino or it will not work.

If no access to endeffector: Run test_nodes/dummy_inputs.py to get fake values of soil
