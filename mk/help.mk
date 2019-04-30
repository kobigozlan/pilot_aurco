help:
	@echo ""
	@echo " ArduPilot Building"
	@echo " =================="
	@echo ""
	@echo " The following web page has detailed information on building the code"
	@echo "     http://dev.ardupilot.org/wiki/building-the-code/"
	@echo ""
	@echo " Linux boards should use waf build system"
	@echo ""
	@echo " Before building a target you need to be in the target vehicle type directory"
	@echo " e.g. ArduPlane, ArduCopter, APMrover2, AntennaTracker"
	@echo ""
	@echo " Most targets support a \"-upload\" extension to upload the firmware"
	@echo " to a connected board.  e.g. \"make px4-v2-upload\""
	@echo ""
	@echo " Some targets support a \"-debug\" extension to enable a debug build"
	@echo " (with debug symbols, and without optimisation)"
	@echo ""
	@echo " Note that the px4 builds are NOT parallel safe, NO -j flag"
	@echo ""
	@echo ""
	@echo " Targets"
	@echo " -------"
	@echo ""
	@echo "  px4-v1 - the PX4v1 board"
	@echo "  px4-v2 - the Pixhawk"
	@echo "  px4-v3 - the Pixhawk with 2M flash"
	@echo "  px4-v4 - the XRacer"
	@echo "  vrbrain - the VRBrain boards"
	@echo "  sitl - the SITL Software In The Loop simulation"
	@echo "  qflight - qualcomm flight board"
	@echo "  f4light - the OpenPilot Revolution Mini"
