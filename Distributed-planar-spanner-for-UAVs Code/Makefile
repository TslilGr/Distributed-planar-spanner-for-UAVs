#
# OMNeT++/OMNEST Makefile for 25_8
#
# This file was generated with the command:
#  opp_makemake -f --deep -O out -I../inet/src/networklayer/common -I../inet/src/linklayer/idealwireless -I../inet/src -I../inet/src/networklayer/icmpv6 -I../inet/src/world/obstacles -I../inet/src/networklayer/xmipv6 -I../inet/src/networklayer/contract -I../inet/src/networklayer/autorouting/ipv4 -I../inet/src/util -I../inet/src/linklayer/common -I../inet/src/transport/contract -I../inet/src/status -I../inet/src/linklayer/radio/propagation -I../inet/src/linklayer/radio -I../inet/src/util/headerserializers/tcp -I../inet/src/networklayer/ipv4 -I../inet/src/mobility/contract -I../inet/src/util/headerserializers/ipv4 -I../inet/src/base -I../inet/src/util/headerserializers -I../inet/src/world/radio -I../inet/src/networklayer/ipv6 -I../inet/src/transport/sctp -I../inet/src/util/headerserializers/udp -I../inet/src/networklayer/ipv6tunneling -I../inet/src/battery/models -I../inet/src/util/headerserializers/ipv6 -I../inet/src/applications/pingapp -I../inet/src/util/headerserializers/sctp -I../inet/src/linklayer/contract -I../inet/src/transport/tcp_common -I../inet/src/networklayer/arp -I../inet/src/transport/udp -L../inet/out/$(CONFIGNAME)/src -linet -DINET_IMPORT -KINET_PROJ=../inet
#

# Name of target to be created (-o option)
TARGET = 25_8$(EXE_SUFFIX)

# User interface (uncomment one) (-u option)
USERIF_LIBS = $(ALL_ENV_LIBS) # that is, $(TKENV_LIBS) $(CMDENV_LIBS)
#USERIF_LIBS = $(CMDENV_LIBS)
#USERIF_LIBS = $(TKENV_LIBS)

# C++ include paths (with -I)
INCLUDE_PATH = \
    -I../inet/src/networklayer/common \
    -I../inet/src/linklayer/idealwireless \
    -I../inet/src \
    -I../inet/src/networklayer/icmpv6 \
    -I../inet/src/world/obstacles \
    -I../inet/src/networklayer/xmipv6 \
    -I../inet/src/networklayer/contract \
    -I../inet/src/networklayer/autorouting/ipv4 \
    -I../inet/src/util \
    -I../inet/src/linklayer/common \
    -I../inet/src/transport/contract \
    -I../inet/src/status \
    -I../inet/src/linklayer/radio/propagation \
    -I../inet/src/linklayer/radio \
    -I../inet/src/util/headerserializers/tcp \
    -I../inet/src/networklayer/ipv4 \
    -I../inet/src/mobility/contract \
    -I../inet/src/util/headerserializers/ipv4 \
    -I../inet/src/base \
    -I../inet/src/util/headerserializers \
    -I../inet/src/world/radio \
    -I../inet/src/networklayer/ipv6 \
    -I../inet/src/transport/sctp \
    -I../inet/src/util/headerserializers/udp \
    -I../inet/src/networklayer/ipv6tunneling \
    -I../inet/src/battery/models \
    -I../inet/src/util/headerserializers/ipv6 \
    -I../inet/src/applications/pingapp \
    -I../inet/src/util/headerserializers/sctp \
    -I../inet/src/linklayer/contract \
    -I../inet/src/transport/tcp_common \
    -I../inet/src/networklayer/arp \
    -I../inet/src/transport/udp \
    -I. \
    -ICentral_Algorithm \
    -ICentral_Algorithm/out \
    -ICentral_Algorithm/out/gcc-debug \
    -ICentral_Algorithm/out/gcc-debug/src \
    -ICentral_Algorithm/out/gcc-debug/src/application \
    -ICentral_Algorithm/out/gcc-debug/src/linklayer \
    -ICentral_Algorithm/out/gcc-debug/src/mobility \
    -ICentral_Algorithm/out/gcc-debug/src/mobility/common \
    -ICentral_Algorithm/out/gcc-debug/src/mobility/single \
    -ICentral_Algorithm/out/gcc-debug/src/mobility/static \
    -ICentral_Algorithm/out/gcc-debug/src/world \
    -ICentral_Algorithm/simulations \
    -ICentral_Algorithm/simulations/results \
    -ICentral_Algorithm/src \
    -ICentral_Algorithm/src/application \
    -ICentral_Algorithm/src/linklayer \
    -ICentral_Algorithm/src/mobility \
    -ICentral_Algorithm/src/mobility/common \
    -ICentral_Algorithm/src/mobility/single \
    -ICentral_Algorithm/src/mobility/static \
    -ICentral_Algorithm/src/nodes \
    -ICentral_Algorithm/src/world

# Additional object and library files to link with
EXTRA_OBJS =

# Additional libraries (-L, -l options)
LIBS = -L../inet/out/$(CONFIGNAME)/src  -linet
LIBS += -Wl,-rpath,`abspath ../inet/out/$(CONFIGNAME)/src`

# Output directory
PROJECT_OUTPUT_DIR = out
PROJECTRELATIVE_PATH =
O = $(PROJECT_OUTPUT_DIR)/$(CONFIGNAME)/$(PROJECTRELATIVE_PATH)

# Object files for local .cc and .msg files
OBJS = \
    $O/Central_Algorithm/src/application/MYPingApp.o \
    $O/Central_Algorithm/src/linklayer/MYIdealRadio.o \
    $O/Central_Algorithm/src/linklayer/MYIdealWirelessMac.o \
    $O/Central_Algorithm/src/mobility/common/MYLineSegmentsMobilityBase.o \
    $O/Central_Algorithm/src/mobility/common/MYMobilityBase.o \
    $O/Central_Algorithm/src/mobility/common/MYMovingMobilityBase.o \
    $O/Central_Algorithm/src/mobility/single/MYConstSpeedMobilityDrone.o \
    $O/Central_Algorithm/src/mobility/single/MYRandomWPMobility.o \
    $O/Central_Algorithm/src/mobility/static/MYStationaryMobility.o \
    $O/Central_Algorithm/src/world/MYIdealChannelModel.o \
    $O/Central_Algorithm/src/world/MYIdealChannelModelAccess.o \
    $O/Central_Algorithm/src/linklayer/MYIdealAirFrame_m.o

# Message files
MSGFILES = \
    Central_Algorithm/src/linklayer/MYIdealAirFrame.msg

# Other makefile variables (-K)
INET_PROJ=../inet

#------------------------------------------------------------------------------

# Pull in OMNeT++ configuration (Makefile.inc or configuser.vc)

ifneq ("$(OMNETPP_CONFIGFILE)","")
CONFIGFILE = $(OMNETPP_CONFIGFILE)
else
ifneq ("$(OMNETPP_ROOT)","")
CONFIGFILE = $(OMNETPP_ROOT)/Makefile.inc
else
CONFIGFILE = $(shell opp_configfilepath)
endif
endif

ifeq ("$(wildcard $(CONFIGFILE))","")
$(error Config file '$(CONFIGFILE)' does not exist -- add the OMNeT++ bin directory to the path so that opp_configfilepath can be found, or set the OMNETPP_CONFIGFILE variable to point to Makefile.inc)
endif

include $(CONFIGFILE)

# Simulation kernel and user interface libraries
OMNETPP_LIB_SUBDIR = $(OMNETPP_LIB_DIR)/$(TOOLCHAIN_NAME)
OMNETPP_LIBS = -L"$(OMNETPP_LIB_SUBDIR)" -L"$(OMNETPP_LIB_DIR)" -loppmain$D $(USERIF_LIBS) $(KERNEL_LIBS) $(SYS_LIBS)

COPTS = $(CFLAGS) -DINET_IMPORT $(INCLUDE_PATH) -I$(OMNETPP_INCL_DIR)
MSGCOPTS = $(INCLUDE_PATH)

# we want to recompile everything if COPTS changes,
# so we store COPTS into $COPTS_FILE and have object
# files depend on it (except when "make depend" was called)
COPTS_FILE = $O/.last-copts
ifneq ($(MAKECMDGOALS),depend)
ifneq ("$(COPTS)","$(shell cat $(COPTS_FILE) 2>/dev/null || echo '')")
$(shell $(MKPATH) "$O" && echo "$(COPTS)" >$(COPTS_FILE))
endif
endif

#------------------------------------------------------------------------------
# User-supplied makefile fragment(s)
# >>>
# <<<
#------------------------------------------------------------------------------

# Main target
all: $O/$(TARGET)
	$(Q)$(LN) $O/$(TARGET) .

$O/$(TARGET): $(OBJS)  $(wildcard $(EXTRA_OBJS)) Makefile
	@$(MKPATH) $O
	@echo Creating executable: $@
	$(Q)$(CXX) $(LDFLAGS) -o $O/$(TARGET)  $(OBJS) $(EXTRA_OBJS) $(AS_NEEDED_OFF) $(WHOLE_ARCHIVE_ON) $(LIBS) $(WHOLE_ARCHIVE_OFF) $(OMNETPP_LIBS)

.PHONY: all clean cleanall depend msgheaders

.SUFFIXES: .cc

$O/%.o: %.cc $(COPTS_FILE)
	@$(MKPATH) $(dir $@)
	$(qecho) "$<"
	$(Q)$(CXX) -c $(COPTS) -o $@ $<

%_m.cc %_m.h: %.msg
	$(qecho) MSGC: $<
	$(Q)$(MSGC) -s _m.cc $(MSGCOPTS) $?

msgheaders: $(MSGFILES:.msg=_m.h)

clean:
	$(qecho) Cleaning...
	$(Q)-rm -rf $O
	$(Q)-rm -f 25_8 25_8.exe lib25_8.so lib25_8.a lib25_8.dll lib25_8.dylib
	$(Q)-rm -f ./*_m.cc ./*_m.h
	$(Q)-rm -f Central_Algorithm/*_m.cc Central_Algorithm/*_m.h
	$(Q)-rm -f Central_Algorithm/out/*_m.cc Central_Algorithm/out/*_m.h
	$(Q)-rm -f Central_Algorithm/out/gcc-debug/*_m.cc Central_Algorithm/out/gcc-debug/*_m.h
	$(Q)-rm -f Central_Algorithm/out/gcc-debug/src/*_m.cc Central_Algorithm/out/gcc-debug/src/*_m.h
	$(Q)-rm -f Central_Algorithm/out/gcc-debug/src/application/*_m.cc Central_Algorithm/out/gcc-debug/src/application/*_m.h
	$(Q)-rm -f Central_Algorithm/out/gcc-debug/src/linklayer/*_m.cc Central_Algorithm/out/gcc-debug/src/linklayer/*_m.h
	$(Q)-rm -f Central_Algorithm/out/gcc-debug/src/mobility/*_m.cc Central_Algorithm/out/gcc-debug/src/mobility/*_m.h
	$(Q)-rm -f Central_Algorithm/out/gcc-debug/src/mobility/common/*_m.cc Central_Algorithm/out/gcc-debug/src/mobility/common/*_m.h
	$(Q)-rm -f Central_Algorithm/out/gcc-debug/src/mobility/single/*_m.cc Central_Algorithm/out/gcc-debug/src/mobility/single/*_m.h
	$(Q)-rm -f Central_Algorithm/out/gcc-debug/src/mobility/static/*_m.cc Central_Algorithm/out/gcc-debug/src/mobility/static/*_m.h
	$(Q)-rm -f Central_Algorithm/out/gcc-debug/src/world/*_m.cc Central_Algorithm/out/gcc-debug/src/world/*_m.h
	$(Q)-rm -f Central_Algorithm/simulations/*_m.cc Central_Algorithm/simulations/*_m.h
	$(Q)-rm -f Central_Algorithm/simulations/results/*_m.cc Central_Algorithm/simulations/results/*_m.h
	$(Q)-rm -f Central_Algorithm/src/*_m.cc Central_Algorithm/src/*_m.h
	$(Q)-rm -f Central_Algorithm/src/application/*_m.cc Central_Algorithm/src/application/*_m.h
	$(Q)-rm -f Central_Algorithm/src/linklayer/*_m.cc Central_Algorithm/src/linklayer/*_m.h
	$(Q)-rm -f Central_Algorithm/src/mobility/*_m.cc Central_Algorithm/src/mobility/*_m.h
	$(Q)-rm -f Central_Algorithm/src/mobility/common/*_m.cc Central_Algorithm/src/mobility/common/*_m.h
	$(Q)-rm -f Central_Algorithm/src/mobility/single/*_m.cc Central_Algorithm/src/mobility/single/*_m.h
	$(Q)-rm -f Central_Algorithm/src/mobility/static/*_m.cc Central_Algorithm/src/mobility/static/*_m.h
	$(Q)-rm -f Central_Algorithm/src/nodes/*_m.cc Central_Algorithm/src/nodes/*_m.h
	$(Q)-rm -f Central_Algorithm/src/world/*_m.cc Central_Algorithm/src/world/*_m.h

cleanall: clean
	$(Q)-rm -rf $(PROJECT_OUTPUT_DIR)

depend:
	$(qecho) Creating dependencies...
	$(Q)$(MAKEDEPEND) $(INCLUDE_PATH) -f Makefile -P\$$O/ -- $(MSG_CC_FILES)  ./*.cc Central_Algorithm/*.cc Central_Algorithm/out/*.cc Central_Algorithm/out/gcc-debug/*.cc Central_Algorithm/out/gcc-debug/src/*.cc Central_Algorithm/out/gcc-debug/src/application/*.cc Central_Algorithm/out/gcc-debug/src/linklayer/*.cc Central_Algorithm/out/gcc-debug/src/mobility/*.cc Central_Algorithm/out/gcc-debug/src/mobility/common/*.cc Central_Algorithm/out/gcc-debug/src/mobility/single/*.cc Central_Algorithm/out/gcc-debug/src/mobility/static/*.cc Central_Algorithm/out/gcc-debug/src/world/*.cc Central_Algorithm/simulations/*.cc Central_Algorithm/simulations/results/*.cc Central_Algorithm/src/*.cc Central_Algorithm/src/application/*.cc Central_Algorithm/src/linklayer/*.cc Central_Algorithm/src/mobility/*.cc Central_Algorithm/src/mobility/common/*.cc Central_Algorithm/src/mobility/single/*.cc Central_Algorithm/src/mobility/static/*.cc Central_Algorithm/src/nodes/*.cc Central_Algorithm/src/world/*.cc

# DO NOT DELETE THIS LINE -- make depend depends on it.
$O/Central_Algorithm/src/application/MYPingApp.o: Central_Algorithm/src/application/MYPingApp.cc \
	Central_Algorithm/src/application/MYPingApp.h \
	$(INET_PROJ)/src/applications/pingapp/PingPayload_m.h \
	$(INET_PROJ)/src/base/Compat.h \
	$(INET_PROJ)/src/base/ILifecycle.h \
	$(INET_PROJ)/src/base/INETDefs.h \
	$(INET_PROJ)/src/base/LifecycleOperation.h \
	$(INET_PROJ)/src/base/ModuleAccess.h \
	$(INET_PROJ)/src/base/NodeOperations.h \
	$(INET_PROJ)/src/linklayer/contract/MACAddress.h \
	$(INET_PROJ)/src/networklayer/contract/IPProtocolId_m.h \
	$(INET_PROJ)/src/networklayer/contract/IPv4Address.h \
	$(INET_PROJ)/src/networklayer/contract/IPv4ControlInfo.h \
	$(INET_PROJ)/src/networklayer/contract/IPv4ControlInfo_m.h \
	$(INET_PROJ)/src/networklayer/contract/IPv6Address.h \
	$(INET_PROJ)/src/networklayer/contract/IPv6ControlInfo.h \
	$(INET_PROJ)/src/networklayer/contract/IPv6ControlInfo_m.h \
	$(INET_PROJ)/src/networklayer/contract/IPvXAddress.h \
	$(INET_PROJ)/src/networklayer/contract/IPvXAddressResolver.h \
	$(INET_PROJ)/src/status/NodeStatus.h
$O/Central_Algorithm/src/linklayer/MYIdealAirFrame_m.o: Central_Algorithm/src/linklayer/MYIdealAirFrame_m.cc \
	Central_Algorithm/src/linklayer/MYIdealAirFrame_m.h \
	$(INET_PROJ)/src/base/Compat.h \
	$(INET_PROJ)/src/base/Coord.h \
	$(INET_PROJ)/src/base/INETDefs.h \
	$(INET_PROJ)/src/util/FWMath.h
$O/Central_Algorithm/src/linklayer/MYIdealRadio.o: Central_Algorithm/src/linklayer/MYIdealRadio.cc \
	Central_Algorithm/src/linklayer/MYIdealAirFrame_m.h \
	Central_Algorithm/src/linklayer/MYIdealRadio.h \
	Central_Algorithm/src/world/MYIdealChannelModel.h \
	Central_Algorithm/src/world/MYIdealChannelModelAccess.h \
	$(INET_PROJ)/src/base/BasicModule.h \
	$(INET_PROJ)/src/base/Compat.h \
	$(INET_PROJ)/src/base/Coord.h \
	$(INET_PROJ)/src/base/ILifecycle.h \
	$(INET_PROJ)/src/base/INETDefs.h \
	$(INET_PROJ)/src/base/INotifiable.h \
	$(INET_PROJ)/src/base/LifecycleOperation.h \
	$(INET_PROJ)/src/base/ModuleAccess.h \
	$(INET_PROJ)/src/base/NodeOperations.h \
	$(INET_PROJ)/src/base/NotificationBoard.h \
	$(INET_PROJ)/src/base/NotifierConsts.h \
	$(INET_PROJ)/src/linklayer/contract/RadioState.h \
	$(INET_PROJ)/src/status/NodeStatus.h \
	$(INET_PROJ)/src/util/FWMath.h
$O/Central_Algorithm/src/linklayer/MYIdealWirelessMac.o: Central_Algorithm/src/linklayer/MYIdealWirelessMac.cc \
	Central_Algorithm/src/linklayer/MYIdealAirFrame_m.h \
	Central_Algorithm/src/linklayer/MYIdealRadio.h \
	Central_Algorithm/src/linklayer/MYIdealWirelessMac.h \
	Central_Algorithm/src/world/MYIdealChannelModel.h \
	Central_Algorithm/src/world/MYIdealChannelModelAccess.h \
	$(INET_PROJ)/src/base/BasicModule.h \
	$(INET_PROJ)/src/base/Compat.h \
	$(INET_PROJ)/src/base/Coord.h \
	$(INET_PROJ)/src/base/ILifecycle.h \
	$(INET_PROJ)/src/base/INETDefs.h \
	$(INET_PROJ)/src/base/INotifiable.h \
	$(INET_PROJ)/src/base/IPassiveQueue.h \
	$(INET_PROJ)/src/base/LifecycleOperation.h \
	$(INET_PROJ)/src/base/ModuleAccess.h \
	$(INET_PROJ)/src/base/NodeOperations.h \
	$(INET_PROJ)/src/base/NotificationBoard.h \
	$(INET_PROJ)/src/base/NotifierConsts.h \
	$(INET_PROJ)/src/linklayer/common/MACBase.h \
	$(INET_PROJ)/src/linklayer/common/WirelessMacBase.h \
	$(INET_PROJ)/src/linklayer/contract/Ieee802Ctrl_m.h \
	$(INET_PROJ)/src/linklayer/contract/MACAddress.h \
	$(INET_PROJ)/src/linklayer/contract/RadioState.h \
	$(INET_PROJ)/src/linklayer/idealwireless/IdealWirelessFrame_m.h \
	$(INET_PROJ)/src/networklayer/common/IInterfaceTable.h \
	$(INET_PROJ)/src/networklayer/common/InterfaceEntry.h \
	$(INET_PROJ)/src/networklayer/common/InterfaceTableAccess.h \
	$(INET_PROJ)/src/networklayer/common/InterfaceToken.h \
	$(INET_PROJ)/src/networklayer/contract/IPv4Address.h \
	$(INET_PROJ)/src/networklayer/contract/IPv6Address.h \
	$(INET_PROJ)/src/networklayer/contract/IPvXAddress.h \
	$(INET_PROJ)/src/status/NodeStatus.h \
	$(INET_PROJ)/src/util/FWMath.h \
	$(INET_PROJ)/src/util/opp_utils.h
$O/Central_Algorithm/src/mobility/common/MYLineSegmentsMobilityBase.o: Central_Algorithm/src/mobility/common/MYLineSegmentsMobilityBase.cc \
	Central_Algorithm/src/mobility/common/MYLineSegmentsMobilityBase.h \
	Central_Algorithm/src/mobility/common/MYMobilityBase.h \
	Central_Algorithm/src/mobility/common/MYMovingMobilityBase.h \
	Central_Algorithm/src/world/MYIdealChannelModel.h \
	$(INET_PROJ)/src/base/Compat.h \
	$(INET_PROJ)/src/base/Coord.h \
	$(INET_PROJ)/src/base/INETDefs.h \
	$(INET_PROJ)/src/base/ModuleAccess.h \
	$(INET_PROJ)/src/mobility/contract/IMobility.h \
	$(INET_PROJ)/src/util/FWMath.h
$O/Central_Algorithm/src/mobility/common/MYMobilityBase.o: Central_Algorithm/src/mobility/common/MYMobilityBase.cc \
	Central_Algorithm/src/mobility/common/MYMobilityBase.h \
	Central_Algorithm/src/world/MYIdealChannelModel.h \
	$(INET_PROJ)/src/base/Compat.h \
	$(INET_PROJ)/src/base/Coord.h \
	$(INET_PROJ)/src/base/INETDefs.h \
	$(INET_PROJ)/src/base/ModuleAccess.h \
	$(INET_PROJ)/src/mobility/contract/IMobility.h \
	$(INET_PROJ)/src/util/FWMath.h
$O/Central_Algorithm/src/mobility/common/MYMovingMobilityBase.o: Central_Algorithm/src/mobility/common/MYMovingMobilityBase.cc \
	Central_Algorithm/src/mobility/common/MYMobilityBase.h \
	Central_Algorithm/src/mobility/common/MYMovingMobilityBase.h \
	Central_Algorithm/src/world/MYIdealChannelModel.h \
	$(INET_PROJ)/src/base/Compat.h \
	$(INET_PROJ)/src/base/Coord.h \
	$(INET_PROJ)/src/base/INETDefs.h \
	$(INET_PROJ)/src/base/ModuleAccess.h \
	$(INET_PROJ)/src/mobility/contract/IMobility.h \
	$(INET_PROJ)/src/util/FWMath.h
$O/Central_Algorithm/src/mobility/single/MYConstSpeedMobilityDrone.o: Central_Algorithm/src/mobility/single/MYConstSpeedMobilityDrone.cc \
	Central_Algorithm/src/linklayer/MYIdealAirFrame_m.h \
	Central_Algorithm/src/linklayer/MYIdealRadio.h \
	Central_Algorithm/src/mobility/common/MYLineSegmentsMobilityBase.h \
	Central_Algorithm/src/mobility/common/MYMobilityBase.h \
	Central_Algorithm/src/mobility/common/MYMovingMobilityBase.h \
	Central_Algorithm/src/mobility/single/MYConstSpeedMobilityDrone.h \
	Central_Algorithm/src/world/MYIdealChannelModel.h \
	Central_Algorithm/src/world/MYIdealChannelModelAccess.h \
	$(INET_PROJ)/src/base/BasicModule.h \
	$(INET_PROJ)/src/base/Compat.h \
	$(INET_PROJ)/src/base/Coord.h \
	$(INET_PROJ)/src/base/ILifecycle.h \
	$(INET_PROJ)/src/base/INETDefs.h \
	$(INET_PROJ)/src/base/INotifiable.h \
	$(INET_PROJ)/src/base/ModuleAccess.h \
	$(INET_PROJ)/src/base/NotificationBoard.h \
	$(INET_PROJ)/src/base/NotifierConsts.h \
	$(INET_PROJ)/src/linklayer/contract/RadioState.h \
	$(INET_PROJ)/src/mobility/contract/IMobility.h \
	$(INET_PROJ)/src/util/FWMath.h
$O/Central_Algorithm/src/mobility/single/MYRandomWPMobility.o: Central_Algorithm/src/mobility/single/MYRandomWPMobility.cc \
	Central_Algorithm/src/mobility/common/MYLineSegmentsMobilityBase.h \
	Central_Algorithm/src/mobility/common/MYMobilityBase.h \
	Central_Algorithm/src/mobility/common/MYMovingMobilityBase.h \
	Central_Algorithm/src/mobility/single/MYRandomWPMobility.h \
	Central_Algorithm/src/world/MYIdealChannelModel.h \
	$(INET_PROJ)/src/base/Compat.h \
	$(INET_PROJ)/src/base/Coord.h \
	$(INET_PROJ)/src/base/INETDefs.h \
	$(INET_PROJ)/src/base/ModuleAccess.h \
	$(INET_PROJ)/src/mobility/contract/IMobility.h \
	$(INET_PROJ)/src/util/FWMath.h
$O/Central_Algorithm/src/mobility/static/MYStationaryMobility.o: Central_Algorithm/src/mobility/static/MYStationaryMobility.cc \
	Central_Algorithm/src/mobility/common/MYMobilityBase.h \
	Central_Algorithm/src/mobility/static/MYStationaryMobility.h \
	Central_Algorithm/src/world/MYIdealChannelModel.h \
	$(INET_PROJ)/src/base/Compat.h \
	$(INET_PROJ)/src/base/Coord.h \
	$(INET_PROJ)/src/base/INETDefs.h \
	$(INET_PROJ)/src/base/ModuleAccess.h \
	$(INET_PROJ)/src/mobility/contract/IMobility.h \
	$(INET_PROJ)/src/util/FWMath.h
$O/Central_Algorithm/src/world/MYIdealChannelModel.o: Central_Algorithm/src/world/MYIdealChannelModel.cc \
	Central_Algorithm/src/linklayer/MYIdealAirFrame_m.h \
	Central_Algorithm/src/linklayer/MYIdealRadio.h \
	Central_Algorithm/src/world/MYIdealChannelModel.h \
	Central_Algorithm/src/world/MYIdealChannelModelAccess.h \
	$(INET_PROJ)/src/base/BasicModule.h \
	$(INET_PROJ)/src/base/Compat.h \
	$(INET_PROJ)/src/base/Coord.h \
	$(INET_PROJ)/src/base/ILifecycle.h \
	$(INET_PROJ)/src/base/INETDefs.h \
	$(INET_PROJ)/src/base/INotifiable.h \
	$(INET_PROJ)/src/base/ModuleAccess.h \
	$(INET_PROJ)/src/base/NotificationBoard.h \
	$(INET_PROJ)/src/base/NotifierConsts.h \
	$(INET_PROJ)/src/linklayer/contract/RadioState.h \
	$(INET_PROJ)/src/util/FWMath.h
$O/Central_Algorithm/src/world/MYIdealChannelModelAccess.o: Central_Algorithm/src/world/MYIdealChannelModelAccess.cc \
	Central_Algorithm/src/linklayer/MYIdealAirFrame_m.h \
	Central_Algorithm/src/linklayer/MYIdealRadio.h \
	Central_Algorithm/src/mobility/common/MYLineSegmentsMobilityBase.h \
	Central_Algorithm/src/mobility/common/MYMobilityBase.h \
	Central_Algorithm/src/mobility/common/MYMovingMobilityBase.h \
	Central_Algorithm/src/mobility/single/MYConstSpeedMobilityDrone.h \
	Central_Algorithm/src/world/MYIdealChannelModel.h \
	Central_Algorithm/src/world/MYIdealChannelModelAccess.h \
	$(INET_PROJ)/src/base/BasicModule.h \
	$(INET_PROJ)/src/base/Compat.h \
	$(INET_PROJ)/src/base/Coord.h \
	$(INET_PROJ)/src/base/ILifecycle.h \
	$(INET_PROJ)/src/base/INETDefs.h \
	$(INET_PROJ)/src/base/INotifiable.h \
	$(INET_PROJ)/src/base/ModuleAccess.h \
	$(INET_PROJ)/src/base/NotificationBoard.h \
	$(INET_PROJ)/src/base/NotifierConsts.h \
	$(INET_PROJ)/src/linklayer/contract/RadioState.h \
	$(INET_PROJ)/src/mobility/contract/IMobility.h \
	$(INET_PROJ)/src/util/FWMath.h

