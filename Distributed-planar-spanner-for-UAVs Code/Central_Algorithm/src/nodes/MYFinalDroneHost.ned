//
// This program is free software: you can redistribute it and/or modify
// it under the terms of the GNU Lesser General Public License as published by
// the Free Software Foundation, either version 3 of the License, or
// (at your option) any later version.
//
// This program is distributed in the hope that it will be useful,
// but WITHOUT ANY WARRANTY; without even the implied warranty of
// MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
// GNU Lesser General Public License for more details.
//
// You should have received a copy of the GNU Lesser General Public License
// along with this program.  If not, see http://www.gnu.org/licenses/.
//

package _25_8.Central_Algorithm.src.nodes;
import inet.applications.IPingApp;;
import inet.base.NotificationBoard;
import inet.linklayer.IExternalNic;

import inet.linklayer.IWirelessNic;

import inet.mobility.contract.IMobility;
import inet.networklayer.common.InterfaceTable;
import inet.networklayer.ipv4.RoutingTable;
import inet.status.NodeStatus;
import inet.util.PcapRecorder;
import _25_8.Central_Algorithm.src.nodes.MYNetworkLayer;


//
// A wireless host containing routing, mobility and battery components.
// Supports only IPv4 protocol, TCP and UDP as transport protocol.
// This is a typical mobile node which can participate in adhoc routing
// and may have TCP/UDP applications installed. Supports ICMP (ping) too.
//
// - By default contains a single wireless cards, however it can be configured
//   by the numRadios parameter. Wirless card type is configured by the
//   **.wlan.typename parameter. see:  inet.linklayer.ieee80211 or other
//   modules implementing ~IWirelessNic
// - Node mobility can be set using **.mobility.typename
//   see: inet.mobility and ~IMobility
//
module MYFinalDroneHost
{
    parameters:
        //AdhocHost.ned
        @display("i=device/cellphone");
        wlan[*].mgmtType = default("Ieee80211MgmtAdhoc");  // use adhoc management
  		//string routingProtocol @enum("AODVUU","DYMOUM","DYMO","DSRUU","OLSR","OLSR_ETX","DSDV_2","Batman") = default("");  // used mobile routing protocol. see: inet.networklayer.manetrouting
        //IPForward = default(true);
        //WirelessHost.ned
        int numRadios = default(1);
        
        //StandardHost.ned
        int numUdpApps = default(0);  // no of UDP apps. Specify the app types in INI file with udpApp[0..1].typename="UDPVideoStreamCli" syntax
        int numPingApps = default(0);  // no of PING apps. Specify the app types in INI file with pingApp[0..1].typename="MYPingApp" syntax        
        bool hasUdp = default(numUdpApps>0);
        string udpType = default(firstAvailable("UDP","UDP_None"));
     
        networkLayer.proxyARP = default(false);

        @node;
        @labels(node,ethernet-node,wireless-node);
        bool hasStatus = default(true);
        int numExtInterfaces = default(0);
        //int numRadios = default(0);               // the number of radios in the router. by default no wireless
        int numPcapRecorders = default(0); // no of PcapRecorders.
        string mobilityType = default(numRadios > 0 ? "MYStationaryMobility" : "");
        string routingFile = default("");
        bool IPForward = default(true);
        bool forwardMulticast = default(false);
        string batteryType = default("");
        // addings
        bool UsedAsLinker=default(false);
      //  Coord position=default(0,0,0);
     	double xPosition=0;
      	double yPosition=0;
     	 bool priority_case=default(false);
     	 int num_Of_Hosts_Cover=default(0);
     	 

    gates:
        input radioIn[numRadios] @directIn;

    submodules:
        pingApp[numPingApps]: <default("MYPingApp")> like IPingApp {
            parameters:
                @display("p=541,124,row,60");
        }

        status: NodeStatus if hasStatus {
            @display("p=50,50");
        }

        notificationBoard: NotificationBoard {
            parameters:
                @display("p=53,155;is=s");
        }

        // optional mobility module. Required only if wireless cards are present
        mobility: <mobilityType> like IMobility if mobilityType != "" {
            parameters:
                @display("p=53,111");
        }

        // network layer
        networkLayer: MYNetworkLayer {
            parameters:
                @display("p=329,287;q=queue");
        }

        routingTable: RoutingTable {
            parameters:
                @display("p=53,225;is=s");
                IPForward = IPForward;
                forwardMulticast = forwardMulticast;
                routingFile = routingFile;
        }

        // linklayer
        interfaceTable: InterfaceTable {
            parameters:
                @display("p=53,287;is=s");
        }

        pcapRecorder[numPcapRecorders]: PcapRecorder {
            @display("p=53,338,r,10");
        }

        wlan[numRadios]: <default("Ieee80211Nic")> like IWirelessNic {
            parameters:
                @display("p=329,403,row,60;q=queue");
        }
    connections allowunconnected:

        for i=0..numPingApps-1 {
            networkLayer.pingOut++ --> pingApp[i].pingIn;
            networkLayer.pingIn++ <-- pingApp[i].pingOut;
        }


         // connections to network outside
        for i=0..sizeof(radioIn)-1 {
            radioIn[i] --> { @display("m=s"); } --> wlan[i].radioIn;
            wlan[i].upperLayerOut --> networkLayer.ifIn++;
            wlan[i].upperLayerIn <-- networkLayer.ifOut++;
        }

}
