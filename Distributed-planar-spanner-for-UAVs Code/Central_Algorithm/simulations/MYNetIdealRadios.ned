//
// Copyright (C) 2013 OpenSim Ltd.
//
// This program is free software; you can redistribute it and/or
// modify it under the terms of the GNU General Public License
// as published by the Free Software Foundation; either version 2
// of the License, or (at your option) any later version.
//
// This program is distributed in the hope that it will be useful,
// but WITHOUT ANY WARRANTY; without even the implied warranty of
// MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
// GNU General Public License for more details.
//
// You should have received a copy of the GNU General Public License
// along with this program; if not, write to the Free Software
// Foundation, Inc., 59 Temple Place - Suite 330, Boston, MA  02111-1307, USA
//

package _25_8.Central_Algorithm.simulations;

import inet.networklayer.autorouting.ipv4.IPv4NetworkConfigurator;
import _25_8.Central_Algorithm.src.nodes.MYAdhocHost;
import _25_8.Central_Algorithm.src.nodes.MYFinalDroneHost;
import _25_8.Central_Algorithm.src.world.MYIdealChannelModel;
import _25_8.Central_Algorithm.src.nodes.MYNFZ;

network Net
{
    parameters:
        int numHosts;
        int numDrones;
        int K_parameter;
        int C_parameter;
        int numNFZ;
        bool Central_Algorithm_On;
        @display("bgb=800,800;bgi=background/l,s");
    submodules:
        host[numHosts]: MYAdhocHost {
            parameters:
                  	@display("p=100,55;i=abstract/soldier4,#000000;b=16,16,rect;is=vs");

                
        }

        drone[numDrones]: MYFinalDroneHost {
            parameters:
                @display("p=35,155;i=abstract/drone,#004080;is=s");	//p is initial position of UNactive drones
        }
        cc: MYAdhocHost {
            parameters:
                @display("p=25,25;i=device/receiverdish,#008040");
        }
        channelControl: MYIdealChannelModel {
            parameters:
                @display("p=575,15");
        }
        NFZ[numNFZ]: MYNFZ;
        configurator: IPv4NetworkConfigurator {
            config = xml("<config><interface hosts='*' address='192.168.x.x' netmask='255.255.0.0'/></config>");

            @display("p=530,20");
        }
}

