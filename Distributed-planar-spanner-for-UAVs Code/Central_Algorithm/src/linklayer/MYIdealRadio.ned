//
// Copyright (C) 2013 OpenSim Ltd
//
// This program is free software; you can redistribute it and/or
// modify it under the terms of the GNU Lesser General Public License
// as published by the Free Software Foundation; either version 2
// of the License, or (at your option) any later version.
//
// This program is distributed in the hope that it will be useful,
// but WITHOUT ANY WARRANTY; without even the implied warranty of
// MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
// GNU Lesser General Public License for more details.
//
// You should have received a copy of the GNU Lesser General Public License
// along with this program; if not, see <http://www.gnu.org/licenses/>.
//
// author: Zoltan Bojthe
//


package _25_8.Central_Algorithm.src.linklayer;
import inet.linklayer.radio.IRadio;

//
// Implements a unit disk radio for use in ~MYIdealWirelessNic.
//
// See ~MYIdealWirelessNic for rationale and details.
//
// See ~MYIdealChannelModel, ~MYIdealWirelessMac, ~MYIdealWirelessNic
//
simple MYIdealRadio like IRadio
{
    parameters:
        double bitrate @unit("bps");            // transmission speed [bit/sec]
        double transmissionRange @unit("m");    // max. distance from sender where reception is possible
        bool drawCoverage = default(true);      // draw the coverage area in Tkenv
        
        double criticalConnectivityStrip @unit("m");

        @display("i=block/wrxtx");
        @signal[radioState](type=long);
        
    gates:
        input upperLayerIn; // from higher layer protocol (MAC)
        output upperLayerOut; // to higher layer protocol (MAC)
        input radioIn @labels(MYIdealAirFrame); // to receive frames (MYIdealAirFrame) on the radio channel
}

