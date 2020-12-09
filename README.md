# Distributed-Planar-Spanner-for-UAVs

About The Project 

The project is a collaboration between our department and RAFAEL which is an Israeli defense technology company. This is a continuous project. Our part  in this project included design and implementation of network architecture with Omnet++ simulation framework based on C++ programming. Implementation of new Distributed Planar Spanner for UAV’s sensors, to obtain a connected, sparse, self-stabilizing graph that can be maintained using only the local information between neighboring nodes. We built a new distributed algorithm that improves the communication between UAV (unmanned aerial vehicle) swarm and the control center which is located far away from all UAVs. The purpose of the algorithm is to get to an optimal state, minimize the number of links (edges) while avoiding congestions in the network. Congestion is defined by several parameters, each one of them have different importance and therefore receive different weight. The edge’s weight function  depends on all variables.

The Work

The main changes and extensions to the code are in the files:
1.	MYIdealAirFrame
2.	MYConstSpeedMobilityDrone
3.	MYIdealRadio
4.	MYIdealChannelModelAccess
5.	MYIdealChannelModel

Furthermore, as part of our work we improved existing code parts, and fixed bugs that were written in previous years. Those changes are not included in the list above.

Extra - In Google Drive https://drive.google.com/drive/u/1/folders/0AKYTDIobSmqVUk9PVA
• Videos of the simulation are attached
• Full info about the project, with final results, is attached in " SPANNER DESIGN FOR SWARM OF UAVs-Final Report.pdf"
