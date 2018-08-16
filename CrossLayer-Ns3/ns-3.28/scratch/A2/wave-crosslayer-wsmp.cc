/* -*-  Mode: C++; c-file-style: "gnu"; indent-tabs-mode:nil; -*- */
/*
 * Copyright (c) 2005,2006,2007 INRIA
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License version 2 as
 * published by the Free Software Foundation;
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with this program; if not, write to the Free Software
 * Foundation, Inc., 59 Temple Place, Suite 330, Boston, MA  02111-1307  USA
 *
 * * Author: Jenny sheng <qdsheng@gmail.com>
 *
 *
 *This project simulate the PHY-MAC-NET interaction cross layer design to increase throughput  Node 1 advertising BSM to node 0:
 *Physical layer parameters
 *1.the transmission power, data rate
 *2.Channel capacity, PDR, BitErrorRate, Propagation loss-> no channel switching because of the emergency bsm
 *3.Rayleigh fading, low signal to noise levels
 *Mac layer task: scheduling the packet delivery, packet segmentation, packet collisions
 *Net layer task: routing,  congestion control
 *
 *
 *
 * The script draws from several ns-3 examples, including:
 * /examples/routing/manet-routing-compare.cc
 * /examples/wave/wave-simple-wsmp.cc
 * /src/propagation/model/itu-r-1411-los-propagation-loss-model.cc
 * /src/mobility/examples/ns2-mobility-trace.cc
 * /src/wave/examples/wave-simple-80211p.cc
 *
 *
 *
 *
 */

#include <bits/stdint-uintn.h>
#include <iostream>
#include <string>

#include "../../build/ns3/address.h"
#include "../../build/ns3/animation-interface.h"
#include "../../build/ns3/callback.h"
#include "../../build/ns3/channel-manager.h"
#include "../../build/ns3/channel-scheduler.h"
#include "../../build/ns3/command-line.h"
#include "../../build/ns3/constant-acceleration-mobility-model.h"
#include "../../build/ns3/event-id.h"
#include "../../build/ns3/event-impl.h"
#include "../../build/ns3/log.h"
#include "../../build/ns3/log-macros-disabled.h"
#include "../../build/ns3/mac48-address.h"
#include "../../build/ns3/mobility-helper.h"
#include "../../build/ns3/net-device-container.h"
#include "../../build/ns3/node.h"
#include "../../build/ns3/node-container.h"
#include "../../build/ns3/nstime.h"
#include "../../build/ns3/packet.h"
#include "../../build/ns3/ptr.h"
#include "../../build/ns3/simulator.h"
#include "../../build/ns3/string.h"
#include "../../build/ns3/type-id.h"
#include "../../build/ns3/vector.h"
#include "../../build/ns3/wave-helper.h"
#include "../../build/ns3/wave-mac-helper.h"
#include "../../build/ns3/wave-net-device.h"
#include "../../build/ns3/wifi-helper.h"
#include "../../build/ns3/wsmp-socket.h"
#include "../../build/ns3/yans-wifi-helper.h"

using namespace ns3;

NS_LOG_COMPONENT_DEFINE("WaveWsmp");

void ReceivePacket(Ptr<WsmpSocket> socket) {
	while (socket->Recv()) {
		NS_LOG_UNCOND("Received one packet!");
	}
}

static void GenerateTraffic(Ptr<WsmpSocket> socket, uint32_t pktSize,
		uint32_t pktCount, Time pktInterval) {
	if (pktCount > 0) {
		socket->Send(Create<Packet>(pktSize));
		Simulator::Schedule(pktInterval, &GenerateTraffic, socket, pktSize,
				pktCount - 1, pktInterval);
	} else {
		socket->Close();
	}
}

int main(int argc, char *argv[]) {
	std::string phyMode("OfdmRate6MbpsBW10MHz");
	uint32_t packetSize = 1000; // bytes
	uint32_t numPackets = 1;
	double interval = 1.0; // seconds
	bool verbose = false;

	CommandLine cmd;

	cmd.AddValue("phyMode", "Wifi Phy mode", phyMode);
	cmd.AddValue("packetSize", "size of application packet sent", packetSize);
	cmd.AddValue("numPackets", "number of packets generated", numPackets);
	cmd.AddValue("interval", "interval (seconds) between packets", interval);
	cmd.AddValue("verbose", "turn on all WifiNetDevice log components",
			verbose);
	cmd.Parse(argc, argv);
	// Convert to time object
	Time interPacketInterval = Seconds(interval);

	NodeContainer nodes;
	nodes.Create(2);

	YansWifiChannelHelper waveChannel = YansWifiChannelHelper::Default();
	YansWavePhyHelper wavePhy = YansWavePhyHelper::Default();
	wavePhy.SetChannel(waveChannel.Create());
	wavePhy.SetPcapDataLinkType(YansWifiPhyHelper::DLT_IEEE802_11);
	QosWaveMacHelper waveMac = QosWaveMacHelper::Default();
	WaveHelper waveHelper = WaveHelper::Default();
	waveHelper.SetRemoteStationManager("ns3::ConstantRateWifiManager",
			"DataMode", StringValue(phyMode), "ControlMode",
			StringValue(phyMode));
	if (verbose) {
		waveHelper.EnableLogComponents();      // Turn on all WAVE logging
	}
	NetDeviceContainer devices = waveHelper.Install(wavePhy, waveMac, nodes);

	// If we do not assign channel access here, the WAVE devices will be assigned
	// continuous CCH access by default.
	for (uint32_t i = 0; i != devices.GetN(); i++) {
		Ptr<WaveNetDevice> device = DynamicCast<WaveNetDevice>(devices.Get(i));
		// Alternating access without immediate channel switch
		const SchInfo schInfo = SchInfo(CCH, false, EXTENDED_ALTERNATING);
		// An important point is that the receiver should also be assigned channel
		// access for the same channel to receive packets.
		Simulator::Schedule(Seconds(0.0), &WaveNetDevice::StartSch, device,
				schInfo);
	}

	// Tracing
	wavePhy.EnablePcap("wave-simple-wsmp", devices);

	MobilityHelper mobility;
	mobility.SetMobilityModel("ns3::ConstantAccelerationMobilityModel");
	mobility.Install(nodes);
	Ptr<ConstantAccelerationMobilityModel> mob = nodes.Get(0)->GetObject<
			ConstantAccelerationMobilityModel>();
	mob->SetVelocityAndAcceleration(Vector(60, 00.00, 00.00),
			Vector(10, 00.00, 00.00));

	std::cout << "Node " << "0" << " velocity: " << mob->GetVelocity()
			<< std::endl;
	Ptr<ConstantAccelerationMobilityModel> mob2 = nodes.Get(1)->GetObject<
			ConstantAccelerationMobilityModel>();
	mob2->SetVelocityAndAcceleration(Vector(30, 00.00, 00.00),
			Vector(15, 00.00, 00.00));
	std::cout << "Node " << "1" << " velocity: " << mob2->GetVelocity()
			<< std::endl;


	TypeId tid = TypeId::LookupByName("ns3::WsmpSocketFactory");
	Ptr<WsmpSocket> recvSink = WsmpSocket::CreateSocket(nodes.Get(0), tid);
	Psid psid = Psid(0x80, 0x01);
	recvSink->Bind(psid);
	recvSink->SetRecvCallback(MakeCallback(&ReceivePacket));

	Ptr<WsmpSocket> source = WsmpSocket::CreateSocket(nodes.Get(1), tid);
	source->Connect(Mac48Address::GetBroadcast(), psid);
	for (int j = 1; j < 10; j += j) {
		Simulator::ScheduleWithContext(nodes.Get(1)->GetId(), Seconds(j),
				&GenerateTraffic, source, packetSize, numPackets,
				interPacketInterval);
	}
	AnimationInterface anim("Crosslayer.xml");
	anim.UpdateNodeSize(1, 40, 20);
	anim.UpdateNodeSize(0, 40, 20);
	Simulator::Stop(Seconds(10.0));
	Simulator::Run();
	Simulator::Destroy();

	return 0;
}

