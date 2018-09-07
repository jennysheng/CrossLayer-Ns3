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
 *
 * Author: Jenny sheng <qdsheng@gmail.com>
 *
 *
 *This project simulate the PHY-MAC-NET interaction cross layer design to increase throughtput.
 *Find the optimal transmission power when comparing throughtput and PDR to avoid congestion.
 *

 *Physical layer parameters
 *1.the transmission power, data rate
 *2.Channel capacity, PDR, BitMeterErrorRate, Propagation loss-> no channel switching because of the emergency bsm
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

#include <bits/stdint-intn.h>
#include <bits/stdint-uintn.h>
#include <cstdio>
#include <fstream>
#include <iostream>
#include <sstream>
#include <string>
#include <vector>

#include "../../build/ns3/address.h"
#include "../../build/ns3/animation-interface.h"
#include "../../build/ns3/application-container.h"
#include "../../build/ns3/callback.h"
#include "../../build/ns3/channel-manager.h"
#include "../../build/ns3/channel-scheduler.h"
#include "../../build/ns3/config.h"
#include "../../build/ns3/constant-acceleration-mobility-model.h"
#include "../../build/ns3/double.h"
#include "../../build/ns3/event-id.h"
#include "../../build/ns3/event-impl.h"
#include "../../build/ns3/inet-socket-address.h"
#include "../../build/ns3/int64x64-128.h"
#include "../../build/ns3/ipv4-address.h"
#include "../../build/ns3/ipv4-interface-container.h"
#include "../../build/ns3/log.h"
#include "../../build/ns3/log-macros-disabled.h"
#include "../../build/ns3/mac48-address.h"
#include "../../build/ns3/mobility-helper.h"
#include "../../build/ns3/net-device-container.h"
#include "../../build/ns3/node.h"
#include "../../build/ns3/node-container.h"
#include "../../build/ns3/nstime.h"
#include "../../build/ns3/object.h"
#include "../../build/ns3/object-factory.h"
#include "../../build/ns3/on-off-helper.h"
#include "../../build/ns3/output-stream-wrapper.h"
#include "../../build/ns3/packet.h"
#include "../../build/ns3/position-allocator.h"
#include "../../build/ns3/ptr.h"
#include "../../build/ns3/random-variable-stream.h"
#include "../../build/ns3/simulator.h"
#include "../../build/ns3/socket.h"
#include "../../build/ns3/string.h"
#include "../../build/ns3/trace-helper.h"
#include "../../build/ns3/type-id.h"
#include "../../build/ns3/uinteger.h"
#include "../../build/ns3/vector.h"
#include "../../build/ns3/wave-bsm-helper.h"
#include "../../build/ns3/wave-bsm-stats.h"
#include "../../build/ns3/wave-helper.h"
#include "../../build/ns3/wave-mac-helper.h"
#include "../../build/ns3/wave-net-device.h"
#include "../../build/ns3/wifi-helper.h"
#include "../../build/ns3/wifi-mode.h"
#include "../../build/ns3/wsmp-socket.h"
#include "../../build/ns3/yans-wifi-helper.h"

using namespace ns3;

NS_LOG_COMPONENT_DEFINE("Wave-Crosslayer-Wsmp");

int BasSpeed = 10;
//the transmission power is referred to the sending node, if more
//than two nodes then the m_txp has to be assigned to the sender node
//according to the node number.
int m_txp; ///< distance
int MaxNodeNbr = 2;
int speedChangeCoef = 20;
Ptr<ConstantAccelerationMobilityModel> mob;
int simTime = 11;
double m_waveInterval = 1; ///< seconds
int currentSpeed;
int m_cumulativeBsmCaptureStart=0; ///< capture start
//10 get set method-------
uint32_t m_phyTxPkts; ///< phy transmit packets
uint32_t m_phyTxBytes; ///< phy transmit bytes
uint32_t m_RxBytes; ///< receive bytes
uint32_t m_RxPkts; ///< receive packets
uint32_t m_TxBytes; ///< transmit bytes
uint32_t m_TxPkts; ///< transmit packets
std::string m_fileName="Wave-Crosslayer-Wsmp"; ///< CSV file name
YansWavePhyHelper wavePhy;
WaveBsmHelper m_waveBsmHelper; ///< helper
Ipv4InterfaceContainer m_adhocTxInterfaces; ///< adhoc transmit interfaces
int m_gpsAccuracyNs=10;
std::vector <double>  m_txSafetyRanges;
int chAccessMode =0;
int m_txMaxDelayMs=10;
/* OfdmRate3MbpsBW10MHz
  OfdmRate4_5MbpsBW10MHz
  OfdmRate6MbpsBW10MHz
  OfdmRate9MbpsBW10MHz
  OfdmRate12MbpsBW10MHz
  OfdmRate18MbpsBW10MHz
  OfdmRate24MbpsBW10MHz
  OfdmRate27MbpsBW10MHz*/
//std::string phyMode("OfdmRate6MbpsBW10MHz");
std::string phyMode("OfdmRate27MbpsBW10MHz");
uint32_t pktSize = 320; // bytes
uint32_t pktNbr = 10; //



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

void ReceivePacket(Ptr<WsmpSocket> socket) {
	Ptr<Packet> packet;
	while (packet = socket->Recv()) {
		m_RxPkts += 1;
		// application data, for goodput
		uint32_t RxRoutingBytes = packet->GetSize();
		m_waveBsmHelper.GetWaveBsmStats()->IncTxByteCount(RxRoutingBytes);
		m_waveBsmHelper.GetWaveBsmStats()->IncRxPktCount();
		//NS_LOG_UNCOND("Received one packet!");
	}
}

int GetPhyTxPkt() {
	return m_phyTxPkts;
}
int GetPhyTxByte() {
	return m_phyTxBytes;
}
void PhyTxTrace(std::string context, Ptr<const Packet> packet, WifiMode mode,
		uint8_t txp) {
	m_txp=txp;
	NS_LOG_FUNCTION(context << packet << "PHYTX mode=" << mode);
	++m_phyTxPkts;
	uint32_t pktSize = packet->GetSize();
	m_phyTxBytes += pktSize;
	NS_LOG_UNCOND("Received PHY size=" << pktSize);
}

void CheckThroughput() {

	uint32_t packetsReceived =	m_waveBsmHelper.GetWaveBsmStats()->GetRxPktCount();

	uint32_t bytesTotal = packetsReceived * pktSize;
	double kbps = (bytesTotal * 8.0) / 1000;
	double wavePDR;
	int wavePktsSent = m_waveBsmHelper.GetWaveBsmStats()->GetTxPktCount();
	if (wavePktsSent > 0) {
		wavePDR = (double) packetsReceived / (double) wavePktsSent;
	}
	// calculate MAC/PHY overhead (mac-phy-oh)
	// total WAVE BSM bytes sent
	uint32_t cumulativeWaveBsmBytes =
			m_waveBsmHelper.GetWaveBsmStats()->GetTxByteCount();
//we don't have routing data here--------------
	uint32_t totalAppBytes = cumulativeWaveBsmBytes;
	uint32_t totalPhyBytes = GetPhyTxByte();
	// mac-phy-oh = (total-phy-bytes - total-app-bytes) / total-phy-bytes
	double mac_phy_oh = 8.0;//Can buss message 8 bytes
	if (totalPhyBytes > 0) {
		mac_phy_oh = (double) (totalPhyBytes - totalAppBytes)
				/ (double) totalPhyBytes;
		NS_LOG_UNCOND("totalPhyBytes"<<totalPhyBytes);
		NS_LOG_UNCOND("totalAppBytes"<<totalAppBytes);
	}

	NS_LOG_UNCOND(
			"t=" << (Simulator::Now ()).GetSeconds () << "s PDR=" << wavePDR
			<< ","<< " packetsReceived=" <<packetsReceived
			<< ","<<" m_txp="<<m_txp << ","
			<< " Goodput=" << kbps << "Kbps" << ","
			<<"MacPhyOh="<<mac_phy_oh /*<< " MAC/PHY-OH=" << mac_phy_oh*/);

	m_waveBsmHelper.GetWaveBsmStats()->ResetTotalRxPktCounts(0);
	m_waveBsmHelper.GetWaveBsmStats()->SetRxPktCount(0);
	m_waveBsmHelper.GetWaveBsmStats()->SetTxPktCount(0);
	for (int index = 1; index <= 10; index++) {
		m_waveBsmHelper.GetWaveBsmStats()->SetExpectedRxPktCount(index, 0);
		m_waveBsmHelper.GetWaveBsmStats()->SetRxPktInRangeCount(index, 0);
	}

	double currentTime = (Simulator::Now()).GetSeconds();
	if (currentTime <= (double) m_cumulativeBsmCaptureStart) {
		for (int index = 1; index <= 10; index++) {
			m_waveBsmHelper.GetWaveBsmStats()->ResetTotalRxPktCounts(index);
		}
	}
Simulator::Schedule(Seconds(1.0), &CheckThroughput);
}

//-------------------------------------------------------------------------
void run(int txp) {

	m_txp = txp;
//create nodes---------------------------------------------------------
	NodeContainer nodes;
	nodes.Create(MaxNodeNbr);
	MobilityHelper mobility;
	int64_t streamIndex = 0; // used to get consistent mobility across scenarios
	ObjectFactory pos;
	pos.SetTypeId("ns3::GridPositionAllocator");
	mobility.SetPositionAllocator("ns3::GridPositionAllocator", "MinX",
			DoubleValue(00.0), "MinY", DoubleValue(100.0), "DeltaX",
			DoubleValue(135.0), "DeltaY", DoubleValue(00.0), "GridWidth",
			UintegerValue(2), "LayoutType", StringValue("ColumnFirst"));

	Ptr<PositionAllocator> taPositionAlloc = pos.Create()->GetObject<
			PositionAllocator>();

	streamIndex += taPositionAlloc->AssignStreams(streamIndex);
	mobility.SetMobilityModel("ns3::ConstantAccelerationMobilityModel");
	mobility.Install(nodes);
	streamIndex += mobility.AssignStreams(nodes, streamIndex);

	  // fix random number streams
	streamIndex += m_waveBsmHelper.AssignStreams (nodes, streamIndex);

	for (int i = 0; i < MaxNodeNbr; i++) {
		mob = nodes.Get(i)->GetObject<ConstantAccelerationMobilityModel>();
		int currentAcc = BasSpeed + (i * speedChangeCoef);
		currentSpeed = BasSpeed - (i * speedChangeCoef);
		mob->SetVelocityAndAcceleration(Vector(currentSpeed, 00.00, 00.00),
				Vector(currentAcc, 00.00, 00.00));
		std::cout << "Node " << i << " velocity: " << mob->GetVelocity()<< std::endl;
		std::cout << "Node " << i << " acceleration: " << currentAcc << std::endl;

	}

//End create nodes----------------------------------------------------
//channel---------------------------------------

	YansWifiChannelHelper waveChannel = YansWifiChannelHelper::Default();
	waveChannel.SetPropagationDelay("ns3::ConstantSpeedPropagationDelayModel");
	waveChannel.AddPropagationLoss("ns3::FriisPropagationLossModel");
	 // waveChannel.AddPropagationLoss ("ns3::NakagamiPropagationLossModel");
	wavePhy = YansWavePhyHelper::Default();
	wavePhy.SetChannel(waveChannel.Create());
	wavePhy.SetPcapDataLinkType(YansWifiPhyHelper::DLT_IEEE802_11);
	wavePhy.Set("TxPowerStart", DoubleValue(txp));
	wavePhy.Set("TxPowerEnd", DoubleValue(txp));

	// every device will have PHY callback for tracing
	// which is used to determine the total amount of
	// data transmitted, and then used to calculate
	// the MAC/PHY overhead beyond the app-data
	Config::Connect("/NodeList/*/DeviceList/*/Phy/State/Tx",
			MakeCallback(&PhyTxTrace));

	QosWaveMacHelper waveMac = QosWaveMacHelper::Default();
	WaveHelper waveHelper = WaveHelper::Default();
	waveHelper.SetRemoteStationManager("ns3::ConstantRateWifiManager",
			"DataMode", StringValue(phyMode), "ControlMode",
			StringValue(phyMode));

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
	wavePhy.EnablePcapAll("wave-jenny-wsmp");

//end channel--------------------------------------------------------------------------
	// Convert to time object
	Time interPacketInterval = Seconds(m_waveInterval);

//create sockets for all the nodes as receiver-----------------------------------------

	TypeId tid = TypeId::LookupByName("ns3::WsmpSocketFactory");
	for (int i = 0; i < MaxNodeNbr; i++) {
		Ptr<WsmpSocket> recvSink = WsmpSocket::CreateSocket(nodes.Get(i), tid);	//original is node 0
		Psid psid = Psid(0x80, 0x01);
		recvSink->Bind(psid);
		recvSink->SetRecvCallback(MakeCallback(&ReceivePacket));
		//create source for all the nodes as sender---------------------------------------------
		Ptr<WsmpSocket> source = WsmpSocket::CreateSocket(nodes.Get(i), tid);//original is node 1
		source->Connect(Mac48Address::GetBroadcast(), psid);
		for (int j = 1; j < simTime; j += j) {
			Simulator::ScheduleWithContext(nodes.Get(i)->GetId(), Seconds(j),
					&GenerateTraffic, source, pktSize, pktNbr,
					interPacketInterval);
		}

	}

//vanet routing compare line 2345---
	AsciiTraceHelper ascii;
	Ptr<OutputStreamWrapper> osw = ascii.CreateFileStream( m_fileName +".mob");
	wavePhy.EnableAsciiAll(osw);

	// calculate and output final results
	/**
	 * \brief Installs BSM generation on devices for nodes
	 * and their interfaces
	 * \param i IPv4 interface container
	 * \param totalTime total amount of time that BSM packets should be transmitted
	 * \param wavePacketSize the size, in bytes, of a WAVE BSM
	 * \param waveInterval the time, in seconds, between each WAVE BSM transmission,
	 * typically 10 Hz (0.1 second)
	 * \param gpsAccuracyNs the timing synchronization accuracy of GPS time, in nanoseconds.
	 * GPS time-sync is ~40-100 ns.  Universally synchronized time among all vehicles
	 * will result in all vehicles transmitting safety messages simultaneously, leading
	 * to excessive wireless collisions.
	 * \param ranges the expected transmission range, in m.
	 * \param chAccessMode channel access mode (0=continuous; 1=switching)
	 * \param txMaxDelay max delay prior to transmit
	 * \return none
	 */

	  m_waveBsmHelper.Install (m_adhocTxInterfaces,
	                           Seconds (simTime),
	                           pktSize,
	                           Seconds (m_waveInterval),
	                           // GPS accuracy (i.e, clock drift), in number of ns
	                           m_gpsAccuracyNs,
	                           m_txSafetyRanges,
	                           chAccessMode,
	                           // tx max delay before transmit, in ms
	                           MilliSeconds (m_txMaxDelayMs));
	// fix random number streams
	streamIndex += m_waveBsmHelper.AssignStreams(nodes, streamIndex);

//	Throughput is the rate at which data is traversing a link.The difference between Goodput and
//throughput is that throughput is the measurement of all data flowing through a link whether
//it is useful data or not, while goodput is focused on useful data only.

	CheckThroughput();
	AnimationInterface anim("Crosslayer.xml");
	for (int i = 0; i < MaxNodeNbr; i++) {
		anim.UpdateNodeSize(i, 50, 50);
	}
	Simulator::Stop(Seconds(simTime));
	Simulator::Run();
	Simulator::Destroy();

}
int main(int argc, char *argv[]) {
	Packet::EnablePrinting();
	std::cout << ("\n Enter a transmission power: ") << std::endl;
	scanf("%d", &m_txp);
	run(m_txp);

	return 0;
}

