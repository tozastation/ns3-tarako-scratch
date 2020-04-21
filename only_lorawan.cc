/*
 * This script simulates a simple network in which one end device sends one
 * packet to the gateway.
 */

#include "ns3/csv.h"
#include "ns3/garbage_station.h"
#include "ns3/node_info.h"

#include "ns3/end-device-lora-phy.h"
#include "ns3/gateway-lora-phy.h"
#include "ns3/class-a-end-device-lorawan-mac.h"
#include "ns3/gateway-lorawan-mac.h"
#include "ns3/simulator.h"
//#include "ns3/log.h"
#include "ns3/constant-position-mobility-model.h"
#include "ns3/node-container.h"
#include "ns3/position-allocator.h"
#include "ns3/command-line.h"
#include "ns3/names.h"
#include "ns3/config.h"
#include "ns3/string.h"
#include "ns3/core-module.h"

#include "ns3/lora-helper.h"
#include "ns3/mobility-helper.h"
#include "ns3/periodic-sender-helper.h"
#include "ns3/basic-energy-source-helper.h"
#include "ns3/lora-radio-energy-model-helper.h"
#include "ns3/file-helper.h"
#include "ns3/trace-helper.h"
#include "ns3/forwarder-helper.h"
#include "ns3/network-server-helper.h"

#include <algorithm>
#include <ctime>
#include <iostream>
#include <fstream>
#include <string>
#include <vector>
#include <exception>
#include <limits>
#include <stdio.h>
#include <unordered_map> 

using namespace std;
using namespace ns3;
using namespace tarako;
using namespace lorawan;

// ROW const value
const int ID = 0;
const int LONGITUDE = 1;
const int LATITUDE = 2;
const int BURNABLE = 3;
const int INCOMBUSTIBLE = 4;
const int RESOURCE = 5;

const ns3::Time INTERVAL = Minutes(10);

NS_LOG_COMPONENT_DEFINE ("OnlyLoRaWANNetworkModel");

// --- Parse CSV --- //
double convert_string_to_double(std::string value)
{
    return stod(value);
}

bool is_supported_garbage_type(std::string value)
{
    if (value == "○") return true;
    else return false;
}
// --- Trace Callback Fuction --- //
void OnLoRaWANEnergyConsumptionChange (Ptr<OutputStreamWrapper> stream, int node_id, NodeInfo* node, double oldEnergyConsumption, double newEnergyConsumption)
{
  ostringstream oss;
  node->energy_consumption = newEnergyConsumption;
}

void OnPacketRecieved (unordered_map<int, NodeInfo>* node_map, Ptr<Packet const> packet) {
    LorawanMacHeader mHdr;
    LoraFrameHeader fHdr;
    Ptr<Packet> myPacket = packet->Copy ();
    myPacket->RemoveHeader (mHdr);
    myPacket->RemoveHeader (fHdr);
    node_map->at(int(fHdr.GetAddress().GetNwkAddr())).recieved_packets.push_back(myPacket);
    // Print Payload from Recieved Packet
    uint8_t *buffer = new uint8_t[myPacket->GetSize()];
    myPacket->CopyData(buffer, myPacket->GetSize());
    string s = string(buffer, buffer + myPacket->GetSize());
    std::cout << "Received: " << s << std::endl;
    delete buffer;
}

void OnActivate (Ptr<LoraNetDevice> device) 
{
    string msg = "ryou";
    Ptr<Packet> new_packet = Create<Packet>((uint8_t*) msg.c_str(), msg.length());
    device->Send(new_packet);
    Simulator::Schedule(INTERVAL, &OnActivate, device);
}

void OnMacAttached(Ptr<LorawanMac> mac)
{
    cout << "hit" << endl;
    cout << "attached: " << mac->GetObject<EndDeviceLorawanMac>()->GetDeviceAddress().GetNwkAddr() << endl;
}

int main (int argc, char *argv[])
{
    // --- Logging ---
    LogComponentEnable ("OnlyLoRaWANNetworkModel", LOG_LEVEL_ALL);
    LogComponentEnable ("AdrComponent", LOG_LEVEL_ALL);
    LogComponentEnable ("EndDeviceLorawanMac", LOG_LEVEL_ALL);
    LogComponentEnable ("LoraPacketTracker", LOG_LEVEL_ALL);
    LogComponentEnable("GatewayLorawanMac", LOG_LEVEL_ALL);
    LogComponentEnable("NetworkServer", LOG_LEVEL_ALL);
    LogComponentEnableAll (LOG_PREFIX_FUNC);
    LogComponentEnableAll (LOG_PREFIX_NODE);
    LogComponentEnableAll (LOG_PREFIX_TIME);
    
    // Set the EDs to require Data Rate control from the NS
    Config::SetDefault ("ns3::EndDeviceLorawanMac::DRControl", BooleanValue (true));
    PacketMetadata::Enable();

    // ---  Read Garbage Station Map from CSV --- //
    const string csv_file = "/Users/tozastation/workspace/ns-3.30/scratch/test_copy.csv";
    vector<vector<string>> data;
    vector<GarbageStation> g_stations;

    try {
        Csv objCsv(csv_file);
        if (!objCsv.getCsv(data)) {
            cout << "[error] can not read file" << endl;
            return 1;
        }
        for (unsigned int row = 1; row < data.size(); row++) {
            vector<string> rec = data[row];
            GarbageStation g_station;

            g_station.id = rec[ID];
            g_station.latitude = stod(rec[LATITUDE]);
            g_station.longitude = stod(rec[LONGITUDE]);
            g_station.burnable = is_supported_garbage_type(rec[BURNABLE]);
            g_station.incombustible = is_supported_garbage_type(rec[INCOMBUSTIBLE]);
            g_station.resource = is_supported_garbage_type(rec[RESOURCE]);
 
            g_stations.push_back(g_station);
        }
    }
    catch (exception& ex) {
        cout << endl;
        cerr << ex.what() << endl;
        return 1;
    }
    
    // --- Mobility ---
    MobilityHelper mobility_gw, mobility_ed;
    mobility_gw.SetMobilityModel ("ns3::ConstantPositionMobilityModel");
    mobility_ed.SetMobilityModel ("ns3::ConstantPositionMobilityModel");
    // Create Garbage Station Position
    Ptr<ListPositionAllocator> allocator = CreateObject<ListPositionAllocator> ();
    int cnt_node = 0;
    
    for (const auto& g_station: g_stations) {
        if (g_station.burnable) {
            allocator->Add (Vector (g_station.latitude, g_station.longitude,0));
            cnt_node++;
        }
        if (g_station.incombustible) {
            allocator->Add (Vector (g_station.latitude + 0.000001, g_station.longitude,0));
            cnt_node++;
        }
        if (g_station.resource) {
            allocator->Add (Vector (g_station.latitude + 0.000002, g_station.longitude,0));
            cnt_node++;
        }
    }
    // --- End Device ---
    NodeContainer endDevices;
    endDevices.Create(cnt_node);
    // Install Mobility
    mobility_ed.SetPositionAllocator (allocator);
    mobility_ed.Install (endDevices);

    // --- Create Channel ---
    Ptr<LogDistancePropagationLossModel> loss = CreateObject<LogDistancePropagationLossModel> ();
    loss->SetPathLossExponent (3.76);
    loss->SetReference (1, 7.7);
    Ptr<UniformRandomVariable> x = CreateObject<UniformRandomVariable> ();
    x->SetAttribute ("Min", DoubleValue (0.0));
    x->SetAttribute ("Max", DoubleValue (10));
    Ptr<RandomPropagationLossModel> randomLoss = CreateObject<RandomPropagationLossModel> ();
    randomLoss->SetAttribute ("Variable", PointerValue (x));
    loss->SetNext (randomLoss);
    Ptr<PropagationDelayModel> delay = CreateObject<ConstantSpeedPropagationDelayModel> ();
    Ptr<LoraChannel> channel = CreateObject<LoraChannel> (loss, delay);
    // --- Helper ---
    // Create the LoraPhyHelper
    LoraPhyHelper phyHelper = LoraPhyHelper ();
    phyHelper.SetChannel (channel);
    // Create the LorawanMacHelper
    LorawanMacHelper macHelper = LorawanMacHelper ();
    LoraHelper helper = LoraHelper ();
    helper.EnablePacketTracking();
    
    // --- Gateway ---
    NodeContainer gateways;
    gateways.Create (3);
    Ptr<ListPositionAllocator> gw_allocator = CreateObject<ListPositionAllocator> ();
    gw_allocator->Add (Vector (34.969392, 136.924615, 15.0));
    gw_allocator->Add (Vector (34.953981, 136.962864, 15.0));
    gw_allocator->Add (Vector (34.973183, 136.967018, 15.0));
    // Install Mobility
    mobility_gw.SetPositionAllocator (gw_allocator);
    mobility_gw.Install (gateways);
    phyHelper.SetDeviceType (LoraPhyHelper::GW);
    macHelper.SetDeviceType (LorawanMacHelper::GW);
    helper.Install (phyHelper, macHelper, gateways);
    
    // --- End Devices ---
    // Install LoRaWAN Helper
    uint8_t nwkId = 54;
    uint32_t nwkAddr = 1864;
    Ptr<LoraDeviceAddressGenerator> addrGen = CreateObject<LoraDeviceAddressGenerator> (nwkId,nwkAddr);
    
    phyHelper.SetDeviceType (LoraPhyHelper::ED);
    macHelper.SetDeviceType (LorawanMacHelper::ED_A);
    macHelper.SetAddressGenerator (addrGen);
    macHelper.SetRegion (LorawanMacHelper::EU);
    NetDeviceContainer endDevicesNetDevices = helper.Install (phyHelper, macHelper, endDevices);
    macHelper.SetSpreadingFactorsUp (endDevices, gateways, channel);

    // Install Energy Consumption
    BasicEnergySourceHelper basicSourceHelper;
    LoraRadioEnergyModelHelper radioEnergyHelper;

    basicSourceHelper.Set ("BasicEnergySourceInitialEnergyJ", DoubleValue (10000)); // Energy in J
    basicSourceHelper.Set ("BasicEnergySupplyVoltageV", DoubleValue (3.3));
    radioEnergyHelper.Set ("StandbyCurrentA", DoubleValue (0.0014));
    radioEnergyHelper.Set ("TxCurrentA", DoubleValue (0.028));
    radioEnergyHelper.Set ("SleepCurrentA", DoubleValue (0.0000015));
    radioEnergyHelper.Set ("RxCurrentA", DoubleValue (0.0112));
    radioEnergyHelper.SetTxCurrentModel ("ns3::ConstantLoraTxCurrentModel","TxCurrent", DoubleValue (0.028));
    EnergySourceContainer sources = basicSourceHelper.Install (endDevices);
    Names::Add ("/Names/EnergySource", sources.Get (0));
    DeviceEnergyModelContainer deviceModels = radioEnergyHelper.Install(endDevicesNetDevices, sources);

    // FileHelper fileHelper;
    // fileHelper.ConfigureFile ("battery-level", FileAggregator::SPACE_SEPARATED);
    // fileHelper.WriteProbe ("ns3::DoubleProbe", "/Names/EnergySource/RemainingEnergy", "Output");

    // --- Network Server ---
    NodeContainer networkServers;
    networkServers.Create (1);
    NetworkServerHelper networkServerHelper;
    networkServerHelper.SetGateways (gateways);
    networkServerHelper.SetEndDevices (endDevices);
    networkServerHelper.EnableAdr (true);
    networkServerHelper.SetAdr ("ns3::AdrComponent");
    ApplicationContainer nsModels = networkServerHelper.Install (networkServers);
    // Connect Gateway 
    ForwarderHelper forwarderHelper;
    forwarderHelper.Install (gateways);

    // --- Connect out traces ---
    string energy_consumption_file = "./scratch/energy_consumption.csv";
    AsciiTraceHelper ascii;
    Ptr<OutputStreamWrapper> stream = ascii.CreateFileStream(energy_consumption_file);
    // Recieved Packet 
    unordered_map<int, NodeInfo> node_map;
    Ptr<NetworkServer> ns = nsModels.Get(0)->GetObject<NetworkServer>();
    ns->TraceConnectWithoutContext("ReceivedPacket", MakeBoundCallback(&OnPacketRecieved, &node_map));
    // Energy Consumption & Schedule Sending Packet
    for (int i=0; i < (int)endDevicesNetDevices.GetN(); i++) {
        Ptr<LoraNetDevice> lora_net_device = endDevicesNetDevices.Get(i)->GetObject<LoraNetDevice>();
        uint32_t nwk_addr = lora_net_device->GetMac()->GetObject<EndDeviceLorawanMac>()->GetDeviceAddress().GetNwkAddr();
        
        NodeInfo node_info;
        node_info.id = i;
        node_map[int(nwk_addr)] = node_info;
        deviceModels.Get(i) -> TraceConnectWithoutContext(
            "TotalEnergyConsumption", 
            MakeBoundCallback(&OnLoRaWANEnergyConsumptionChange, stream, i, &node_map.at(int(nwk_addr)))
        );
        ns3::Time activate_time = Seconds(60*i+1);
        cout << "scheduled device: " << i << ", activate on: " << activate_time.GetSeconds() << endl;
        Simulator::Schedule(activate_time, &OnActivate, lora_net_device);
    }
    // --- Simulation ---
    Time simulationTime = Hours(24);
    Simulator::Stop (simulationTime);
    Simulator::Run ();
    Simulator::Destroy ();
    // --- Write Log ---
    for (auto itr = node_map.begin(); itr != node_map.end(); ++itr)
    {
        int nwk_addr = itr->first;
        NodeInfo node_info = itr->second;
        *stream->GetStream () << nwk_addr << "," << node_info.energy_consumption << endl;
        cout << "node id: " << nwk_addr  << ", " << "energy_consumption: " << node_info.energy_consumption << ", " << "packet num: " << node_info.recieved_packets.size() << endl;
    }

    LoraPacketTracker &tracker = helper.GetPacketTracker ();
    std::cout << tracker.CountMacPacketsGlobally (Seconds (0), simulationTime + Minutes (1)) << std::endl;

    return 0;
}