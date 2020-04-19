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
#include "ns3/one-shot-sender-helper.h"

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
#include <cstdio>

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
  cout <<std::fixed << "node id: " << node_id << " | " << "new: " << newEnergyConsumption << "mA" << endl;
}

void OnPacketRecieved (Ptr<const Packet> packet) {
    uint8_t *buffer = new uint8_t[packet->GetSize ()];
    packet->CopyData (buffer, packet->GetSize ());
    string message ((char *)buffer);
    cout << message << endl;
}

void OnDataRateChange (uint8_t oldDr, uint8_t newDr)
{
  NS_LOG_DEBUG ("DR" << unsigned(oldDr) << " -> DR" << unsigned(newDr));
}

void OnTxPowerChange (double oldTxPower, double newTxPower)
{
  NS_LOG_DEBUG (oldTxPower << " dBm -> " << newTxPower << " dBm");
}

int main (int argc, char *argv[])
{
    // Logging
    LogComponentEnable ("OnlyLoRaWANNetworkModel", LOG_LEVEL_ALL);
    LogComponentEnableAll (LOG_PREFIX_FUNC);
    LogComponentEnableAll (LOG_PREFIX_NODE);
    LogComponentEnableAll (LOG_PREFIX_TIME);
  
    // Set the EDs to require Data Rate control from the NS
    Config::SetDefault ("ns3::EndDeviceLorawanMac::DRControl", BooleanValue (true));
  
    // Read Garbage Station Map from CSV 
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

    // End Device mobility
    MobilityHelper mobility_ed, mobility_gw;
    
    mobility_ed.SetMobilityModel ("ns3::ConstantPositionMobilityModel");
    mobility_gw.SetMobilityModel ("ns3::ConstantPositionMobilityModel");
    
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

    // Create a simple wireless channel
    NS_LOG_INFO ("--- initialize LoRaWAN channel ---");
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
    // Create the LoraHelper
    LoraHelper helper = LoraHelper ();
    helper.EnablePacketTracking ();

    // --- Create GWs ---
    NodeContainer gateways;
    gateways.Create (3);
    // Install mobility model on fixed gateway
    Ptr<ListPositionAllocator> gw_allocator = CreateObject<ListPositionAllocator> ();
    gw_allocator->Add (Vector (34.969392, 136.924615, 15.0));
    gw_allocator->Add (Vector (34.953981, 136.962864, 15.0));
    gw_allocator->Add (Vector (34.973183, 136.967018, 15.0));
    mobility_gw.SetPositionAllocator (gw_allocator);
    mobility_gw.Install (gateways);
    
    phyHelper.SetDeviceType (LoraPhyHelper::GW);
    macHelper.SetDeviceType (LorawanMacHelper::GW);
    macHelper.SetRegion(LorawanMacHelper::AS923MHz);
    helper.Install (phyHelper, macHelper, gateways);

    // --- Create End Devices ---
    NS_LOG_INFO("create " << cnt_node << " end devices");
    NodeContainer endDevices;
    endDevices.Create(cnt_node);
    // Install mobility model on fixed nodes
    mobility_ed.SetPositionAllocator (allocator);
    mobility_ed.Install (endDevices);

    // Create a LoraDeviceAddressGenerator
    uint8_t nwkId = 54;
    uint32_t nwkAddr = 1864;
    Ptr<LoraDeviceAddressGenerator> addrGen = CreateObject<LoraDeviceAddressGenerator> (nwkId,nwkAddr);
    // Create the LoraNetDevices of the end devices
    phyHelper.SetDeviceType (LoraPhyHelper::ED);
    macHelper.SetDeviceType (LorawanMacHelper::ED_A);
    macHelper.SetAddressGenerator(addrGen);
    macHelper.SetRegion(LorawanMacHelper::AS923MHz);
    NetDeviceContainer endDevicesNetDevices = helper.Install (phyHelper, macHelper, endDevices);
    
    OneShotSenderHelper oneShotSenderHelper;
    oneShotSenderHelper.SetSendTime (Seconds (2));

    oneShotSenderHelper.Install (endDevices);

    // Install applications in EDs
    // int appPeriodSeconds = 10;
    // PeriodicSenderHelper appHelper = PeriodicSenderHelper ();
    // appHelper.SetPeriod (Seconds (appPeriodSeconds));
    // appHelper.SetPacketSize (150);
    // Ptr<RandomVariableStream> rv = CreateObjectWithAttributes<UniformRandomVariable> ("Min", DoubleValue (0), "Max", DoubleValue (10));
    // ApplicationContainer appContainer = appHelper.Install (endDevices);
    // macHelper.SetSpreadingFactorsUp (endDevices, gateways, channel);
    // appContainer.Start (Seconds (0));
    // appContainer.Stop (Seconds(1200));
    
    // for (int i=0; i < (int)endDevicesNetDevices.GetN(); i++) {
    //     Ptr<LoraNetDevice> lora_net_device = endDevicesNetDevices.Get(i)->GetObject<LoraNetDevice>();
    //     Ptr<LorawanMac> lora_mac = lora_net_device->GetMac();
    //     Ptr<EndDeviceLorawanMac> end_device_lora_mac = lora_mac->GetDevice()->GetObject<EndDeviceLorawanMac>();
    //     string payload = "test";
    //     Ptr<Packet> packet = Create<Packet>((u_int8_t*) payload.c_str(), payload.length()+1);
    //     //end_device_lora_mac->postponeTransmission(Seconds(50), packet);
    //     Simulator::Schedule(Seconds(10), &OnMyMacRecieved, lora_net_device, packet);    
    // }

    // iterate our nodes and print their position.
    // for(NodeContainer::Iterator j = endDevices.Begin (); j != endDevices.End (); ++j)
    // {
    //   Ptr<Node> object = *j;
    //   Ptr<MobilityModel> position = object->GetObject<MobilityModel> ();
    //   NS_ASSERT (position != 0);
    //   Vector pos = position->GetPosition ();
    //   std::cout << std::fixed << "x=" << pos.x << ", y=" << pos.y << ", z=" << pos.z << std::endl;
    // }
    
    // --- Create LoRaWAN Energy Consumption ---
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

    // --- Output Log ---
    // FileHelper fileHelper;
    // fileHelper.ConfigureFile ("battery-level", FileAggregator::SPACE_SEPARATED);
    // fileHelper.WriteProbe ("ns3::DoubleProbe", "/Names/EnergySource/RemainingEnergy", "Output");
    
    string energy_efficiency_file = "energy_efficiency.csv";
    AsciiTraceHelper ascii;
    Ptr<OutputStreamWrapper> stream = ascii.CreateFileStream(energy_efficiency_file);

    // --- Create the NS node ---
    NodeContainer network_server;
    network_server.Create (1);
    // Install the NetworkServer application on the network server
    NetworkServerHelper network_server_helper;
    network_server_helper.SetGateways (gateways);
    network_server_helper.SetEndDevices (endDevices);
    network_server_helper.EnableAdr (true);
    network_server_helper.SetAdr ("ns3::AdrComponent");
    ApplicationContainer ns_container = network_server_helper.Install (network_server);
    // Install the Forwarder application on the gateways
    ForwarderHelper forwarderHelper;
    forwarderHelper.Install (gateways);

    // --- Connect out traces ---
    Ptr<NetworkServer> ns = ns_container.Get(0)->GetObject<NetworkServer>();
    ns->TraceConnectWithoutContext("ReceivedPacket", MakeCallback(&OnPacketRecieved));

    vector<NodeInfo> node_infos(cnt_node);
    for (int i=0; i < (int)deviceModels.GetN(); i++) {
        node_infos[i].id = std::to_string(i);
        deviceModels.Get(i) -> TraceConnectWithoutContext(
            "TotalEnergyConsumption", 
            MakeBoundCallback(&OnLoRaWANEnergyConsumptionChange, stream, i, &node_infos[i])
        );
    }

    Config::ConnectWithoutContext ("/NodeList/*/DeviceList/0/$ns3::LoraNetDevice/Mac/$ns3::EndDeviceLorawanMac/TxPower", MakeCallback (&OnTxPowerChange));
    Config::ConnectWithoutContext ("/NodeList/*/DeviceList/0/$ns3::LoraNetDevice/Mac/$ns3::EndDeviceLorawanMac/DataRate", MakeCallback (&OnDataRateChange));

    /****************
    *  Simulation  *
    ****************/
    Time simulationTime = Minutes (11);
    Simulator::Stop (simulationTime);
    Simulator::Run ();
    Simulator::Destroy ();

    for (const auto& info: node_infos) {
        *stream->GetStream () << info.id << "," << info.energy_consumption << endl;
    }
    return 0;
}
