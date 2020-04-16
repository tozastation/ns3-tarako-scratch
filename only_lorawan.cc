/*
 * This script simulates a simple network in which one end device sends one
 * packet to the gateway.
 */

#include "ns3/csv.h"
#include "ns3/garbage_station.h"

#include "ns3/end-device-lora-phy.h"
#include "ns3/gateway-lora-phy.h"
#include "ns3/class-a-end-device-lorawan-mac.h"
#include "ns3/gateway-lorawan-mac.h"
#include "ns3/simulator.h"
#include "ns3/log.h"
#include "ns3/constant-position-mobility-model.h"
#include "ns3/lora-helper.h"
#include "ns3/mobility-helper.h"
#include "ns3/node-container.h"
#include "ns3/position-allocator.h"
#include "ns3/periodic-sender-helper.h"
#include "ns3/command-line.h"
#include "ns3/basic-energy-source-helper.h"
#include "ns3/lora-radio-energy-model-helper.h"
#include "ns3/file-helper.h"
#include "ns3/names.h"
#include "ns3/config.h"
#include "ns3/string.h"
#include "ns3/core-module.h"
#include "ns3/trace-helper.h"

#include <algorithm>
#include <ctime>
#include <iostream>
#include <fstream>
#include <string>
#include <vector>
#include <exception>
#include <limits>
#include <stdio.h>

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

Ptr<OutputStreamWrapper> lorawan_energy_stream;

static void EnergyEfficiencyTracer(Ptr<OutputStreamWrapper>stream, Time val)
{
    *stream->GetStream () << Simulator::Now ().GetSeconds () << val << std::endl;
}

void
MonitorLoRaWANEnergy(int32_t nodeId, Ptr<OutputStreamWrapper> stream)
{
  Config::ConnectWithoutContext("/NodeList/" + std::to_string(nodeId) + "/$ns3::DoubleProbe/Names/EnergySource/RemainingEnergy", MakeCallback(&EnergyEfficiencyTracer));
}

double convert_string_to_double(std::string value)
{
    return stod(value);
}

bool is_supported_garbage_type(std::string value)
{
    if (value == "○") {
        return true;
    } else {
        return false;
    }
}

NS_LOG_COMPONENT_DEFINE ("OnlyLoRaWANNetworkModel");

int main (int argc, char *argv[])
{
  LogComponentEnable ("OnlyLoRaWANNetworkModel", LOG_LEVEL_ALL);
  LogComponentEnable ("LoraChannel", LOG_LEVEL_INFO);
  LogComponentEnable ("LoraPhy", LOG_LEVEL_ALL);
  LogComponentEnable ("EndDeviceLoraPhy", LOG_LEVEL_ALL);
  LogComponentEnable ("GatewayLoraPhy", LOG_LEVEL_ALL);
  LogComponentEnable ("LoraInterferenceHelper", LOG_LEVEL_ALL);
  LogComponentEnable ("LorawanMac", LOG_LEVEL_ALL);
  LogComponentEnable ("EndDeviceLorawanMac", LOG_LEVEL_ALL);
  LogComponentEnable ("ClassAEndDeviceLorawanMac", LOG_LEVEL_ALL);
  LogComponentEnable ("GatewayLorawanMac", LOG_LEVEL_ALL);
  LogComponentEnable ("LogicalLoraChannelHelper", LOG_LEVEL_ALL);
  LogComponentEnable ("LogicalLoraChannel", LOG_LEVEL_ALL);
  LogComponentEnable ("LoraHelper", LOG_LEVEL_ALL);
  LogComponentEnable ("LoraPhyHelper", LOG_LEVEL_ALL);
  LogComponentEnable ("LorawanMacHelper", LOG_LEVEL_ALL);
  LogComponentEnable ("OneShotSenderHelper", LOG_LEVEL_ALL);
  LogComponentEnable ("OneShotSender", LOG_LEVEL_ALL);
  LogComponentEnable ("LorawanMacHeader", LOG_LEVEL_ALL);
  LogComponentEnable ("LoraFrameHeader", LOG_LEVEL_ALL);
  LogComponentEnableAll (LOG_PREFIX_FUNC);
  LogComponentEnableAll (LOG_PREFIX_NODE);
  LogComponentEnableAll (LOG_PREFIX_TIME);
  /************************
  *  Setup Simulation Area (愛知県東浦町) *
  ************************/
  
  // --- (BEGIN) Read Garbage Station Map from CSV --- //
  const string csv_file = "/home/vagrant/ns-3.30/scratch/test.csv";
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
 
            // cout << "id: " << g_station.id << endl;
            // cout << "longitude: " << fixed << g_station.longitude << endl;
            // cout << "latitude: " << g_station.latitude << endl;
            // cout << "burnable: " << g_station.burnable << endl;
            // cout << "incombustible: " << g_station.incombustible << endl;
            // cout << "resource: " << g_station.resource << endl;

            // cout << endl;

            g_stations.push_back(g_station);
        }
    }
    catch (exception& ex) {
        cout << endl;
        cerr << ex.what() << endl;
        return 1;
    }
    // --- (END) Read Garbage Station Map from CSV --- //

    // --- (BEGIN) NS3 SetUp --- //
    NS_LOG_INFO ("Creating the channel...");
    Ptr<LogDistancePropagationLossModel> loss = CreateObject<LogDistancePropagationLossModel> ();
    loss->SetPathLossExponent (3.76);
    loss->SetReference (1, 7.7);
    Ptr<PropagationDelayModel> delay = CreateObject<ConstantSpeedPropagationDelayModel> ();
    Ptr<LoraChannel> channel = CreateObject<LoraChannel> (loss, delay);
    
    NS_LOG_INFO ("Setting up mobilities...");
    MobilityHelper mobility;
    mobility.SetMobilityModel ("ns3::ConstantPositionMobilityModel");
    
    NS_LOG_INFO ("Setting up helpers...");
    LoraPhyHelper phyHelper = LoraPhyHelper ();
    phyHelper.SetChannel (channel);
    LorawanMacHelper macHelper = LorawanMacHelper ();
    LoraHelper helper = LoraHelper ();
    
    NS_LOG_INFO ("Creating the end device...");
    NodeContainer endDevices;
    Ptr<ListPositionAllocator> allocator = CreateObject<ListPositionAllocator> ();
    int cnt_node = 0;
    for (const auto& g_station: g_stations) {
        if (g_station.burnable) {
            allocator->Add (Vector (g_station.latitude, g_station.longitude,0));
            cnt_node++;
        }
        if (g_station.incombustible) {
            allocator->Add (Vector (g_station.latitude + 0.0001, g_station.longitude,0));
            cnt_node++;
        }
        if (g_station.resource) {
            allocator->Add (Vector (g_station.latitude + 0.0002, g_station.longitude,0));
            cnt_node++;
        }
    }
    //allocator->Add (Vector (34.969392, 136.962864,0));
    mobility.SetPositionAllocator (allocator);
    cout << "info: Node Numver is " << cnt_node << endl;
    endDevices.Create (cnt_node);
    //endDevices.Create (1);
    mobility.Install (endDevices);

    phyHelper.SetDeviceType (LoraPhyHelper::ED);
    macHelper.SetDeviceType (LorawanMacHelper::ED_A);
    NetDeviceContainer endDevicesNetDevices = helper.Install (phyHelper, macHelper, endDevices);

    for (NodeContainer::Iterator j = endDevices.Begin (); j != endDevices.End (); ++j)
    {
      Ptr<Node> node = *j;
      Ptr<LoraNetDevice> loraNetDevice = node->GetDevice (0)->GetObject<LoraNetDevice> ();
      Ptr<LoraPhy> phy = loraNetDevice->GetPhy ();
    }
    NS_LOG_INFO ("Creating the gateway...");
    NodeContainer gateways;
    gateways.Create (3);
    Ptr<ListPositionAllocator> gw_allocator = CreateObject<ListPositionAllocator> ();
    gw_allocator->Add (Vector (34.969392, 136.924615, 15.0));
    gw_allocator->Add (Vector (34.953981, 136.962864, 15.0));
    gw_allocator->Add (Vector (34.973183, 136.967018, 15.0));
    mobility.SetPositionAllocator (gw_allocator);
    mobility.Install (gateways);
    phyHelper.SetDeviceType (LoraPhyHelper::GW);
    macHelper.SetDeviceType (LorawanMacHelper::GW);
    helper.Install (phyHelper, macHelper, gateways);
    macHelper.SetSpreadingFactorsUp (endDevices, gateways, channel);

    PeriodicSenderHelper periodicSenderHelper;
    periodicSenderHelper.SetPeriod (Seconds (5));
    periodicSenderHelper.Install (endDevices);

    BasicEnergySourceHelper basicSourceHelper;
    LoraRadioEnergyModelHelper radioEnergyHelper;

    // configure energy source
    basicSourceHelper.Set ("BasicEnergySourceInitialEnergyJ", DoubleValue (10000)); // Energy in J
    basicSourceHelper.Set ("BasicEnergySupplyVoltageV", DoubleValue (3.3));
    radioEnergyHelper.Set ("StandbyCurrentA", DoubleValue (0.0014));
    radioEnergyHelper.Set ("TxCurrentA", DoubleValue (0.028));
    radioEnergyHelper.Set ("SleepCurrentA", DoubleValue (0.0000015));
    radioEnergyHelper.Set ("RxCurrentA", DoubleValue (0.0112));
    radioEnergyHelper.SetTxCurrentModel ("ns3::ConstantLoraTxCurrentModel","TxCurrent", DoubleValue (0.028));
    // install source on EDs' nodes
    EnergySourceContainer sources = basicSourceHelper.Install (endDevices);
    Names::Add ("/Names/EnergySource", sources.Get (0));
    DeviceEnergyModelContainer deviceModels = radioEnergyHelper.Install(endDevicesNetDevices, sources);
    /**************
    * Get output *
    **************/
    FileHelper fileHelper;
    fileHelper.ConfigureFile ("battery-level", FileAggregator::SPACE_SEPARATED);
    fileHelper.WriteProbe ("ns3::DoubleProbe", "/Names/EnergySource/RemainingEnergy", "Output");

    std::string energy_efficiency_file = "energy_efficiency.csv";
    AsciiTraceHelper ascii;
    Ptr<OutputStreamWrapper> stream = ascii.CreateFileStream(energy_efficiency_file);
    Simulator::Schedule(Seconds(0.0), &MonitorLoRaWANEnergy, endDevices.Get(0)->GetId(), stream);
    /****************
    *  Simulation  *
    ****************/
    Simulator::Stop (Seconds(1));
    Simulator::Run ();
    Simulator::Destroy ();
    return 0;
}
