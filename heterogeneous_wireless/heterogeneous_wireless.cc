/*
 * This script simulates a simple network in which one end device sends one
 * packet to the gateway.
 */

/* Custom Tarako Module */
#include "ns3/const.h"
#include "ns3/util.h"
#include "ns3/tracer.h"
#include "ns3/node_payload.h"
#include "ns3/logger.h"
#include "ns3/garbage_station.h"
#include "ns3/group_node.h"

#include "ns3/simulator.h"
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
#include "ns3/basic-energy-source-helper.h"
#include "ns3/lora-radio-energy-model-helper.h"
#include "ns3/file-helper.h"
#include "ns3/trace-helper.h"
#include "ns3/forwarder-helper.h"
#include "ns3/network-server-helper.h"

#include "ns3/ble-helper.h"
#include <ns3/lr-wpan-net-device.h>
#include <ns3/propagation-loss-model.h>
#include <ns3/propagation-delay-model.h>
#include <ns3/single-model-spectrum-channel.h>
#include "ns3/lr-wpan-helper.h"
#include "ns3/lr-wpan-mac-header.h"

#include <algorithm>
#include <string>
#include <vector>
#include <unordered_map> 

// using namespace std;
using namespace ns3;
using namespace lorawan;

// --- Global Object --- //
tarako::TarakoLogger tarako_logger;
// --- LoRaWAN Gateway, LoRaWAN & BLE End Device --- //
MobilityHelper mobility_gw, mobility_ed;
Ptr<ListPositionAllocator> ed_allocator = CreateObject<ListPositionAllocator> ();
Ptr<ListPositionAllocator> gw_allocator = CreateObject<ListPositionAllocator> ();
NodeContainer end_devices, gateways, network_servers;
ApplicationContainer lora_network_apps;
NetDeviceContainer ble_net_devices;

Ptr<LogDistancePropagationLossModel> loss = CreateObject<LogDistancePropagationLossModel> ();
Ptr<UniformRandomVariable> x = CreateObject<UniformRandomVariable> ();
Ptr<RandomPropagationLossModel> randomLoss = CreateObject<RandomPropagationLossModel> ();
Ptr<PropagationDelayModel> delay = CreateObject<ConstantSpeedPropagationDelayModel> ();

LoraPhyHelper lora_phy_helper = LoraPhyHelper ();
LorawanMacHelper lora_mac_helper = LorawanMacHelper ();
LoraHelper lora_helper = LoraHelper ();
BasicEnergySourceHelper basic_src_helper;
LoraRadioEnergyModelHelper radio_energy_helper;
NetworkServerHelper network_server_helper;
ForwarderHelper forwarderHelper;

BleHelper ble_helper;
LrWpanHelper lr_wpan_helper;
// --- Global Const Variable --- //
const std::string GARBAGE_BOX_MAP_FILE = "/home/vagrant/workspace/tozastation/ns-3.30/scratch/test_copy.csv";
const int LORAWAN_GATEWAY_NUM          = 3;
const int LORAWAN_NETWORK_SERVER_NUM   = 1;
const double BLE_RX_POWER              = 0.003;
const double BLE_TX_POWER              = 0.003;
// --- Global Variable --- //
int cnt_node = 0;
std::vector<std::vector<std::tuple<int, bool>>> available_connections;
std::vector<tarako::TarakoNodeData> tarako_nodes;
std::unordered_map<int, tarako::TarakoNodeData> trace_node_data_map;

NS_LOG_COMPONENT_DEFINE ("HeterogeneousWirelessNetworkModel");

int main (int argc, char *argv[])
{
    // --- Logging --- //
    LogComponentEnable ("HeterogeneousWirelessNetworkModel", LOG_LEVEL_ALL);
    LogComponentEnable ("TarakoTracer", LOG_LEVEL_ALL);
    LogComponentEnableAll (LOG_PREFIX_FUNC);
    LogComponentEnableAll (LOG_PREFIX_NODE);
    LogComponentEnableAll (LOG_PREFIX_TIME);
    // --- Set the EDs to require Data Rate control from the NS --- //
    Config::SetDefault ("ns3::EndDeviceLorawanMac::DRControl", BooleanValue (true));
    PacketMetadata::Enable();
    // lr_wpan_helper.EnableLogComponents();
    // [CSV READ] Garbageå Box
    const auto garbage_boxes = tarako::TarakoUtil::GetGarbageBox(GARBAGE_BOX_MAP_FILE);  
    // [ADD] garbage boxes geolocation in allocator 
    for (const auto& g_box: garbage_boxes) {
        std::vector<std::tuple<int, bool>> available_connection;
        //std::cout << g_box.burnable << "," << g_box.incombustible << "," << g_box.resource << std::endl;
        if (g_box.burnable) {
            ns3::Vector3D pos = Vector (g_box.latitude, g_box.longitude,1);
            ed_allocator->Add (pos);
            
            tarako::TarakoNodeData node;
            node.id = cnt_node;
            node.position = pos;
            tarako_nodes.push_back(node);
            available_connection.push_back({cnt_node, false});
            cnt_node++;
        }
        if (g_box.incombustible) {
            ns3::Vector3D pos = Vector (g_box.latitude + 0.000001, g_box.longitude,1);
            ed_allocator->Add(pos);

            tarako::TarakoNodeData node;
            node.id = cnt_node;
            node.position = pos;
            tarako_nodes.push_back(node);
            available_connection.push_back({cnt_node, false});
            cnt_node++;
        }
        if (g_box.resource) {
            ns3::Vector3D pos = Vector (g_box.latitude + 0.000002, g_box.longitude,1);
            ed_allocator->Add(pos);

            tarako::TarakoNodeData node;
            node.id = cnt_node;
            node.position = pos;
            tarako_nodes.push_back(node);
            available_connection.push_back({cnt_node, false});
            cnt_node++;
        }
        // select leader node
        int available_cnt = available_connection.size();
        int leader_node = tarako::TarakoUtil::CreateRandomInt(0, available_cnt-1);
        std::get<1>(available_connection.at(leader_node)) = true;
        // add connection
        available_connections.push_back(available_connection);
    }
    // [INIT] End Devices (LoRaWAN, BLE)
    end_devices.Create(cnt_node);
    mobility_ed.SetMobilityModel ("ns3::ConstantPositionMobilityModel");
    mobility_ed.SetPositionAllocator(ed_allocator);
    mobility_ed.Install (end_devices);
    
    // [INIT] LoRaWAN Gateway
    gateways.Create (LORAWAN_GATEWAY_NUM);
    mobility_gw.SetMobilityModel ("ns3::ConstantPositionMobilityModel");
    gw_allocator->Add (Vector (34.969392, 136.924615, 15.0));
    gw_allocator->Add (Vector (34.953981, 136.962864, 15.0));
    gw_allocator->Add (Vector (34.973183, 136.967018, 15.0));
    mobility_gw.SetPositionAllocator (gw_allocator);
    mobility_gw.Install (gateways);
    mobility_gw.SetPositionAllocator (gw_allocator);
    mobility_gw.Install (gateways);

    // [INIT] LoRaWAN Setting
    // -- Channel --- //
    loss->SetPathLossExponent (3.76);
    loss->SetReference (1, 7.7);
    x->SetAttribute ("Min", DoubleValue (0.0));
    x->SetAttribute ("Max", DoubleValue (10));
    randomLoss->SetAttribute ("Variable", PointerValue (x));
    loss->SetNext (randomLoss);
    Ptr<LoraChannel> channel = CreateObject<LoraChannel> (loss, delay);
    // --- Helper --- //
    lora_phy_helper.SetChannel (channel);
    lora_helper.EnablePacketTracking();
    // --- Install Helper to EndDevices ---
    uint8_t nwk_id = 54;
    uint32_t nwk_addr = 1864;
    Ptr<LoraDeviceAddressGenerator> addr_gen = CreateObject<LoraDeviceAddressGenerator> (nwk_id,nwk_addr);
    lora_phy_helper.SetDeviceType (LoraPhyHelper::ED);
    lora_mac_helper.SetDeviceType (LorawanMacHelper::ED_A);
    lora_mac_helper.SetAddressGenerator (addr_gen);
    lora_mac_helper.SetRegion (LorawanMacHelper::EU);
    NetDeviceContainer ed_net_devices = lora_helper.Install (lora_phy_helper, lora_mac_helper, end_devices);
    // --- Install Helper to Gateway --- //
    lora_phy_helper.SetDeviceType (LoraPhyHelper::GW);
    lora_mac_helper.SetDeviceType (LorawanMacHelper::GW);
    NetDeviceContainer gw_net_devices = lora_helper.Install (lora_phy_helper, lora_mac_helper, gateways);
    // --- Install EndDevices and Gateway --- //
    lora_mac_helper.SetSpreadingFactorsUp (end_devices, gateways, channel);
    // --- Install Energy Consumption --- //
    basic_src_helper.Set ("BasicEnergySourceInitialEnergyJ", DoubleValue (10000));
    basic_src_helper.Set ("BasicEnergySupplyVoltageV", DoubleValue (3.3));
    radio_energy_helper.Set ("StandbyCurrentA", DoubleValue (0.0014));
    radio_energy_helper.Set ("TxCurrentA", DoubleValue (0.028));
    radio_energy_helper.Set ("SleepCurrentA", DoubleValue (0.0000015));
    radio_energy_helper.Set ("RxCurrentA", DoubleValue (0.0112));
    radio_energy_helper.SetTxCurrentModel ("ns3::ConstantLoraTxCurrentModel","TxCurrent", DoubleValue (0.028));
    EnergySourceContainer lora_energy_sources = basic_src_helper.Install (end_devices);
    Names::Add ("/Names/EnergySource", lora_energy_sources.Get (0));
    DeviceEnergyModelContainer device_energy_models = radio_energy_helper.Install(ed_net_devices, lora_energy_sources);
    // [INIT] LoRaWAN Network Server
    network_servers.Create (LORAWAN_NETWORK_SERVER_NUM);
    network_server_helper.SetGateways (gateways);
    network_server_helper.SetEndDevices (end_devices);
    network_server_helper.EnableAdr (true);
    network_server_helper.SetAdr ("ns3::AdrComponent");
    lora_network_apps = network_server_helper.Install (network_servers);
    // Connect Gateway 
    forwarderHelper.Install (gateways);

    // --- [INIT] BLE Setting --- //
    ble_net_devices = ble_helper.Install(end_devices);
    // Set Adresses
    NS_LOG_INFO("[INFO] Set BLE Device Address");
    for (uint32_t i = 0; i < end_devices.GetN(); i++)
    {
      std::stringstream stream;
      stream << std::hex << i+1;
      std::string s( stream.str());
      while (s.size() < 4)
        s.insert(0,1,'0');
      s.insert(2,1,':');
      char const * buffer = s.c_str();
      tarako_nodes[i].ble_network_addr = buffer;
    }
    // [Declare] Channel
    Ptr<SingleModelSpectrumChannel> lr_wpan_channel = CreateObject<SingleModelSpectrumChannel> ();
    Ptr<LogDistancePropagationLossModel> lr_wpan_prop_model = CreateObject<LogDistancePropagationLossModel> ();
    Ptr<ConstantSpeedPropagationDelayModel> lr_wpan_delay_model = CreateObject<ConstantSpeedPropagationDelayModel> ();
    lr_wpan_channel->AddPropagationLossModel (lr_wpan_prop_model);
    lr_wpan_channel->SetPropagationDelayModel (lr_wpan_delay_model); 

    // --- [INIT] trace_node_data_map && trace_garbage_box_sensor_map --- //
    NS_LOG_INFO("[INIT] init trace_node_data_map && trace_garbage_box_sensor_map");
    for (int i=0; i < (int)ed_net_devices.GetN(); i++) {
        // [Init] object initial value {TarakoNodeData}
        tarako::TarakoNodeData node_data     = tarako_nodes[i];
        node_data.conn_interval              = Minutes(10);
        node_data.total_energy_consumption   = 0;
        node_data.lora_energy_consumption    = 0;
        node_data.ble_energy_consumption     = 0;
        // [Install] net devices {LoraNetDevice, LrWpanNetDevice}
        // --- Setup LoraNetDevice ---
        node_data.lora_net_device   = ed_net_devices.Get(i)->GetObject<LoraNetDevice>();
        node_data.lora_network_addr = node_data.lora_net_device->GetMac()->GetObject<EndDeviceLorawanMac>()->GetDeviceAddress().GetNwkAddr();
        // --- Setup LrWpanNetDevice ---
        Ptr<LrWpanNetDevice> lr_wpan_net_device = CreateObject<LrWpanNetDevice> ();
        lr_wpan_net_device->SetAddress(Mac16Address(tarako_nodes[i].ble_network_addr.c_str()));
        lr_wpan_net_device->SetChannel(lr_wpan_channel);
        end_devices.Get(i)->AddDevice(lr_wpan_net_device);
        node_data.lr_wpan_net_device = lr_wpan_net_device;
        node_data.lr_wpan_net_device->GetMac()->SetMcpsDataConfirmCallback(MakeCallback(&tarako::DataConfirm));
        // [Function] Judge it is multiple nodes and target node vector index
        auto result_is_multiple_node = tarako::TarakoUtil::IsMultipleNode(available_connections, i);
        bool is_multiple_node        = std::get<0>(result_is_multiple_node);
        int target_index             = std::get<1>(result_is_multiple_node);
        // [Function] Register Role {Group Leader, Group Member}
        if (is_multiple_node && tarako::TarakoConst::EnableGrouping) 
        {
            NS_LOG_INFO("(available_connections) size  : " << available_connections.size());
            NS_LOG_INFO("(target index)                : " << target_index);
            auto conn = available_connections.at(target_index);
            for (auto& conn_info: conn)
            {
                int node_id         = std::get<0>(conn_info);
                bool this_is_leader = std::get<1>(conn_info);
                if (this_is_leader) node_data.leader_node_addr = tarako_nodes[node_id].ble_network_addr;
                // [Function] Register nodes {lora net addr, ble net addr} without me
                int me = i;
                if (node_id != me)
                {
                    auto lora_nd        = ed_net_devices.Get(node_id)->GetObject<LoraNetDevice>();
                    auto ble_net_addr   = tarako_nodes[node_id].ble_network_addr;
                    auto lora_net_addr  = lora_nd->GetMac()->GetObject<EndDeviceLorawanMac>()->GetDeviceAddress().GetNwkAddr();
                    node_data.group_node_addrs.push_back({lora_net_addr,ble_net_addr});
                }
                else 
                {
                    // [Function] Judge Am I Which Nodes
                    if (this_is_leader)
                    {
                        node_data.current_status = tarako::TarakoNodeStatus::group_leader;
                        node_data.activate_time  = Seconds(10 * (target_index + 1));
                    }
                    else
                    {
                        node_data.current_status = tarako::TarakoNodeStatus::group_member;
                        node_data.activate_time  = Seconds(12 * (target_index + 1));
                    }
                }
            }
        }
        else if (is_multiple_node)
        {
            node_data.activate_time  = Seconds(10 * (target_index + 1));
            node_data.current_status = tarako::TarakoNodeStatus::only_lorawan;
        }
        else
        {
            node_data.current_status = tarako::TarakoNodeStatus::only_lorawan;
        }
        // [Init] Garbage Box Sensor
        tarako::GarbageBoxSensor garbage_box_sensor;
        garbage_box_sensor.current_volume = 0;
        // Push node_data to hash map
        node_data.sensor = garbage_box_sensor;
        trace_node_data_map[node_data.lora_network_addr] = node_data;
    }
    // --- [Declare] Trace --- //
    NS_LOG_INFO("[TRACE] tarako::OnPacketRecievedAtNetworkServer");
    Ptr<NetworkServer> ns = lora_network_apps.Get(0)->GetObject<NetworkServer>();
    ns->TraceConnectWithoutContext(
        "ReceivedPacket", 
        MakeBoundCallback(&tarako::OnPacketRecievedAtNetworkServerForGroup, &trace_node_data_map)
    );
    NS_LOG_INFO("[TRACE] tarako::OnLoRaWANEnergyConsumptionChange");
    for (auto itr = trace_node_data_map.begin(); itr != trace_node_data_map.end(); ++itr)
    {
        uint index = itr->second.id;
        device_energy_models.Get(index) -> TraceConnectWithoutContext(
            "TotalEnergyConsumption", 
            MakeBoundCallback(
                &tarako::OnLoRaWANEnergyConsumptionChangeForGroup, 
                &trace_node_data_map.at(itr->second.lora_network_addr)
            )
        );
        itr->second.lr_wpan_net_device->GetMac()->SetMcpsDataIndicationCallback(
            MakeBoundCallback(&tarako::DataIndication, &itr->second)
        );
        Simulator::Schedule(
            itr->second.activate_time, 
            &tarako::OnActivateNodeForGroup, 
            &itr->second
        );
    }
    // [Simulation]
    Time simulationTime = Hours(24);
    Simulator::Stop (simulationTime);
    Simulator::Run ();
    Simulator::Destroy ();
    // --- Write Log --- //
    std::string file_prefix    = tarako::TarakoUtil::GetCurrentTimeStamp();
    std::string base_file_name = "";
    if (tarako::TarakoConst::EnableGrouping) base_file_name = "_group_log.csv";
    else base_file_name   = "_lorawan_log.csv";
    std::string file_name = file_prefix + base_file_name;
    const std::string log_file_path = "./scratch/heterogeneous_wireless/" + file_name; 
    AsciiTraceHelper ascii;
    Ptr<OutputStreamWrapper> log_stream = ascii.CreateFileStream(log_file_path); 
    // Write Header
    *log_stream->GetStream() << "id,";
    *log_stream->GetStream() << "pos_x,pos_y,pos_z,";
    *log_stream->GetStream() << "activation_time,connection_interval,";
    *log_stream->GetStream() << "total_energy_consumption,lora_energy_consumption,ble_energy_consumption";
    *log_stream->GetStream() << std::endl;
    double ble_rx = 0;
    double ble_tx = 0;
    double lora_all = 0;
    for (auto itr = trace_node_data_map.begin(); itr != trace_node_data_map.end(); ++itr)
    {
        *log_stream->GetStream() << itr->second.id << ",";
        *log_stream->GetStream() << itr->second.position.x << ",";
        *log_stream->GetStream() << itr->second.position.y << ",";
        *log_stream->GetStream() << itr->second.position.z << ",";
        *log_stream->GetStream() << std::fixed << itr->second.activate_time.GetSeconds() << ",";
        *log_stream->GetStream() << std::fixed << itr->second.conn_interval.GetSeconds() << ",";
        if (tarako::TarakoConst::EnableGrouping)
        {
            ble_rx = itr->second.received_packets_by_ble.size() * BLE_RX_POWER;
            ble_tx = itr->second.sent_packets_by_ble.size() * BLE_TX_POWER;
            itr->second.ble_energy_consumption = ble_tx + ble_rx;
        }
        lora_all = itr->second.lora_energy_consumption;
        itr->second.total_energy_consumption = ble_rx + ble_tx + lora_all;
        // Continue
        *log_stream->GetStream() << itr->second.total_energy_consumption << ",";
        *log_stream->GetStream() << itr->second.lora_energy_consumption << ",";
        *log_stream->GetStream() << itr->second.ble_energy_consumption << std::endl;
        std::cout << itr->second.ToString() << std::endl;
    }
    return 0;
}