/**
 * @file mlo_simulator.cc
 * @brief Multi-Link Operation (MLO) WiFi Simulator for IEEE 802.11be
 * 
 * This comprehensive simulation framework evaluates Multi-Link Operation capabilities
 * in IEEE 802.11be (WiFi 7) networks. The simulator provides:
 * 
 * 1. CORE FUNCTIONALITY:
 *    - Multi-link WiFi network setup with simultaneous operation across multiple bands
 *    - Traffic flow management with different QoS requirements (Emergency, Critical, Best Effort)
 *    - Link quality monitoring and performance analysis
 *    - SLA (Service Level Agreement) compliance tracking
 * 
 * 2. LINK MAPPING STRATEGIES:
 *    - Round-robin traffic distribution
 *    - Reliability-aware link selection
 *    - Load balancing algorithms
 *    - SLA-optimized traffic routing
 * 
 * 3. PERFORMANCE MONITORING:
 *    - Real-time link quality assessment (latency, throughput, packet loss)
 *    - SLA violation detection and reporting
 *    - Comprehensive statistics collection and analysis
 *    - CSV-based result logging for post-simulation analysis
 * 
 * 4. TRAFFIC PATTERNS:
 *    - Enhanced UDP traffic generation with configurable patterns
 *    - TCP traffic simulation with congestion control
 *    - Interference generation for testing robustness
 *    - Mobility support for dynamic network conditions
 * 
 * The simulator is designed for researchers and engineers working on MLO optimization, and next-generation WiFi performance evaluation.
 */

// Standard C++ includes
// Core I/O and data structures
#include <iostream>        // Console input/output operations
#include <map>            // Associative container for key-value pairs
#include <fstream>        // File stream operations for CSV and log output
#include <sstream>        // String stream operations for data formatting
#include <iomanip>        // I/O manipulators for formatted output
#include <memory>         // Smart pointers for automatic memory management
#include <numeric>        // Numerical operations and accumulate functions
#include <mutex>          // Thread synchronization primitives
#include <algorithm>      // Standard algorithms (sort, find, etc.)
#include <deque>          // Double-ended queue for efficient insertion/deletion
#include <functional>     // Function objects and lambda utilities
#include <cctype>         // Character classification functions (toupper, tolower)

// System-level includes
#include <sys/stat.h>     // File system statistics and directory operations
#include <errno.h>        // Error number definitions
#include <ctime>          // Time and date utilities

// Thread safety
#include <mutex>          // Thread synchronization primitives
#include <atomic>         // Atomic operations for thread-safe counters

// NS-3 includes
// Core simulation framework
#include "ns3/command-line.h"              // Command line argument parsing
#include "ns3/simulator.h"                 // Main simulation engine
#include "ns3/config.h"                    // Configuration system for attributes
#include "ns3/rng-seed-manager.h"          // Random number generator seeding
#include "ns3/system-path.h"               // File system path utilities
#include "ns3/attribute-helper.h"          // Attribute value conversion utilities
#include "ns3/object-factory.h"            // Object creation factory
#include "ns3/enum.h"                      // Enumeration attribute support

// Node and network device management
#include "ns3/node-container.h"            // Container for network nodes
#include "ns3/net-device-container.h"      // Container for network devices
#include "ns3/application-container.h"     // Container for applications

// WiFi-specific includes
#include "ns3/wifi-helper.h"               // Main WiFi setup helper
#include "ns3/spectrum-wifi-helper.h"      // Spectrum-based WiFi helper for MLO
#include "ns3/multi-model-spectrum-channel.h" // Multi-band spectrum channel for MLO
#include "ns3/wifi-mac.h"                  // WiFi MAC layer functionality
#include "ns3/ap-wifi-mac.h"               // Access Point MAC implementation
#include "ns3/qos-txop.h"                  // Quality of Service transmission opportunities
#include "ns3/wifi-mac-header.h"           // WiFi MAC header parsing
#include "ns3/wifi-mpdu.h"                 // MAC Protocol Data Unit handling
#include "ns3/wifi-phy.h"                  // WiFi physical layer
#include "ns3/wifi-net-device.h"           // WiFi network device implementation
#include "ns3/eht-phy.h"                   // EHT (802.11be) physical layer
#include "ns3/wifi-acknowledgment.h"       // WiFi acknowledgment mechanisms
#include "ns3/propagation-loss-model.h"    // Radio propagation modeling
#include "ns3/error-model.h"               // Packet error modeling

// Network layer and addressing
#include "ns3/internet-stack-helper.h"     // TCP/IP stack installation
#include "ns3/ipv4-address-helper.h"       // IPv4 address assignment
#include "ns3/ipv4-address.h"              // IPv4 address representation
#include "ns3/ipv4-address-generator.h"    // Automatic IPv4 address generation
#include "ns3/ipv4-global-routing-helper.h" // Global routing protocol
#include "ns3/inet-socket-address.h"       // Internet socket addressing

// Transport layer protocols
#include "ns3/socket.h"                    // Generic socket interface
#include "ns3/udp-socket-factory.h"        // UDP socket creation
#include "ns3/tcp-socket-factory.h"        // TCP socket creation
#include "ns3/tcp-socket-base.h"           // TCP socket base implementation
#include "ns3/tcp-congestion-ops.h"        // TCP congestion control algorithms

// Application layer
#include "ns3/application.h"               // Base application class
#include "ns3/udp-client.h"                // UDP client application
#include "ns3/udp-client-server-helper.h" // UDP client-server setup helper
#include "ns3/packet-sink-helper.h"        // Packet sink application helper
#include "ns3/bulk-send-application.h"     // Bulk data transfer application
#include "ns3/on-off-helper.h"             // On-off traffic pattern application

// Mobility and positioning
#include "ns3/mobility-helper.h"           // Node mobility configuration
#include "ns3/mobility-module.h"           // Mobility model implementations

// Performance monitoring and analysis
#include "ns3/flow-monitor-helper.h"       // Network flow monitoring setup
#include "ns3/flow-monitor.h"              // Flow monitoring implementation
#include "ns3/ipv4-flow-classifier.h"      // IPv4 flow classification
#include "ns3/seq-ts-header.h"             // Sequence number and timestamp headers

// Packet tagging and metadata
#include "ns3/tag.h"                       // Packet tagging system

// Attribute types and utilities
#include "ns3/uinteger.h"                  // Unsigned integer attributes
#include "ns3/boolean.h"                   // Boolean attributes
#include "ns3/double.h"                    // Double-precision float attributes
#include "ns3/string.h"                    // String attributes
#include "ns3/ssid.h"                      // WiFi SSID representation
#include "ns3/random-variable-stream.h"    // Random variable distributions

using namespace ns3;  // Use NS-3 namespace for all simulation components

// ================== CONSTANTS AND CONFIGURATION ==================

// MLO Implementation Constants
namespace MLOConstants {
    // Physical Layer Constants
    const uint32_t MAX_CHANNEL_WIDTH_2_4GHZ = 320;  // EHT supports up to 320MHz in 2.4GHz
    const uint32_t MAX_CHANNEL_WIDTH_5GHZ = 320;    // EHT supports up to 320MHz in 5GHz  
    const uint32_t MAX_CHANNEL_WIDTH_6GHZ = 320;    // EHT supports up to 320MHz in 6GHz
    
    // QoS and TID Constants (IEEE 802.11 Access Categories)
    const uint8_t AC_BK_TID_START = 1;  // Background traffic TIDs 1,2
    const uint8_t AC_BE_TID_START = 0;  // Best Effort traffic TIDs 0,3
    const uint8_t AC_VI_TID_START = 4;  // Video traffic TIDs 4,5
    const uint8_t AC_VO_TID_START = 6;  // Voice traffic TIDs 6,7
    const uint8_t MAX_TID_VALUE = 7;    // Maximum valid TID value
    
    // Performance Measurement Constants  
    const double MIN_REALISTIC_DELAY_MS = 0.5;      // Minimum realistic WiFi delay
    const double DEFAULT_JITTER_THRESHOLD_MS = 10.0; // Default jitter threshold
    const uint32_t PDR_CALCULATION_WINDOW = 100;    // Packets for PDR calculation
    
    // Load Balancing Constants
    const uint32_t LOAD_BALANCER_MAX_ATTEMPTS = 10; // Max retries for load balancing
    const double LINK_QUALITY_WEIGHT_PDR = 0.6;     // PDR weight in quality calculation
    const double LINK_QUALITY_WEIGHT_DELAY = 0.3;   // Delay weight in quality calculation  
    const double LINK_QUALITY_WEIGHT_JITTER = 0.1;  // Jitter weight in quality calculation
    
    // Default MCS and Channel Settings
    const uint32_t DEFAULT_MCS = 3;
    const uint32_t DEFAULT_TCP_SEGMENT_SIZE = 1460;
    const uint32_t DEFAULT_PAYLOAD_SIZE = 1000;
    const uint32_t DEFAULT_CHANNEL_WIDTH = 80;
    
    // Thread Safety Constants
    const uint32_t MAX_CONCURRENT_THREADS = 10;
}

// Thread-safe global variables
std::atomic<uint32_t> g_verbosityLevel{1};
std::mutex g_logMutex;  // Protect logging operations
std::mutex g_metricsMutex;  // Protect metrics updates

// Thread-safe logging function
#define SAFE_LOG_IF(level, message) \
    do { \
        if (g_verbosityLevel >= level) { \
            std::lock_guard<std::mutex> lock(g_logMutex); \
            std::cout << message << std::flush; \
        } \
    } while(0)

// QoS Utility Functions
namespace QoSUtils {
    /**
     * @brief Get IEEE 802.11 Access Category for a given TID
     * @param tid Traffic Identifier (0-7)
     * @return Access Category (AC_BK=0, AC_BE=1, AC_VI=2, AC_VO=3)
     */
    uint8_t GetAccessCategory(uint8_t tid) {
        if (tid > MLOConstants::MAX_TID_VALUE) {
            tid = tid % 8;  // Wrap around to valid TID range
        }
        
        // IEEE 802.11 standard AC mapping
        switch (tid) {
            case 1:
            case 2:
                return 0; // AC_BK (Background)
            case 0:
            case 3:
                return 1; // AC_BE (Best Effort)
            case 4:
            case 5:
                return 2; // AC_VI (Video)
            case 6:
            case 7:
                return 3; // AC_VO (Voice)
            default:
                return 1; // Default to Best Effort
        }
    }
    
    /**
     * @brief Determine if TID represents critical traffic based on AC
     * @param tid Traffic Identifier
     * @param emergencyTids Number of emergency TIDs (highest priority)
     * @param criticalTids Number of critical TIDs  
     * @return true if traffic is critical
     */
    bool IsCriticalTraffic(uint8_t tid, uint32_t emergencyTids, uint32_t criticalTids) {
        // CLEAR PRECEDENCE ORDER:
        // 1. Emergency TIDs (highest priority) - always critical
        if (tid < emergencyTids) {
            return true;
        }
        
        // 2. Critical TIDs (high priority) - always critical
        if (tid < (emergencyTids + criticalTids)) {
            return true;
        }
        
        // 3. For remaining TIDs: Only use AC-based classification if explicitly enabled
        // IMPORTANT: If criticalTids=0, user wants NO critical traffic, so respect that
        // Only fall back to AC-based classification if critical TIDs are configured
        if (emergencyTids == 0 && criticalTids == 0) {
            // User explicitly set no critical TIDs - respect this choice
            // Don't auto-classify based on Access Category
            return false;
        }
        
        // 4. If critical TIDs are configured but this TID isn't in range,
        //    optionally check Access Category (currently disabled for clarity)
        // uint8_t ac = GetAccessCategory(tid);
        // return (ac == 2 || ac == 3); // AC_VI or AC_VO
        
        // 5. All other TIDs are non-critical by default
        return false;
    }
    
    /**
     * @brief Get priority level for scheduling decisions
     * @param tid Traffic Identifier
     * @param emergencyTids Number of emergency TIDs
     * @param criticalTids Number of critical TIDs
     * @return Priority level (0=lowest, 3=highest)
     */
    uint8_t GetPriorityLevel(uint8_t tid, uint32_t emergencyTids, uint32_t criticalTids) {
        if (tid < emergencyTids) {
            return 3; // Emergency - Highest priority
        }
        if (tid < (emergencyTids + criticalTids)) {
            return 2; // Critical - High priority
        }
        
        // Use AC for remaining TIDs
        return GetAccessCategory(tid);
    }
}

// Input Validation Utilities
namespace ValidationUtils {
    /**
     * @brief Comprehensive input parameter validation
     */
    void ValidateSimulationParameters(uint32_t mcs, double simtime, uint32_t nWifi, 
                                     uint32_t tidCount, uint32_t emergencyTids, 
                                     uint32_t criticalTids, double distance,
                                     uint32_t channelWidth, double interferenceIntensity) {
        // MCS validation
        if (mcs > 13) {
            NS_FATAL_ERROR("Invalid MCS value: " << mcs << ". EHT supports MCS 0-13");
        }
        
        // Simulation time validation
        if (simtime <= 0 || simtime > 3600) { // Max 1 hour
            NS_FATAL_ERROR("Invalid simulation time: " << simtime << ". Must be 0 < simtime <= 3600 seconds");
        }
        
        // Node count validation
        if (nWifi == 0 || nWifi > 1000) {
            NS_FATAL_ERROR("Invalid WiFi node count: " << nWifi << ". Must be 1 <= nWifi <= 1000");
        }
        
        // TID validation
        if (tidCount == 0 || tidCount > 256) {
            NS_FATAL_ERROR("Invalid TID count: " << tidCount << ". Must be 1 <= tidCount <= 256");
        }
        
        if (emergencyTids > tidCount) {
            NS_FATAL_ERROR("Emergency TIDs (" << emergencyTids << ") exceed total TID count (" << tidCount << ")");
        }
        
        if (criticalTids > tidCount) {
            NS_FATAL_ERROR("Critical TIDs (" << criticalTids << ") exceed total TID count (" << tidCount << ")");
        }
        
        if ((emergencyTids + criticalTids) > tidCount) {
            NS_FATAL_ERROR("Emergency (" << emergencyTids << ") + Critical (" << criticalTids 
                          << ") TIDs exceed total TID count (" << tidCount << ")");
        }
        
        // Distance validation
        if (distance < 0.1 || distance > 10000.0) { // 10cm to 10km
            NS_FATAL_ERROR("Invalid distance: " << distance << "m. Must be 0.1m <= distance <= 10000m");
        }
        
        // Channel width validation
        if (channelWidth != 20 && channelWidth != 40 && channelWidth != 80 && 
            channelWidth != 160 && channelWidth != 320) {
            NS_FATAL_ERROR("Invalid channel width: " << channelWidth << " MHz. Must be 20, 40, 80, 160, or 320 MHz");
        }
        
        // Interference intensity validation
        if (interferenceIntensity < 0.0 || interferenceIntensity > 1.0) {
            NS_FATAL_ERROR("Invalid interference intensity: " << interferenceIntensity 
                          << ". Must be 0.0 <= intensity <= 1.0");
        }
        
        SAFE_LOG_IF(2, "✅ Input parameters validated successfully\n");
    }
    
    /**
     * @brief Validate TID is within acceptable range
     */
    bool IsValidTid(uint8_t tid, uint32_t maxTids = 256) {
        return tid < maxTids;
    }
    
    /**
     * @brief Validate link ID is within acceptable range
     */
    bool IsValidLinkId(uint8_t linkId, uint8_t numLinks = 3) {
        return linkId < numLinks;
    }
    
    /**
     * @brief Validate delay value is realistic
     */
    bool IsRealisticDelay(double delay) {
        return delay >= 0.0 && delay <= 10000.0; // 0 to 10 seconds max
    }
    
    /**
     * @brief Validate packet size is reasonable
     */
    bool IsValidPacketSize(uint32_t size) {
        return size >= 64 && size <= 65535; // Standard Ethernet frame limits
    }
}

// Forward declarations
// Declare classes before their full definitions to resolve dependencies
class ResultLogger;  // Comprehensive logging and results management system

// Global configuration
/**
 * @brief Global verbosity level for simulation output control
 * 
 * Controls the amount of diagnostic and progress information displayed:
 * - Level 0: Basic essential results only (final metrics, key statistics)
 * - Level 1: More specific information (setup details, SLA validation, progress updates)
 * - Level 2: Detailed report (per-flow stats, link quality metrics, comprehensive analysis)
 */
// Verbosity level now defined as atomic variable above

/**
 * @brief Conditional logging macro based on verbosity level
 * 
 * This macro enables selective output based on the current verbosity setting.
 * Higher-level messages are only displayed when the global verbosity level
 * meets or exceeds the specified threshold.
 * 
 * @param level Minimum verbosity level required to display the message
 * @param stream Output stream content to be displayed
 */
#define LOG_IF(level, stream) \
    if (g_verbosityLevel >= level) { std::cout << stream; }

// Enhanced Result Logger
/**
 * @class ResultLogger
 * @brief Comprehensive logging and results management system for MLO simulations
 * 
 * This class handles all aspects of simulation data collection, processing, and output:
 * 
 * KEY FEATURES:
 * - Real-time performance metric collection and aggregation
 * - CSV file generation with detailed simulation results
 * - Statistical analysis (mean, percentiles, standard deviation)
 * - Multi-scenario result comparison and benchmarking
 * - Automatic directory creation and file management
 * - Thread-safe operation for concurrent data collection
 * 
 * PERFORMANCE METRICS TRACKED:
 * - Network throughput (aggregate and per-link)
 * - Packet Delivery Ratio (PDR) for different traffic classes
 * - End-to-end delay and jitter statistics
 * - SLA deviation measurements for QoS compliance
 * - Link utilization and load balancing effectiveness
 * - Recovery time from network failures or interference
 * 
 * OUTPUT FORMATS:
 * - Detailed CSV files for post-processing and visualization
 * - Console progress indicators and real-time status updates
 * - Statistical summaries with confidence intervals
 * - Comparative analysis across different strategies and configurations
 */
class ResultLogger {
public:
    /**
     * @brief Constructor initializes the logging system
     * 
     * Sets up the output directory structure and initializes CSV file headers.
     * Creates a unique scenario identifier for result organization.
     * 
     * @param scenarioName Optional custom name for the simulation scenario
     * @param customCsvFile Optional custom CSV filename (auto-generated if empty)
     * @param numLinks Number of links for dynamic CSV header generation
     */
    ResultLogger(const std::string& scenarioName = "", const std::string& customCsvFile = "", uint8_t numLinks = 3) 
        : m_scenarioName(scenarioName), m_customCsvFile(customCsvFile), m_numLinks(numLinks) {
        CreateOutputDirectory();  // Ensure output directories exist
        InitializeCsv();         // Set up CSV file with appropriate headers
    }

    /**
     * @brief Initialize all simulation configuration parameters
     * 
     * This method captures the complete simulation setup for later analysis and reporting.
     * It stores both the experimental configuration and initializes performance metric
     * containers with default values that will be updated during simulation execution.
     * 
     * @param strategy Link mapping strategy being evaluated (e.g., "RoundRobin", "SLA_MLO")
     * @param protocol Transport protocol used (TCP/UDP)
     * @param nodeCount Number of client nodes in the simulation
     * @param simTime Total simulation duration in seconds
     * @param tidCount Total number of Traffic Identifiers (TIDs)
     * @param criticalTids Number of TIDs assigned to critical traffic
     * @param distance Physical distance between nodes (affects signal propagation)
     * @param mobility Whether node mobility is enabled
     * @param runNumber Current simulation run number (for statistical significance)
     * @param duplication Whether packet duplication across links is enabled
     * @param interference Whether interference generation is active
     * @param interferencePattern Type of interference pattern applied
     * @param interferenceIntensity Strength of interference (0.0-1.0)
     * @param mobilityPattern Type of mobility model used
     * @param emergencyTids Number of TIDs assigned to emergency traffic
     */
    void InitializeSimulationParameters(const std::string& strategy, const std::string& protocol, 
                                       uint32_t nodeCount, double simTime,
                                       uint32_t tidCount, uint32_t criticalTids, double distance,
                                       bool mobility, uint32_t runNumber, bool duplication, bool interference,
                                       const std::string& interferencePattern = "none",
                                       double interferenceIntensity = 0.0,
                                       const std::string& mobilityPattern = "none",
                                       uint32_t emergencyTids = 0) {
        
        // Store simulation parameters that are known at simulation start
        m_currentStrategy = strategy;
        m_currentProtocol = protocol;
        m_currentNodeCount = nodeCount;
        m_currentSimTime = simTime;
        m_currentTidCount = tidCount;
        m_currentCriticalTids = criticalTids;
        m_currentDistance = distance;
        m_currentMobility = mobility;
        m_currentRunNumber = runNumber;
        m_currentDuplication = duplication;
        m_currentInterference = interference;
        m_currentInterferencePattern = interferencePattern;
        m_currentInterferenceIntensity = interferenceIntensity;
        m_currentMobilityPattern = mobilityPattern;
        m_currentEmergencyTids = emergencyTids;
        
        // Initialize performance metrics with default values (will be updated during simulation)
        m_currentThroughput = 0.0;
        m_currentPdr = 0.0;
        m_currentAvgDelay = 0.0;
        m_currentAvgJitter = 0.0;
        m_currentRecoveryTime = 0.0;
        m_currentCriticalPdr = 0.0;
        m_currentCriticalAvgDelay = 0.0;
        m_currentNonCriticalPdr = 0.0;
        m_currentNonCriticalAvgDelay = 0.0;
        m_currentOverallSLADeviation = 0.0;
        m_currentNonCriticalSLADeviation = 0.0;
        m_currentCriticalHighSLADeviation = 0.0;
        m_currentCriticalBasicSLADeviation = 0.0;
        
        // Initialize link metrics
        m_currentLinkUsage = {0.0, 0.0, 0.0};
        m_currentLinkThroughput = {0.0, 0.0, 0.0};
    }

    // Method to update performance metrics during simulation
    void UpdateSimulationMetrics(double throughput, double pdr, double avgDelay, double avgJitter,
                                const std::vector<double>& linkUsage, const std::vector<double>& linkThroughput,
                                double recoveryTime = 0.0, double criticalPdr = 0.0, double criticalAvgDelay = 0.0,
                                double nonCriticalPdr = 0.0, double nonCriticalAvgDelay = 0.0,
                                double overallSLADeviation = 0.0, double nonCriticalSLADeviation = 0.0,
                                double criticalHighSLADeviation = 0.0, double criticalBasicSLADeviation = 0.0) {
        
        // Update performance metrics that change during simulation
        m_currentThroughput = throughput;
        m_currentPdr = pdr;
        m_currentAvgDelay = avgDelay;
        m_currentAvgJitter = avgJitter;
        m_currentLinkUsage = linkUsage;
        m_currentLinkThroughput = linkThroughput;
        m_currentRecoveryTime = recoveryTime;
        m_currentCriticalPdr = criticalPdr;
        m_currentCriticalAvgDelay = criticalAvgDelay;
        m_currentNonCriticalPdr = nonCriticalPdr;
        m_currentNonCriticalAvgDelay = nonCriticalAvgDelay;
        m_currentOverallSLADeviation = overallSLADeviation;
        m_currentNonCriticalSLADeviation = nonCriticalSLADeviation;
        m_currentCriticalHighSLADeviation = criticalHighSLADeviation;
        m_currentCriticalBasicSLADeviation = criticalBasicSLADeviation;
    }

    void Log(const std::string& strategy, const std::string& protocol, uint32_t apCount, uint32_t nodeCount, 
            uint32_t payloadSize, double simTime, double throughput, double pdr,
            double avgDelay, double tailLatency, double avgJitter,
            const std::vector<double>& linkUsage, const std::vector<double>& linkThroughput,
            bool duplication, bool interference,
            double recoveryTime, double criticalPdr, double criticalAvgDelay,
            double nonCriticalPdr, double nonCriticalAvgDelay,
            uint32_t tidCount, uint32_t criticalTids, double distance,
            bool mobility, uint32_t runNumber,
            const std::string& interferencePattern = "none",
            double interferenceIntensity = 0.0,
            const std::string& mobilityPattern = "none",
            uint32_t emergencyTids = 0,

            // 3-Tier SLA Deviation metrics
            double overallSLADeviation = 0.0,
            double nonCriticalSLADeviation = 0.0,
            double criticalHighSLADeviation = 0.0,
            double criticalBasicSLADeviation = 0.0) {
        
        // Update performance metrics
        UpdateSimulationMetrics(throughput, pdr, avgDelay, avgJitter, linkUsage, linkThroughput,
                               recoveryTime, criticalPdr, criticalAvgDelay, nonCriticalPdr, nonCriticalAvgDelay,
                               overallSLADeviation, nonCriticalSLADeviation, criticalHighSLADeviation, criticalBasicSLADeviation);
        
        // Enhanced validation before logging
        if (g_verbosityLevel >= 3) {
            SAFE_LOG_IF(3, "\n=== ResultLogger Validation ===\n");
            SAFE_LOG_IF(3, "Strategy: " << strategy << ", Protocol: " << protocol << "\n");
            SAFE_LOG_IF(3, "Throughput: " << throughput << " Mbps, PDR: " << pdr << "%\n");
            SAFE_LOG_IF(3, "Recovery Time: " << recoveryTime << " ms\n");
            SAFE_LOG_IF(3, "Critical PDR: " << criticalPdr << "%, Delay: " << criticalAvgDelay << " ms\n");
            SAFE_LOG_IF(3, "Non-Critical PDR: " << nonCriticalPdr << "%, Delay: " << nonCriticalAvgDelay << " ms\n");
            SAFE_LOG_IF(3, "Link Usage: " << linkUsage[0] << "%, " << linkUsage[1] << "%, " << linkUsage[2] << "%\n");
            
            // Validation checks
            if (criticalTids > 0 && criticalPdr >= 99.9) {
                SAFE_LOG_IF(3, "⚠️  Warning: Critical PDR is very high despite critical TIDs existing\n");
            }
            if (recoveryTime == 0.0 && interference) {
                SAFE_LOG_IF(3, "⚠️  Warning: Recovery time is 0 despite interference being enabled\n");
            }
            if (criticalAvgDelay == 0.0 && criticalTids > 0) {
                SAFE_LOG_IF(3, "⚠️  Warning: Critical delay is 0 despite critical TIDs existing\n");
            }
        }
        
        std::ofstream outfile(m_filename, std::ios_base::app);
        if (!outfile.is_open()) {
            std::cerr << "ERROR: Failed to open CSV file: " << m_filename << std::endl;
            return;
        }
        auto t = std::time(nullptr);
        auto tm = *std::localtime(&t);
        
        double loadBalanceVariance = 0.0;
        double avgLinkUsage = (linkUsage[0] + linkUsage[1] + linkUsage[2]) / 3.0;
        for (int i = 0; i < 3; i++) {
            loadBalanceVariance += std::pow(linkUsage[i] - avgLinkUsage, 2);
        }
        loadBalanceVariance = std::sqrt(loadBalanceVariance / 3.0);
        
        double loadBalancingEfficiency = std::max(0.0, 100.0 - loadBalanceVariance);
        double reliabilityScore = pdr * 0.6 + (100.0 - avgDelay) * 0.4;
        
        std::string slaTier;
        uint32_t normalTids = tidCount - emergencyTids - criticalTids;

        if (emergencyTids > 0 && criticalTids > 0 && normalTids > 0) {
            slaTier = "Mixed";
        } else if (emergencyTids > 0 && criticalTids > 0 && normalTids == 0) {
            slaTier = "Mixed";
        } else if (emergencyTids > 0 && criticalTids == 0 && normalTids > 0) {
            slaTier = "Mixed";
        } else if (emergencyTids == 0 && criticalTids > 0 && normalTids > 0) {
            slaTier = "Mixed";
        } else if (emergencyTids > 0 && criticalTids == 0 && normalTids == 0) {
            slaTier = "Critical High";
        } else if (emergencyTids == 0 && criticalTids > 0 && normalTids == 0) {
            slaTier = "Critical Basic";
        } else if (emergencyTids == 0 && criticalTids == 0 && normalTids > 0) {
            slaTier = "Non Critical";
        } else {
            slaTier = "Unknown";
        }
        
        // Extract scenario type from scenario name
        std::string scenarioType = m_scenarioName;
        size_t pos = scenarioType.find('_');
        if (pos != std::string::npos) {
            scenarioType = scenarioType.substr(0, pos);
        }
        
        // Write CSV output in the exact required order
        outfile << std::put_time(&tm, "%Y-%m-%d %H:%M:%S") << ","
                << m_scenarioName << ","
                << scenarioType << ","
                << strategy << "," << protocol << "," << runNumber << ","
                << apCount << ","  
                << nodeCount << "," 
                << tidCount << ","
                << emergencyTids << ","
                << criticalTids << ","
                << std::fixed << std::setprecision(2) << simTime << ","
                << payloadSize << ","
                << (interference ? "Yes" : "No") << ","
                << interferencePattern << ","
                << (mobility ? "Yes" : "No") << ","
                << mobilityPattern << ","
                << distance << ","
                << (duplication ? "Yes" : "No") << ","
                << std::setprecision(2) << pdr << ","
                << criticalPdr << ","
                << nonCriticalPdr << ","
                << avgDelay << ","
                << criticalAvgDelay << ","
                << nonCriticalAvgDelay << ","
                << throughput << ","
                << std::setprecision(3) << avgJitter << ","
                << recoveryTime << ","
                << tailLatency << ","
                << overallSLADeviation << ","
                << criticalHighSLADeviation << ","
                << criticalBasicSLADeviation << ","
                << nonCriticalSLADeviation << ",";
        
        // Write dynamic link usage data
        for (uint8_t i = 0; i < m_numLinks; i++) {
            if (i < linkUsage.size()) {
                outfile << std::setprecision(1) << linkUsage[i];
            } else {
                outfile << "0.0"; // Default if no data
            }
            outfile << ",";
        }
        
        // Write dynamic link throughput data
        for (uint8_t i = 0; i < m_numLinks; i++) {
            if (i < linkThroughput.size()) {
                outfile << std::setprecision(2) << linkThroughput[i];
            } else {
                outfile << "0.00"; // Default if no data
            }
            outfile << ",";
        }
        
        outfile << std::setprecision(1) << loadBalancingEfficiency << ","
                << std::setprecision(2) << reliabilityScore << ","
                << slaTier << "\n";
        
        outfile.close();
        
        SAFE_LOG_IF(3, "✅ Data logged successfully to CSV\n");
    }

    /**
     * @brief Log TID-specific data for per-flow analysis
     * This provides much more granular data for detailed analysis
     */
    void LogTIDData(uint8_t tid, const std::string& strategy, const std::string& protocol,
                   bool success, double delay, uint32_t bytes, uint8_t linkId,
                   double pdr, double avgDelay, double jitter, 
                   double slaDeviation, bool isCritical, uint32_t runNumber) {
        
        std::string tidFilename = GenerateTIDFilename();
        
        // Check if TID-specific file exists, create header if not
        std::ifstream checkFile(tidFilename);
        bool fileExists = checkFile.good();
        checkFile.close();
        
        std::ofstream outfile(tidFilename, std::ios::app);
        
        if (!fileExists) {
            // Create TID-specific CSV header
            std::string tidHeader = "Timestamp,TID,Strategy,Protocol,Run,LinkID,Success,DelayMs,Bytes,PDR,AvgDelayMs,JitterMs,SLADeviation,IsCritical,PacketSeq";
            
            // Add dynamic link usage at packet level
            for (uint8_t i = 0; i < m_numLinks; i++) {
                tidHeader += ",Link" + std::to_string(i) + "Load";
            }
            tidHeader += "\n";
            outfile << tidHeader;
        }
        
        // Generate timestamp
        auto now = std::chrono::system_clock::now();
        auto time_t = std::chrono::system_clock::to_time_t(now);
        auto ms = std::chrono::duration_cast<std::chrono::milliseconds>(now.time_since_epoch()) % 1000;
        
        outfile << std::put_time(std::localtime(&time_t), "%Y-%m-%d %H:%M:%S")
                << "." << std::setfill('0') << std::setw(3) << ms.count() << ","
                << (uint32_t)tid << ","
                << strategy << ","
                << protocol << ","
                << runNumber << ","
                << (uint32_t)linkId << ","
                << (success ? "1" : "0") << ","
                << std::setprecision(3) << delay << ","
                << bytes << ","
                << std::setprecision(3) << pdr << ","
                << avgDelay << ","
                << jitter << ","
                << slaDeviation << ","
                << (isCritical ? "1" : "0") << ","
                << m_tidPacketSequence[tid]++; // Track packet sequence per TID
        
        // Add current link loads
        for (uint8_t i = 0; i < m_numLinks; i++) {
            auto it = m_linkPacketCount.find(i);
            outfile << "," << (it != m_linkPacketCount.end() ? it->second : 0);
        }
        
        outfile << "\n";
        outfile.close();
        
        SAFE_LOG_IF(4, "📊 TID " << (uint32_t)tid << " data logged (seq=" << (m_tidPacketSequence[tid]-1) << ")\n");
    }

    /**
     * @brief Log windowed data every N packets for more frequent data points
     */
    void LogWindowedData(uint32_t windowSize = 100) {
        m_windowPacketCount++;
        
        if (m_windowPacketCount >= windowSize) {
            std::string windowFilename = GenerateWindowFilename();
            
            // Check if windowed file exists, create header if not
            std::ifstream checkFile(windowFilename);
            bool fileExists = checkFile.good();
            checkFile.close();
            
            std::ofstream outfile(windowFilename, std::ios::app);
            
            if (!fileExists) {
                // Create windowed CSV header
                std::string windowHeader = "Timestamp,WindowID,Strategy,Protocol,Run,WindowSize,WindowPDR,WindowAvgDelay,WindowThroughput,WindowJitter";
                
                // Add per-TID metrics in window
                for (uint32_t tid = 0; tid < 8; tid++) { // Assuming max 8 TIDs
                    windowHeader += ",TID" + std::to_string(tid) + "_PDR,TID" + std::to_string(tid) + "_AvgDelay,TID" + std::to_string(tid) + "_Throughput";
                }
                
                // Add dynamic link metrics
                for (uint8_t i = 0; i < m_numLinks; i++) {
                    windowHeader += ",Link" + std::to_string(i) + "_Usage,Link" + std::to_string(i) + "_Throughput";
                }
                
                windowHeader += "\n";
                outfile << windowHeader;
            }
            
            // Calculate window-based metrics
            double windowPDR = CalculateWindowPDR();
            double windowAvgDelay = CalculateWindowAvgDelay();
            double windowThroughput = CalculateWindowThroughput();
            double windowJitter = CalculateWindowJitter();
            
            // Generate timestamp
            auto now = std::chrono::system_clock::now();
            auto time_t = std::chrono::system_clock::to_time_t(now);
            auto ms = std::chrono::duration_cast<std::chrono::milliseconds>(now.time_since_epoch()) % 1000;
            
            outfile << std::put_time(std::localtime(&time_t), "%Y-%m-%d %H:%M:%S")
                    << "." << std::setfill('0') << std::setw(3) << ms.count() << ","
                    << m_windowID++ << ","
                    << m_currentStrategy << ","
                    << m_currentProtocol << ","
                    << m_currentRunNumber << ","
                    << windowSize << ","
                    << std::setprecision(3) << windowPDR << ","
                    << windowAvgDelay << ","
                    << windowThroughput << ","
                    << windowJitter;
            
            // Add per-TID window metrics
            for (uint32_t tid = 0; tid < 8; tid++) {
                auto tidMetrics = CalculateTIDWindowMetrics(tid);
                outfile << "," << tidMetrics.pdr << "," << tidMetrics.avgDelay << "," << tidMetrics.throughput;
            }
            
            // Add link usage metrics
            auto linkMetrics = CalculateLinkWindowMetrics();
            for (uint8_t i = 0; i < m_numLinks; i++) {
                if (i < linkMetrics.size()) {
                    outfile << "," << linkMetrics[i].usage << "," << linkMetrics[i].throughput;
                } else {
                    outfile << ",0.0,0.0";
                }
            }
            
            outfile << "\n";
            outfile.close();
            
            // Reset window counters
            m_windowPacketCount = 0;
            ResetWindowMetrics();
            
            SAFE_LOG_IF(3, "📈 Window data logged (ID=" << (m_windowID-1) << ", size=" << windowSize << ")\n");
        }
    }
    
    /**
     * @brief Set logging mode and parameters
     */
    void SetLoggingMode(bool enableTIDLogging, bool enableWindowLogging, uint32_t windowSize = 100) {
        m_enableTIDLogging = enableTIDLogging;
        m_enableWindowLogging = enableWindowLogging;
        m_windowSize = windowSize;
        
        SAFE_LOG_IF(2, "📋 Logging mode: TID=" << (enableTIDLogging ? "ON" : "OFF")
                  << ", Window=" << (enableWindowLogging ? "ON" : "OFF")
                  << " (size=" << windowSize << ")\n");
    }
    
    /**
     * @brief Update window metrics (public method)
     */
    void UpdateWindowMetrics(uint8_t tid, bool success, double delay, uint32_t bytes) {
        if (!m_enableWindowLogging) return; // Only update if window logging is enabled
        
        // Update global window metrics
        m_currentWindow.totalPackets++;
        if (success) {
            m_currentWindow.successfulPackets++;
            m_currentWindow.totalDelay += delay;
            m_currentWindow.delayHistory.push_back(delay);
        }
        m_currentWindow.totalThroughput += static_cast<double>(bytes * 8) / 1000.0; // Convert to kbps
        
        // Update TID-specific window metrics
        m_tidWindowMetrics[tid].totalPackets++;
        if (success) {
            m_tidWindowMetrics[tid].successfulPackets++;
            m_tidWindowMetrics[tid].totalDelay += delay;
        }
        m_tidWindowMetrics[tid].totalThroughput += static_cast<double>(bytes * 8) / 1000.0;
    }

private:
    void CreateOutputDirectory() {
        std::string dir = "scratch/output_files_csv";
        struct stat st = {0};
        if (stat(dir.c_str(), &st) == -1) {
            if (mkdir(dir.c_str(), 0777) == -1 && errno != EEXIST) {
                std::cerr << "Failed to create output directory: " << dir << std::endl;
            }
        }
    }

    void InitializeCsv() {
        if (!m_customCsvFile.empty()) {
            m_filename = m_customCsvFile;
        } else {
            m_filename = "scratch/output_files_csv/mlo_unified_results.csv";
        }
        
        std::ifstream checkFile(m_filename);
        bool fileExists = checkFile.good();
        checkFile.close();
        
        if (!fileExists) {
            std::ofstream outfile(m_filename);
            
            // Build dynamic CSV header
            std::string header = "Timestamp,Scenario,ScenarioType,Strategy,Protocol,Run,APCount,WifiCount,TidCount,EmergencyTids,CriticalTids,SimTime,PayloadSize,Interference,InterferencePattern,Mobility,MobilityPattern,Distance,Duplication,PDR,CriticalPDR,NonCriticalPDR,AvgDelay,CriticalAvgDelay,NonCriticalAvgDelay,Throughput,AvgJitterMs,RecoveryTimeMs,TailLatencyMs,OverallSLADeviation,CriticalHighSLADeviation,CriticalBasicSLADeviation,NonCriticalSLADeviation";
            
            // Add dynamic link usage columns
            for (uint8_t i = 0; i < m_numLinks; i++) {
                header += ",Link" + std::to_string(i) + "Usage";
            }
            
            // Add dynamic link throughput columns
            for (uint8_t i = 0; i < m_numLinks; i++) {
                header += ",Link" + std::to_string(i) + "ThroughputMbps";
            }
            
            header += ",LoadBalancingEfficiency,ReliabilityScore,SLATier\n";
            
            outfile << header;
            outfile.close();
        }
    }


    std::string m_filename;
    std::string m_scenarioName;
    std::string m_customCsvFile;
    uint8_t m_numLinks;  // Number of links for dynamic CSV generation
    std::map<uint8_t, uint64_t> m_linkPacketCount;
    
    // Store simulation parameters
    std::string m_currentStrategy;
    std::string m_currentProtocol;
    uint32_t m_currentNodeCount;
    double m_currentSimTime;
    double m_currentThroughput;
    double m_currentPdr;
    double m_currentAvgDelay;
    double m_currentAvgJitter;
    std::vector<double> m_currentLinkUsage;
    std::vector<double> m_currentLinkThroughput;
    bool m_currentDuplication;
    bool m_currentInterference;
    double m_currentRecoveryTime;
    double m_currentCriticalPdr;
    double m_currentCriticalAvgDelay;
    double m_currentNonCriticalPdr;
    double m_currentNonCriticalAvgDelay;
    uint32_t m_currentTidCount;
    uint32_t m_currentCriticalTids;
    
    // Enhanced logging support
    bool m_enableTIDLogging = false;
    bool m_enableWindowLogging = false;
    uint32_t m_windowSize = 100;
    uint32_t m_windowPacketCount = 0;
    uint32_t m_windowID = 1;
    std::map<uint8_t, uint32_t> m_tidPacketSequence;
    
    // Window-based metrics tracking
    struct WindowMetrics {
        uint32_t totalPackets = 0;
        uint32_t successfulPackets = 0;
        double totalDelay = 0.0;
        double totalThroughput = 0.0;
        std::vector<double> delayHistory;
    };
    WindowMetrics m_currentWindow;
    std::map<uint8_t, WindowMetrics> m_tidWindowMetrics;
    
    struct LinkWindowMetrics {
        double usage = 0.0;
        double throughput = 0.0;
    };
    double m_currentDistance;
    bool m_currentMobility;
    uint32_t m_currentRunNumber;
    std::string m_currentInterferencePattern;
    double m_currentInterferenceIntensity;
    std::string m_currentMobilityPattern;
    uint32_t m_currentEmergencyTids;
    uint32_t m_currentHighPriorityTids;
    std::string m_currentFadingModel;
    double m_currentOverallSLADeviation;
    double m_currentNonCriticalSLADeviation;
    double m_currentCriticalHighSLADeviation;
    double m_currentCriticalBasicSLADeviation;
    std::string m_currentTrafficType;
    std::string m_currentStatus;
    
    // Helper methods for enhanced logging
    std::string GenerateTIDFilename() {
        return "scratch/output_files_csv/mlo_tid_detailed_" + m_currentStrategy + "_" + m_currentProtocol + ".csv";
    }
    
    std::string GenerateWindowFilename() {
        return "scratch/output_files_csv/mlo_windowed_" + m_currentStrategy + "_" + m_currentProtocol + ".csv";
    }
    
    double CalculateWindowPDR() {
        if (m_currentWindow.totalPackets == 0) return 0.0;
        return (static_cast<double>(m_currentWindow.successfulPackets) / m_currentWindow.totalPackets) * 100.0;
    }
    
    double CalculateWindowAvgDelay() {
        if (m_currentWindow.successfulPackets == 0) return 0.0;
        return m_currentWindow.totalDelay / m_currentWindow.successfulPackets;
    }
    
    double CalculateWindowThroughput() {
        return m_currentWindow.totalThroughput;
    }
    
    double CalculateWindowJitter() {
        if (m_currentWindow.delayHistory.size() < 2) return 0.0;
        
        double variance = 0.0;
        double mean = CalculateWindowAvgDelay();
        
        for (double delay : m_currentWindow.delayHistory) {
            variance += std::pow(delay - mean, 2);
        }
        
        return std::sqrt(variance / m_currentWindow.delayHistory.size());
    }
    
    struct TIDMetrics {
        double pdr = 0.0;
        double avgDelay = 0.0;
        double throughput = 0.0;
    };
    
    TIDMetrics CalculateTIDWindowMetrics(uint8_t tid) {
        TIDMetrics metrics;
        
        auto it = m_tidWindowMetrics.find(tid);
        if (it != m_tidWindowMetrics.end()) {
            const WindowMetrics& tidWindow = it->second;
            if (tidWindow.totalPackets > 0) {
                metrics.pdr = (static_cast<double>(tidWindow.successfulPackets) / tidWindow.totalPackets) * 100.0;
                if (tidWindow.successfulPackets > 0) {
                    metrics.avgDelay = tidWindow.totalDelay / tidWindow.successfulPackets;
                }
                metrics.throughput = tidWindow.totalThroughput;
            }
        }
        
        return metrics;
    }
    
    std::vector<LinkWindowMetrics> CalculateLinkWindowMetrics() {
        std::vector<LinkWindowMetrics> linkMetrics(m_numLinks);
        
        uint64_t totalLoad = 0;
        for (uint8_t i = 0; i < m_numLinks; i++) {
            auto it = m_linkPacketCount.find(i);
            if (it != m_linkPacketCount.end()) {
                totalLoad += it->second;
            }
        }
        
        for (uint8_t i = 0; i < m_numLinks; i++) {
            auto it = m_linkPacketCount.find(i);
            if (it != m_linkPacketCount.end() && totalLoad > 0) {
                linkMetrics[i].usage = (static_cast<double>(it->second) / totalLoad) * 100.0;
                linkMetrics[i].throughput = m_currentLinkThroughput.size() > i ? m_currentLinkThroughput[i] : 0.0;
            }
        }
        
        return linkMetrics;
    }
    
    void ResetWindowMetrics() {
        m_currentWindow = WindowMetrics();
        m_tidWindowMetrics.clear();
    }

};

// Packet Tags for MLO Tracking
/**
 * @class MLOLinkTag
 * @brief Packet tag for tracking which MLO link a packet was transmitted on
 * 
 * This tag is attached to each packet to identify which specific link in the
 * Multi-Link Operation setup was used for transmission. This information is
 * crucial for:
 * - Per-link performance analysis and monitoring
 * - Link utilization tracking for load balancing evaluation  
 * - Debugging packet routing decisions in MLO strategies
 * - Correlating packet success/failure with specific link conditions
 * 
 */
class MLOLinkTag : public Tag {
public:
    /**
     * @brief Register this tag type with the NS-3 type system
     * @return TypeId for the MLOLinkTag class
     */
    static TypeId GetTypeId() {
        static TypeId tid = TypeId("MLOLinkTag")
            .SetParent<Tag>()
            .AddConstructor<MLOLinkTag>();
        return tid;
    }
    
    /**
     * @brief Set the link identifier for this packet
     * @param linkId Unique identifier for the MLO link
     */
    void SetLinkId(uint8_t linkId) { m_linkId = linkId; }
    
    /**
     * @brief Get the stored link identifier
     * @return Link identifier associated with this packet
     */
    uint8_t GetLinkId() const { return m_linkId; }
    
    // NS-3 Tag interface implementation
    virtual TypeId GetInstanceTypeId() const { return GetTypeId(); }
    virtual uint32_t GetSerializedSize() const { return 1; }  // Single byte storage
    virtual void Serialize(TagBuffer buf) const { buf.WriteU8(m_linkId); }
    virtual void Deserialize(TagBuffer buf) { m_linkId = buf.ReadU8(); }
    virtual void Print(std::ostream& os) const { os << "LinkId=" << (uint32_t)m_linkId; }
    
private:
    uint8_t m_linkId = 0;  ///Link identifier
};
NS_OBJECT_ENSURE_REGISTERED(MLOLinkTag);  // Register with NS-3 object system

/**
 * @class TidTag
 * @brief Packet tag for tracking Traffic Identifier (TID) assignments
 * 
 * This tag identifies which Traffic Identifier (TID) a packet belongs to,
 * enabling proper QoS classification and SLA compliance monitoring. TIDs
 * are used to categorize traffic into different service levels:
 * 
 * - Emergency TIDs: Highest priority, ultra-low latency requirements
 * - Critical TIDs: High priority, low latency and high reliability needs  
 * - Best Effort TIDs: Standard priority, best-effort delivery
 * 
 * The TID information is essential for:
 * - Applying appropriate scheduling algorithms
 * - Measuring SLA compliance per traffic class
 * - Making intelligent link selection decisions in MLO strategies
 */
class TidTag : public Tag {
public:
    /**
     * @brief Register this tag type with the NS-3 type system
     * @return TypeId for the TidTag class
     */
    static TypeId GetTypeId() {
        static TypeId tid = TypeId("TidTag")
            .SetParent<Tag>()
            .AddConstructor<TidTag>();
        return tid;
    }
    
    /**
     * @brief Set the Traffic Identifier for this packet
     * @param tid Traffic Identifier value
     */
    void SetTid(uint8_t tid) { m_tid = tid; }
    
    /**
     * @brief Get the stored Traffic Identifier
     * @return TID value associated with this packet
     */
    uint8_t GetTid() const { return m_tid; }
    
    // NS-3 Tag interface implementation
    virtual TypeId GetInstanceTypeId() const { return GetTypeId(); }
    virtual uint32_t GetSerializedSize() const { return 1; }  // Single byte storage
    virtual void Serialize(TagBuffer buf) const { buf.WriteU8(m_tid); }
    virtual void Deserialize(TagBuffer buf) { m_tid = buf.ReadU8(); }
    virtual void Print(std::ostream& os) const { os << "TID=" << (uint32_t)m_tid; }
private:
    uint8_t m_tid = 0;  /// Traffic Identifier
};
NS_OBJECT_ENSURE_REGISTERED(TidTag);  // Register with NS-3 object system

/**
 * @class TimestampTag
 * @brief Packet tag for precise timing measurements and delay analysis
 * 
 * This tag records high-precision timestamps for packets, enabling accurate
 * end-to-end delay measurements and jitter analysis. The timestamp is captured
 * when the packet is first transmitted and compared with the reception time
 * to calculate network delays.
 * 
 * Key applications:
 * - End-to-end delay measurement for SLA compliance verification
 * - Jitter calculation for real-time application quality assessment  
 * - Network performance monitoring and debugging
 * - Comparative analysis of different MLO strategies' latency characteristics
 * - Detection of network congestion and bottleneck identification
 * 
 * The tag uses NS-3's high-precision Time class
 */
class TimestampTag : public Tag {
public:
    /**
     * @brief Register this tag type with the NS-3 type system
     * @return TypeId for the TimestampTag class
     */
    static TypeId GetTypeId() {
        static TypeId tid = TypeId("TimestampTag")
            .SetParent<Tag>()
            .AddConstructor<TimestampTag>();
        return tid;
    }
    
    /**
     * @brief Set the timestamp for this packet
     * @param timestamp High-precision timestamp when packet was created/transmitted
     */
    void SetTimestamp(Time timestamp) { m_timestamp = timestamp; }
    
    /**
     * @brief Get the stored timestamp
     * @return Timestamp associated with this packet
     */
    Time GetTimestamp() const { return m_timestamp; }
    // NS-3 Tag interface implementation
    virtual TypeId GetInstanceTypeId() const { return GetTypeId(); }
    virtual uint32_t GetSerializedSize() const { return sizeof(Time); }
    virtual void Serialize(TagBuffer i) const { 
        i.WriteU64(m_timestamp.GetNanoSeconds()); 
    }
    virtual void Deserialize(TagBuffer i) { 
        m_timestamp = NanoSeconds(i.ReadU64());
    }
    virtual void Print(std::ostream &os) const { 
        os << "Timestamp=" << m_timestamp.GetSeconds() << "s"; 
    }

private:
    Time m_timestamp;  // High-precision timestamp
};
NS_OBJECT_ENSURE_REGISTERED(TimestampTag);  // Register with NS-3 object system

/**
 * @class DuplicationTag
 * @brief Tag to track packet duplication for reliability analysis
 */
class DuplicationTag : public Tag {
public:
    static TypeId GetTypeId() {
        static TypeId tid = TypeId("ns3::DuplicationTag")
            .SetParent<Tag>()
            .SetGroupName("MLO")
            .AddConstructor<DuplicationTag>();
        return tid;
    }
    
    void SetOriginalLink(uint8_t linkId) { m_originalLink = linkId; }
    void SetDuplicateLink(uint8_t linkId) { m_duplicateLink = linkId; }
    uint8_t GetOriginalLink() const { return m_originalLink; }
    uint8_t GetDuplicateLink() const { return m_duplicateLink; }
    
    virtual TypeId GetInstanceTypeId() const { return GetTypeId(); }
    virtual uint32_t GetSerializedSize() const { return 2; }
    virtual void Serialize(TagBuffer i) const { 
        i.WriteU8(m_originalLink);
        i.WriteU8(m_duplicateLink);
    }
    virtual void Deserialize(TagBuffer i) { 
        m_originalLink = i.ReadU8();
        m_duplicateLink = i.ReadU8();
    }
    virtual void Print(std::ostream &os) const { 
        os << "Duplicate: " << (uint32_t)m_originalLink << " -> " << (uint32_t)m_duplicateLink;
    }

private:
    uint8_t m_originalLink = 0;
    uint8_t m_duplicateLink = 0;
};
NS_OBJECT_ENSURE_REGISTERED(DuplicationTag);

class CriticalityTag : public Tag {
public:
    static TypeId GetTypeId() {
        static TypeId tid = TypeId("CriticalityTag")
            .SetParent<Tag>()
            .AddConstructor<CriticalityTag>();
        return tid;
    }
    void SetIsCritical(bool critical) { m_isCritical = critical; }
    bool GetIsCritical() const { return m_isCritical; }
    virtual TypeId GetInstanceTypeId() const { return GetTypeId(); }
    virtual uint32_t GetSerializedSize() const { return 1; }
    virtual void Serialize(TagBuffer buf) const { buf.WriteU8(m_isCritical ? 1 : 0); }
    virtual void Deserialize(TagBuffer buf) { m_isCritical = buf.ReadU8() != 0; }
    virtual void Print(std::ostream& os) const { os << "Critical=" << (m_isCritical ? "true" : "false"); }
private:
    bool m_isCritical = false;
};
NS_OBJECT_ENSURE_REGISTERED(CriticalityTag);

// ====================== LINK QUALITY MONITOR ======================
/**
 * @class LinkQualityMonitor
 * @brief Real-time monitoring and analysis of MLO link performance characteristics
 * 
 * This class provides comprehensive monitoring capabilities for Multi-Link Operation
 * networks, tracking detailed performance metrics for each individual link and
 * providing aggregate statistics for the entire MLO setup.
 * 
 * KEY MONITORING CAPABILITIES:
 * - Real-time packet delivery ratio (PDR) calculation with sliding window analysis
 * - End-to-end delay measurement and jitter analysis
 * - Throughput monitoring with historical trend analysis
 * - Failure detection and recovery time measurement
 * - Critical vs non-critical traffic differentiation and separate analysis
 * - Link utilization tracking for load balancing optimization
 * 
 * ADAPTIVE MONITORING FEATURES:
 * - Sliding window approach for responsive PDR calculation (configurable window size)
 * - Failure state detection with automatic recovery tracking
 * - Historical performance data retention for trend analysis
 * - Real-time adaptation to changing network conditions
 * 
 * DATA COLLECTION AND ANALYSIS:
 * - Statistical analysis with percentile calculations (50th, 90th, 95th, 99th)
 * - Confidence interval computation for reliable performance assessment
 * - Time-series data collection for post-simulation analysis
 * - Integration with SLA monitoring for compliance verification
 * 
 * This monitor is essential for evaluating different MLO strategies and
 * understanding their impact on network performance under various conditions.
 */
class LinkQualityMonitor {
public:
    /**
     * @struct LinkMetrics
     * @brief Comprehensive performance metrics for a single MLO link
     * 
     * This structure maintains detailed performance statistics for an individual
     * link within the MLO configuration, supporting both real-time monitoring
     * and historical analysis.
     */
    struct LinkMetrics {
        // ===== BASIC PERFORMANCE METRICS =====
        double pdr = 1.0;                    ///< Packet Delivery Ratio (0.0-1.0)
        double avgDelay = 0.0;               ///< Average end-to-end delay in milliseconds
        double jitter = 0.0;                 ///< Delay variation (jitter) in milliseconds
        uint64_t packetsTransmitted = 0;     ///< Total packets transmitted on this link
        uint64_t packetsReceived = 0;        ///< Total packets successfully received
        uint64_t packetsDropped = 0;         ///< Total packets dropped/lost
        uint64_t bytesTransmitted = 0;       ///< Total bytes transmitted
        uint64_t bytesReceived = 0;          ///< Total bytes successfully received
        double throughputMbps = 0.0;         ///< Current throughput in Mbps
        Time lastUpdate = Seconds(0);        ///< Timestamp of last metric update
        
        // ===== ENHANCED RECOVERY TRACKING =====
        Time recoveryTime = Seconds(0);              ///< Time to recover from last failure
        bool isInFailureState = false;               ///< Current failure state indicator
        bool hasRecoveredAtLeastOnce = false;        ///< Has experienced at least one recovery
        Time failureStartTime = Seconds(0);          ///< When current failure began
        Time lastRecoveryTime = Seconds(0);          ///< When last recovery completed
        uint32_t failureCount = 0;                   ///< Total number of failures detected
        uint32_t recoveryCount = 0;                  ///< Total number of recoveries
        std::vector<double> recoveryTimes;           ///< Historical recovery time measurements
        
        // ===== DUPLICATION TRACKING =====
        uint64_t duplicatesTransmitted = 0;         ///< Total duplicate packets sent via this link
        uint64_t duplicatesReceived = 0;            ///< Total duplicate packets received on this link
        double duplicationRatio = 0.0;              ///< Ratio of duplicates to original packets
        
        // ===== SLIDING WINDOW DATA STRUCTURES =====
        std::deque<bool> recentPacketResults;        ///< Recent packet success/failure history
        static const size_t WINDOW_SIZE = 50;       ///< Size of sliding window for responsive metrics
        
        std::deque<double> recentDelays;             ///< Recent delay measurements for jitter calculation
        std::deque<Time> throughputMeasurements;     ///< Timestamps for throughput calculation
        std::deque<uint64_t> bytesHistory;           ///< Recent byte counts for throughput tracking
        
        // Enhanced Critical vs Non-Critical tracking
        uint64_t criticalPacketsTransmitted = 0;
        uint64_t criticalPacketsReceived = 0;
        uint64_t nonCriticalPacketsTransmitted = 0;
        uint64_t nonCriticalPacketsReceived = 0;
        double criticalDelaySum = 0.0;
        double nonCriticalDelaySum = 0.0;
        uint32_t criticalDelayCount = 0;
        uint32_t nonCriticalDelayCount = 0;
        
        // Alpha-smoothed delays for more responsive calculations
        double criticalAvgDelay = 0.0;
        double nonCriticalAvgDelay = 0.0;
        
        // Track timing for validation
        Time lastCriticalPacketTime = Seconds(0);
        Time lastNonCriticalPacketTime = Seconds(0);
        
        // Calculate current window PDR
        double GetCurrentWindowPDR() const {
            if (recentPacketResults.empty()) return 1.0;
            
            uint32_t successes = 0;
            for (bool result : recentPacketResults) {
                if (result) successes++;
            }
            return (double)successes / recentPacketResults.size();
        }
    };

    LinkQualityMonitor(uint8_t numLinks, double pdrThreshold = 0.95) 
        : m_numLinks(numLinks), m_pdrThreshold(pdrThreshold), 
          m_monitoringStartTime(Simulator::Now()) {
        m_metrics.resize(numLinks);
        for (auto& metric : m_metrics) {
            metric.recentDelays.clear();
            metric.throughputMeasurements.clear();
            metric.bytesHistory.clear();
            metric.recoveryTimes.clear();
            metric.recentPacketResults.clear();
        }
        SAFE_LOG_IF(2, "LinkQualityMonitor initialized with " << (uint32_t)numLinks 
                  << " links, PDR threshold: " << pdrThreshold << "\n");
    }

    void UpdateLinkMetrics(uint8_t linkId, bool success, double delay = 0, uint32_t bytes = 0, uint8_t tid = 0, bool isDuplicate = false, bool isCritical = false) {
        // Enhanced bounds checking with proper error handling
        if (!ValidationUtils::IsValidLinkId(linkId, m_numLinks)) {
            SAFE_LOG_IF(1, "ERROR: Invalid linkId " << (uint32_t)linkId << " >= " << (uint32_t)m_numLinks << "\n");
            return;
        }
        
        if (!ValidationUtils::IsValidTid(tid)) {
            SAFE_LOG_IF(1, "ERROR: Invalid TID " << (uint32_t)tid << " >= 256\n");
            return;
        }
        
        // Skip delay validation for transmission tracking (delay = -1.0)
        // This is used to mark packets as "transmitted but not yet received"
        if (delay != -1.0 && !ValidationUtils::IsRealisticDelay(delay)) {
            SAFE_LOG_IF(1, "ERROR: Unrealistic delay " << delay << "ms (max 10000ms)\n");
            return;
        }
        
        // Use passed isCritical parameter if provided, otherwise calculate it
        // Note: isCritical parameter is now part of the function signature
        if (tid < 255 && !isCritical) { // If not explicitly marked critical, check QoS rules
            isCritical = QoSUtils::IsCriticalTraffic(tid, m_emergencyTids, m_criticalTids);
        }
        
        auto& metric = m_metrics[linkId];
        Time currentTime = Simulator::Now();
        
        // CRITICAL FIX: Only use simulated outcomes if this is NOT from actual packet reception
        // If bytes > 0 and delay >= 0, this is real packet data from EnhancedPacketSink
        bool isRealPacketData = (bytes > 0);
        if (!isRealPacketData) {
            // This is synthetic data for testing - use simulated outcome
            success = SimulatePacketOutcome(linkId, currentTime, isCritical);
        }
        // Otherwise, use the actual success value passed from EnhancedPacketSink
        
        // Update sliding window for responsive PDR calculation
        metric.recentPacketResults.push_back(success);
        if (metric.recentPacketResults.size() > LinkMetrics::WINDOW_SIZE) {
            metric.recentPacketResults.pop_front();
        }
        
        // Update packet counts
        metric.packetsTransmitted++;
        metric.bytesTransmitted += bytes;
        
        if (isCritical) {
            metric.criticalPacketsTransmitted++;
            SAFE_LOG_IF(3, "LinkQualityMonitor: Critical packet TX - TID=" << (uint32_t)tid 
                      << ", Link=" << (uint32_t)linkId << ", Total=" << metric.criticalPacketsTransmitted << "\n");
        } else {
            metric.nonCriticalPacketsTransmitted++;
            SAFE_LOG_IF(3, "LinkQualityMonitor: Non-critical packet TX - TID=" << (uint32_t)tid 
                      << ", Link=" << (uint32_t)linkId << ", Total=" << metric.nonCriticalPacketsTransmitted << "\n");
        }
        
        // Track duplication
        if (isDuplicate) {
            if (success) {
                metric.duplicatesReceived++;
                SAFE_LOG_IF(3, "LinkQualityMonitor: Duplicate packet received - TID=" << (uint32_t)tid 
                          << ", Link=" << (uint32_t)linkId << ", Total duplicates RX=" << metric.duplicatesReceived << "\n");
            } else {
                metric.duplicatesTransmitted++; // Count failed duplicates as transmitted attempts
                SAFE_LOG_IF(3, "LinkQualityMonitor: Duplicate packet TX attempt - TID=" << (uint32_t)tid 
                          << ", Link=" << (uint32_t)linkId << ", Total duplicates TX=" << metric.duplicatesTransmitted << "\n");
            }
        }
        
        if (success) {
            metric.packetsReceived++;
            metric.bytesReceived += bytes;
            
            if (isCritical) {
                metric.criticalPacketsReceived++;
                SAFE_LOG_IF(3, "LinkQualityMonitor: Critical packet RX - TID=" << (uint32_t)tid 
                          << ", Link=" << (uint32_t)linkId << ", Total=" << metric.criticalPacketsReceived 
                          << ", Delay=" << delay << "ms\n");
                if (delay > 0) {
                    metric.criticalDelaySum += delay;
                    metric.criticalDelayCount++;
                    metric.lastCriticalPacketTime = currentTime;
                    
                    // Update critical delay with EWMA
                    if (metric.criticalAvgDelay == 0) {
                        metric.criticalAvgDelay = delay;
                    } else {
                        double alpha = 0.125; // Standard EWMA factor (1/8)
                        metric.criticalAvgDelay = alpha * delay + (1 - alpha) * metric.criticalAvgDelay;
                    }
                }
            } else {
                metric.nonCriticalPacketsReceived++;
                SAFE_LOG_IF(3, "LinkQualityMonitor: Non-critical packet RX - TID=" << (uint32_t)tid 
                          << ", Link=" << (uint32_t)linkId << ", Total=" << metric.nonCriticalPacketsReceived 
                          << ", Delay=" << delay << "ms\n");
                if (delay > 0) {
                    metric.nonCriticalDelaySum += delay;
                    metric.nonCriticalDelayCount++;
                    metric.lastNonCriticalPacketTime = currentTime;
                    
                    // Update non-critical delay with EWMA
                    if (metric.nonCriticalAvgDelay == 0) {
                        metric.nonCriticalAvgDelay = delay;
                    } else {
                        double alpha = 0.125; // Standard EWMA factor (1/8)
                        metric.nonCriticalAvgDelay = alpha * delay + (1 - alpha) * metric.nonCriticalAvgDelay;
                    }
                }
            }
            
            // Update delay tracking with proper jitter calculation
            if (delay > 0 && metric.recentDelays.size() < 100) {
                metric.recentDelays.push_back(delay);
                
                // Use EWMA for more responsive average delay calculation
                if (metric.avgDelay == 0) {
                    metric.avgDelay = delay;
                } else {
                    double alpha = 0.125; // Standard EWMA factor (1/8)
                    metric.avgDelay = alpha * delay + (1 - alpha) * metric.avgDelay;
                }
                
                // Calculate proper jitter as inter-packet delay variation (RFC 3550)
                if (metric.recentDelays.size() > 1) {
                    double jitterSum = 0.0;
                    for (size_t i = 1; i < metric.recentDelays.size(); i++) {
                        double delayDiff = std::abs(metric.recentDelays[i] - metric.recentDelays[i-1]);
                        jitterSum += delayDiff;
                    }
                    metric.jitter = jitterSum / (metric.recentDelays.size() - 1);
                }
            }
        } else {
            metric.packetsDropped++;
            SAFE_LOG_IF(3, "Packet dropped on link " << (uint32_t)linkId 
                      << " (critical: " << (isCritical ? "Yes" : "No") << ")\n");
        }
        
        double windowPdr = metric.GetCurrentWindowPDR();
        
        // Update overall PDR - Fixed: Include both successful and failed transmissions
        if (metric.packetsTransmitted > 0) {
            metric.pdr = (double)metric.packetsReceived / metric.packetsTransmitted;
        }
        
        // Track failed transmissions for accurate PDR calculation
        if (!success && bytes > 0) {
            // This was a transmission attempt that failed
            metric.packetsTransmitted++; // Ensure failed transmissions are counted
            SAFE_LOG_IF(3, "LinkQualityMonitor: Failed transmission counted - TID=" << (uint32_t)tid 
                      << ", Link=" << (uint32_t)linkId << ", Total TX=" << metric.packetsTransmitted << "\n");
        }
        
        // Use sliding window for more responsive failure detection
        bool currentlyFailing = (windowPdr < m_pdrThreshold) && (metric.recentPacketResults.size() >= 10);
        
        if (!metric.isInFailureState && currentlyFailing) {
            // Link failure detected
            metric.isInFailureState = true;
            metric.failureStartTime = currentTime;
            metric.failureCount++;
            
            SAFE_LOG_IF(2, "🔴 Link " << (uint32_t)linkId << " failure detected at " 
                      << currentTime.GetSeconds() << "s (Window PDR: " << (windowPdr * 100.0) 
                      << "%, Overall PDR: " << (metric.pdr * 100.0) << "%)\n");
        } 
        else if (metric.isInFailureState && !currentlyFailing) {
            // Recovery detected
            Time recoveryDuration = currentTime - metric.failureStartTime;
            metric.recoveryTime = recoveryDuration;
            metric.recoveryTimes.push_back(recoveryDuration.GetMilliSeconds());
            metric.isInFailureState = false;
            metric.hasRecoveredAtLeastOnce = true;
            metric.lastRecoveryTime = currentTime;
            metric.recoveryCount++;
            
            SAFE_LOG_IF(2, "🟢 Link " << (uint32_t)linkId << " recovered at " 
                      << currentTime.GetSeconds() << "s (Recovery time: " 
                      << recoveryDuration.GetMilliSeconds() << "ms, Window PDR restored to: " 
                      << (windowPdr * 100.0) << "%)\n");
        }

        // Update throughput calculation with proper sliding window
        if (success && bytes > 0) {
            metric.throughputMeasurements.push_back(currentTime);
            metric.bytesHistory.push_back(bytes); // Store actual packet bytes, not cumulative
            
            while (!metric.throughputMeasurements.empty() && 
                   (currentTime - metric.throughputMeasurements.front()).GetSeconds() > 1.0) {
                metric.throughputMeasurements.pop_front();
                metric.bytesHistory.pop_front();
            }
            
            // Calculate throughput based on bytes in the current sliding window
            if (!metric.bytesHistory.empty() && !metric.throughputMeasurements.empty()) {
                Time windowDuration = currentTime - metric.throughputMeasurements.front();
                if (windowDuration.GetSeconds() > 0) {
                    uint64_t totalBytesInWindow = std::accumulate(metric.bytesHistory.begin(), 
                                                                metric.bytesHistory.end(), 0ULL);
                    metric.throughputMbps = (totalBytesInWindow * 8.0) / (windowDuration.GetSeconds() * 1e6);
                }
            }
        }
        
        metric.lastUpdate = currentTime;
    }

    bool SimulatePacketOutcome(uint8_t linkId, Time currentTime, bool isCritical) {
        // Create realistic failure patterns based on simulation conditions
        static Ptr<UniformRandomVariable> random = CreateObject<UniformRandomVariable>();
        
        // Base success probability (varies by link)
        double baseSuccessProb = 0.98; // 98% base success rate
        if (linkId == 0) baseSuccessProb = 0.95; // 2.4GHz more congested
        if (linkId == 1) baseSuccessProb = 0.97; // 5GHz moderate
        if (linkId == 2) baseSuccessProb = 0.98; // 6GHz best
        
        // Apply interference effects (time-based patterns)
        double interferenceEffect = 1.0;
        double time = currentTime.GetSeconds();
        
        // Simulate interference cycles (every 3 seconds, 1.5s interference on, 1.5s off)
        if (fmod(time, 3.0) < 1.5) {
            interferenceEffect = 0.85; // 15% additional failure during interference
            SAFE_LOG_IF(3, "Interference active at " << time << "s on link " << (uint32_t)linkId << "\n");
        }
        
        // Critical traffic gets slight priority
        double criticalBonus = isCritical ? 1.02 : 1.0;
        
        // Add some randomness for realistic variations
        double randomFactor = 0.95 + random->GetValue(0.0, 0.1); // ±5% random variation
        
        double finalSuccessProb = baseSuccessProb * interferenceEffect * criticalBonus * randomFactor;
        finalSuccessProb = std::min(1.0, std::max(0.0, finalSuccessProb)); // Clamp to [0,1]
        
        bool success = random->GetValue(0.0, 1.0) < finalSuccessProb;
        
        if (!success && g_verbosityLevel >= 3) {
            SAFE_LOG_IF(3, "Packet failure simulated on link " << (uint32_t)linkId 
                      << " (prob: " << finalSuccessProb << ", critical: " << isCritical << ")\n");
        }
        
        return success;
    }

    // Enhanced recovery time calculation
    Time GetAverageRecoveryTime() const {
        std::vector<double> allRecoveryTimes;
        
        for (const auto& metric : m_metrics) {
            for (double recoveryMs : metric.recoveryTimes) {
                allRecoveryTimes.push_back(recoveryMs);
            }
        }
        
        if (allRecoveryTimes.empty()) {
            SAFE_LOG_IF(3, "No recovery events found across all links\n");
            return Seconds(0);
        }
        
        double avgRecoveryMs = std::accumulate(allRecoveryTimes.begin(), allRecoveryTimes.end(), 0.0) 
                              / allRecoveryTimes.size();
        SAFE_LOG_IF(2, "Average recovery time calculated: " << avgRecoveryMs << "ms from " 
                  << allRecoveryTimes.size() << " events\n");
        return MilliSeconds(avgRecoveryMs);
    }

    // Enhanced critical traffic PDR calculation
    double GetCriticalPDR() const {
        uint64_t totalCriticalTx = 0;
        uint64_t totalCriticalRx = 0;
        
        for (const auto& metric : m_metrics) {
            totalCriticalTx += metric.criticalPacketsTransmitted;
            totalCriticalRx += metric.criticalPacketsReceived;
        }
        
        if (totalCriticalTx == 0) {
            SAFE_LOG_IF(3, "No critical packets transmitted - returning sentinel value\n");
            return -1.0; // Sentinel value indicating no critical traffic data
        }
        
        double criticalPDR = (double)totalCriticalRx / totalCriticalTx * 100.0;
        SAFE_LOG_IF(3, "Critical PDR calculation: " << totalCriticalRx << "/" << totalCriticalTx 
                  << " = " << criticalPDR << "%\n");
        return criticalPDR;
    }

    double GetNonCriticalPDR() const {
        uint64_t totalNonCriticalTx = 0;
        uint64_t totalNonCriticalRx = 0;
        
        for (const auto& metric : m_metrics) {
            totalNonCriticalTx += metric.nonCriticalPacketsTransmitted;
            totalNonCriticalRx += metric.nonCriticalPacketsReceived;
        }
        
        if (totalNonCriticalTx == 0) {
            SAFE_LOG_IF(3, "No non-critical packets transmitted - returning sentinel value\n");
            return -1.0; // Sentinel value indicating no non-critical traffic data
        }
        
        return (double)totalNonCriticalRx / totalNonCriticalTx * 100.0;
    }

    double GetCriticalAvgDelay() const {
        double totalCriticalDelaySum = 0.0;
        uint32_t totalCriticalDelayCount = 0;
        
        for (const auto& metric : m_metrics) {
            if (metric.criticalDelayCount > 0) {
                totalCriticalDelaySum += metric.criticalDelaySum;
                totalCriticalDelayCount += metric.criticalDelayCount;
            }
        }
        
        if (totalCriticalDelayCount == 0) {
            SAFE_LOG_IF(3, "No critical delay measurements - returning sentinel value\n");
            return -1.0; // Sentinel value indicating no critical delay data
        }
        
        return totalCriticalDelaySum / totalCriticalDelayCount;
    }

    double GetNonCriticalAvgDelay() const {
        double totalNonCriticalDelaySum = 0.0;
        uint32_t totalNonCriticalDelayCount = 0;
        
        for (const auto& metric : m_metrics) {
            if (metric.nonCriticalDelayCount > 0) {
                totalNonCriticalDelaySum += metric.nonCriticalDelaySum;
                totalNonCriticalDelayCount += metric.nonCriticalDelayCount;
            }
        }
        
        if (totalNonCriticalDelayCount == 0) {
            SAFE_LOG_IF(3, "No non-critical delay measurements - returning sentinel value\n");
            return -1.0; // Sentinel value indicating no non-critical delay data
        }
        
        return totalNonCriticalDelaySum / totalNonCriticalDelayCount;
    }

    void PrintDebugInfo() const {
        if (g_verbosityLevel >= 2) {
            SAFE_LOG_IF(2, "\n=== LinkQualityMonitor Debug Info ===\n");
            for (size_t i = 0; i < m_metrics.size(); i++) {
                const auto& metric = m_metrics[i];
                SAFE_LOG_IF(2, "Link " << i << ":\n");
                SAFE_LOG_IF(2, "  Total TX/RX/Dropped: " << metric.packetsTransmitted 
                          << "/" << metric.packetsReceived << "/" << metric.packetsDropped << "\n");
                SAFE_LOG_IF(2, "  Critical TX/RX: " << metric.criticalPacketsTransmitted 
                          << "/" << metric.criticalPacketsReceived << "\n");
                SAFE_LOG_IF(2, "  Non-Critical TX/RX: " << metric.nonCriticalPacketsTransmitted 
                          << "/" << metric.nonCriticalPacketsReceived << "\n");
                SAFE_LOG_IF(2, "  Recovery events: " << metric.recoveryTimes.size() << "\n");
                SAFE_LOG_IF(2, "  Current PDR: " << (metric.pdr * 100) << "% (Window: " 
                          << (metric.GetCurrentWindowPDR() * 100) << "%)\n");
                SAFE_LOG_IF(2, "  Is in failure state: " << (metric.isInFailureState ? "Yes" : "No") << "\n");
                SAFE_LOG_IF(2, "  Failure/Recovery count: " << metric.failureCount 
                          << "/" << metric.recoveryCount << "\n");
            }
        }
    }

    LinkMetrics GetLinkMetrics(uint8_t linkId) const {
        return (linkId < m_numLinks) ? m_metrics[linkId] : LinkMetrics{};
    }

    std::vector<LinkMetrics> GetAllMetrics() const {
        return m_metrics;
    }
    
    void SetResultLogger(std::shared_ptr<ResultLogger> logger) {
        m_resultLogger = logger;
    }
    
    std::shared_ptr<ResultLogger> GetResultLogger() const {
        return m_resultLogger;
    }

    void SetGlobalTidParameters(uint32_t emergencyTids, uint32_t criticalTids) {
        m_emergencyTids = emergencyTids;
        m_criticalTids = criticalTids;
        SAFE_LOG_IF(2, "LinkQualityMonitor: Set TID parameters - Emergency: " << emergencyTids 
                  << ", Critical: " << criticalTids << "\n");
    }

private:
    uint8_t m_numLinks;
    double m_pdrThreshold;
    std::vector<LinkMetrics> m_metrics;
    Time m_monitoringStartTime;
    std::shared_ptr<ResultLogger> m_resultLogger;
    uint32_t m_emergencyTids = 0;
    uint32_t m_criticalTids = 0;
};

// ================== UNIVERSAL SLA DEVIATION FRAMEWORK ==================
/**
 * @struct SLAContract
 * @brief Service Level Agreement contract definition for monitoring
 * 
 * This structure defines the performance thresholds and measurement parameters
 * that constitute a Service Level Agreement for different types of traffic flows.
 * Each contract specifies the maximum acceptable delay and error rate within
 * a defined measurement window.
 */
struct SLAContract {
    double delayThreshold;      ///< DTH_f: Maximum acceptable end-to-end delay (milliseconds)
    double errorThreshold;      ///< ErrorTH_f: Maximum acceptable packet loss rate (0-100%)
    uint32_t packetWindow;      ///< T_SLA: Measurement window size in packets for SLA evaluation
    std::string contractName;   ///< Human-readable identifier for this SLA contract
};

/**
 * @class UniversalSLADeviationMonitor
 * @brief Comprehensive SLA compliance monitoring and violation detection system
 * 
 * This class implements a sophisticated Service Level Agreement monitoring system
 * that tracks compliance across multiple traffic flows and different classes.
 * It provides real-time SLA deviation detection, trend analysis, and detailed
 * compliance reporting for MLO network performance evaluation.
 * 
 * KEY FEATURES:
 * - Multi-flow SLA tracking with individual contract enforcement
 * - Real-time violation detection with configurable thresholds
 * - Time-windowed analysis for responsive compliance measurement
 * - Statistical analysis of SLA deviations with trend identification
 * - Support for different traffic classes (Emergency, Critical, Best Effort)
 * - Integration with link quality monitoring for root cause analysis
 * 
 * MEASUREMENT METHODOLOGY:
 * - Sliding window approach for continuous SLA compliance assessment
 * - Weighted deviation calculation based on traffic priority
 * - Historical trend analysis for predictive SLA management
 * - Configurable measurement intervals and reporting granularity
 */
class UniversalSLADeviationMonitor {
public:
    /**
     * @struct FlowSLAMetrics
     * @brief Per-flow SLA compliance metrics and violation tracking
     * 
     * Maintains detailed SLA compliance statistics for individual traffic flows,
     * enabling granular analysis of performance and violation patterns.
     */
    struct FlowSLAMetrics {
        uint32_t totalPackets = 0;
        uint32_t packetsExceedingThreshold = 0;
        
        // Time-windowed error tracking for proper moving average
        std::deque<double> errorPercentageHistory;  // Error_f(t) values
        
        double currentErrorPercentage = 0.0;        // Error_f(t)
        double movingAverageError = 0.0;            // Avg_error_f(t)
        double slaDeviation = 0.0;                  // SLA_dev_f(t) = max(Avg_error_f(t) - ErrorTH_f, 0)
        
        SLAContract assignedContract;
        bool isCritical = false;
        
        // Enhanced delay tracking
        double totalDelay = 0.0;
        uint32_t delayMeasurements = 0;
        double averageDelay = 0.0;
    
        std::deque<bool> packetWindowResults;        // Sliding window of packet SLA compliance (true=met, false=exceeded)
        uint32_t packetsInCurrentWindow = 0;
        uint32_t exceedingInCurrentWindow = 0;
        
        // Legacy time-based window support (for backward compatibility)
        Time lastWindowUpdate = Seconds(0);
        Time windowDuration = Seconds(1.0);
        
        // Scheduler performance tracking
        std::map<std::string, uint32_t> schedulerPacketCount;
        std::map<std::string, double> schedulerSLADeviation;
    };

    UniversalSLADeviationMonitor(uint32_t maxFlows) : m_maxFlows(maxFlows) {
        InitializeContracts();
        m_startTime = Simulator::Now();
    }
    
    void SetResultLogger(std::shared_ptr<ResultLogger> logger) {
        m_resultLogger = logger;
    }
    
    void SetSimulationParameters(const std::string& strategy, const std::string& protocol,
                                uint32_t nodeCount, uint32_t tidCount, uint32_t criticalTids,
                                uint32_t emergencyTids, double simTime, uint32_t payloadSize) {
        m_strategy = strategy;
        m_protocol = protocol;
        m_nodeCount = nodeCount;
        m_tidCount = tidCount;
        m_criticalTids = criticalTids;
        m_emergencyTids = emergencyTids;
        m_simTime = simTime;
        m_payloadSize = payloadSize;
    }
    

    void SetFlowContract(uint8_t tid, const std::string& contractLevel = "auto") {
        // Use proper QoS-based criticality determination
        bool isCritical = QoSUtils::IsCriticalTraffic(tid, m_emergencyTids, m_criticalTids);
        SLAContract contract = SelectContractForFlow(isCritical, contractLevel);
        
        m_flowMetrics[tid] = FlowSLAMetrics();
        m_flowMetrics[tid].assignedContract = contract;
        m_flowMetrics[tid].isCritical = isCritical;
        m_flowMetrics[tid].lastWindowUpdate = Simulator::Now();
        m_flowMetrics[tid].windowDuration = Seconds(1.0); 
    }
    
    void UpdateFlowMetrics(uint8_t tid, bool packetSuccess, double delay, Time currentTime, 
                          const std::string& schedulerName = "unknown") {
        if (m_flowMetrics.find(tid) == m_flowMetrics.end()) {
            SetFlowContract(tid, "auto");
        }
        
        static uint32_t totalCallCount = 0;
        totalCallCount++;
        if (totalCallCount <= 5 || totalCallCount % 20 == 0) {
            SAFE_LOG_IF(2, "🔍 UpdateFlowMetrics called #" << totalCallCount << " - TID:" << (uint32_t)tid 
                      << ", Success:" << (packetSuccess ? "✓" : "✗") << ", Delay:" << delay << "ms\n");
        }
        
        SAFE_LOG_IF(3, "📋 Contract for TID " << (uint32_t)tid << ": " 
                  << m_flowMetrics[tid].assignedContract.contractName 
                  << " (Critical: " << (m_flowMetrics[tid].isCritical ? "Yes" : "No") << ")\n");
        
        
        FlowSLAMetrics& metrics = m_flowMetrics[tid];
        const SLAContract& contract = metrics.assignedContract;
        
        // Always process packets, regardless of success or delay
        metrics.totalPackets++;
        metrics.packetsInCurrentWindow++;
        metrics.schedulerPacketCount[schedulerName]++;
        
        bool slaCompliant;
        if (packetSuccess) {
            // Use minimum delay if needed
            double processedDelay = (delay > 0) ? delay : 0.1;
            
            metrics.totalDelay += processedDelay;
            metrics.delayMeasurements++;
            metrics.averageDelay = metrics.totalDelay / metrics.delayMeasurements;
            
            // Check SLA compliance
            slaCompliant = (processedDelay <= contract.delayThreshold);
            
            SAFE_LOG_IF(3, "TID " << (uint32_t)tid << " packet: delay=" << processedDelay 
                      << "ms, threshold=" << contract.delayThreshold 
                      << "ms, newAvg=" << metrics.averageDelay 
                      << "ms, compliant=" << (slaCompliant ? "YES" : "NO") << "\n");
        } else {
            // Failed packets always violate SLA
            slaCompliant = false;
        }
        
        if (!slaCompliant) {
            metrics.packetsExceedingThreshold++;
            metrics.exceedingInCurrentWindow++;
        }
        
        // Add packet result to sliding window
        metrics.packetWindowResults.push_back(slaCompliant);
        
        SAFE_LOG_IF(3, "📈 Packet " << metrics.totalPackets << " added to TID " << (uint32_t)tid 
                  << " window (size: " << metrics.packetWindowResults.size() 
                  << "/" << contract.packetWindow << ", SLA " 
                  << (slaCompliant ? "✓" : "✗") << ")\n");
        
        // Update packet-based moving average
        UpdateMovingAverage(tid, currentTime);
        
        // Update time-windowed moving average (Paper's methodology)
        UpdateMovingAveragePaperMethod(tid, currentTime);
        
        // Force immediate SLA deviation calculation after each update
        CalculateDirectSLADeviation(tid);
    }
    
    void UpdateMovingAveragePaperMethod(uint8_t tid, Time currentTime) {
        FlowSLAMetrics& metrics = m_flowMetrics[tid];
        
        // Check if we need to process a time window
        if (currentTime - metrics.lastWindowUpdate >= metrics.windowDuration) {
            if (metrics.packetsInCurrentWindow > 0) {
                // Calculate Error_f(t) for this window
                double windowErrorPercentage = 
                    (double)metrics.exceedingInCurrentWindow / metrics.packetsInCurrentWindow * 100.0;
                
                // Add to time-windowed history
                metrics.errorPercentageHistory.push_back(windowErrorPercentage);
                
                // Maintain sliding window (keep last 10 windows for moving average)
                while (metrics.errorPercentageHistory.size() > 10) {
                    metrics.errorPercentageHistory.pop_front();
                }
                
                // Calculate Avg_error_f(t) using time-weighted moving average
                CalculatePaperMovingAverage(tid);
            }
            
            // Reset window counters
            metrics.packetsInCurrentWindow = 0;
            metrics.exceedingInCurrentWindow = 0;
            metrics.lastWindowUpdate = currentTime;
        }
    }
    
    double CalculateDirectSLADeviation(uint8_t tid) {
        FlowSLAMetrics& metrics = m_flowMetrics[tid];
        const SLAContract& contract = metrics.assignedContract;
        
        SAFE_LOG_IF(3, "🔍 SLA Check TID " << (uint32_t)tid 
                  << ": avgDelay=" << metrics.averageDelay 
                  << "ms, threshold=" << contract.delayThreshold 
                  << "ms, measurements=" << metrics.delayMeasurements << "\n");
        
        if (metrics.delayMeasurements == 0) {
            // If no delay measurements exist but packets were processed, there's a data collection issue
            SAFE_LOG_IF(2, "WARNING: TID " << (uint32_t)tid << " has no delay measurements but contract exists\n");
            metrics.slaDeviation = -1.0; // Sentinel value indicating no data
            return -1.0; // Return sentinel value to distinguish from perfect compliance
        }
        
        double actualAvgDelay = metrics.averageDelay;
        double thresholdDelay = contract.delayThreshold;
        
        if (actualAvgDelay > thresholdDelay) {
            // Calculate percentage deviation above threshold
            metrics.slaDeviation = ((actualAvgDelay - thresholdDelay) / thresholdDelay) * 100.0;
            
            SAFE_LOG_IF(2, "⚠️ SLA VIOLATION TID " << (uint32_t)tid 
                      << ": " << actualAvgDelay << "ms > " << thresholdDelay 
                      << "ms = " << metrics.slaDeviation << "% deviation\n");
        } else {
            metrics.slaDeviation = 0.0;
        }
        
        return metrics.slaDeviation;
    }

    void CalculatePaperMovingAverage(uint8_t tid) {
        FlowSLAMetrics& metrics = m_flowMetrics[tid];
        // const SLAContract& contract = metrics.assignedContract;  // Comment out unused variable
        
        if (metrics.errorPercentageHistory.empty()) {
            metrics.movingAverageError = 0.0;
            metrics.slaDeviation = 0.0;
            return;
        }
        
        double sum = 0.0;
        uint32_t count = metrics.errorPercentageHistory.size();
        
        for (double errorPercent : metrics.errorPercentageHistory) {
            sum += errorPercent;
        }
        
        // Avg_error_f(t) = average of Error_f(t) values
        metrics.movingAverageError = count > 0 ? sum / count : 0.0;
        
        // Use direct SLA deviation calculation instead of error-based formula
        CalculateDirectSLADeviation(tid);
    }

    double CalculateSLADeviation(uint8_t tid) {
        if (m_flowMetrics.find(tid) == m_flowMetrics.end()) {
            return -1.0; // Sentinel value indicating no data available
        }    
        return CalculateDirectSLADeviation(tid);
    }

    double GetOverallSLADeviation() const {
        return GetOverallSLADeviationAverage();
    }

    double GetOverallSLADeviationAverage() const {
        SAFE_LOG_IF(3, "🔍 GetOverallSLADeviationAverage called - Flow metrics size: " << m_flowMetrics.size() << "\n");
        
        if (m_flowMetrics.empty()) {
            SAFE_LOG_IF(3, "⚠️ GetOverallSLADeviationAverage: No flow metrics available\n");
            return -1.0; // Sentinel value indicating no data available
        }
        
        double totalDeviation = 0.0;
        uint32_t flowCount = 0;
        
        for (const auto& [tid, metrics] : m_flowMetrics) {
            if (metrics.delayMeasurements > 0) {
                double tidDeviation = ((const_cast<UniversalSLADeviationMonitor*>(this))->CalculateDirectSLADeviation(tid));
                totalDeviation += tidDeviation;
                flowCount++;
                
                SAFE_LOG_IF(3, "🔍 TID " << (uint32_t)tid << " - Contract: " << metrics.assignedContract.contractName
                          << ", AvgDelay: " << metrics.averageDelay << "ms"
                          << ", Threshold: " << metrics.assignedContract.delayThreshold << "ms"
                          << ", Deviation: " << tidDeviation << "%\n");
            }
        }
        
        double result = flowCount > 0 ? totalDeviation / flowCount : 0.0;
        SAFE_LOG_IF(1, "📊 Overall SLA Deviation: " << std::fixed << std::setprecision(2) 
                  << result << "% (from " << flowCount << " flows)\n");
        return result;
    }

    double GetNonCriticalSLADeviation() const {
        double totalDeviation = 0.0;
        uint32_t nonCriticalFlows = 0;
        
        for (const auto& [tid, metrics] : m_flowMetrics) {
            if (!metrics.isCritical && metrics.delayMeasurements > 0) {
                double tidDeviation = const_cast<UniversalSLADeviationMonitor*>(this)->CalculateDirectSLADeviation(tid);
                totalDeviation += tidDeviation;
                nonCriticalFlows++;
                SAFE_LOG_IF(3, "NonCritical TID " << (uint32_t)tid << ": AvgDelay=" << metrics.averageDelay 
                          << "ms, Threshold=" << metrics.assignedContract.delayThreshold 
                          << "ms, Deviation=" << tidDeviation << "%\n");
            }
        }
        double result = nonCriticalFlows > 0 ? totalDeviation / nonCriticalFlows : 0.0;
        SAFE_LOG_IF(1, "📊 Non Critical SLA Deviation: " << std::fixed << std::setprecision(2) 
                  << result << "% (from " << nonCriticalFlows << " flows)\n");
        return result;
    }

    double GetCriticalHighSLADeviation() const {
        double totalDeviation = 0.0;
        uint32_t criticalHighFlows = 0;
        
        for (const auto& [tid, metrics] : m_flowMetrics) {
            if (metrics.assignedContract.contractName == "CriticalHigh" && 
                metrics.delayMeasurements > 0) {
                
                // Force fresh calculation
                double tidDeviation = const_cast<UniversalSLADeviationMonitor*>(this)->CalculateDirectSLADeviation(tid);
                totalDeviation += tidDeviation;
                criticalHighFlows++;
                
                SAFE_LOG_IF(3, "📊 CriticalHigh TID " << (uint32_t)tid 
                          << ": avg=" << metrics.averageDelay << "ms"
                          << ", threshold=" << metrics.assignedContract.delayThreshold << "ms"
                          << ", deviation=" << tidDeviation << "%\n");
            }
        }
        
        double result = criticalHighFlows > 0 ? totalDeviation / criticalHighFlows : 0.0;
        
        SAFE_LOG_IF(1, "📊 Critical High SLA Deviation: " << std::fixed << std::setprecision(2) 
                  << result << "% (from " << criticalHighFlows << " flows)\n");
        
        return result;
    }

    double GetCriticalBasicSLADeviation() const {
        double totalDeviation = 0.0;
        uint32_t criticalBasicFlows = 0;
        
        for (const auto& [tid, metrics] : m_flowMetrics) {
            if (metrics.isCritical && metrics.delayMeasurements > 0 && 
                metrics.assignedContract.contractName == "CriticalBasic") {
                double tidDeviation = const_cast<UniversalSLADeviationMonitor*>(this)->CalculateDirectSLADeviation(tid);
                totalDeviation += tidDeviation;
                criticalBasicFlows++;
                SAFE_LOG_IF(3, "CriticalBasic TID " << (uint32_t)tid << ": AvgDelay=" << metrics.averageDelay 
                          << "ms, Threshold=" << metrics.assignedContract.delayThreshold 
                          << "ms, Deviation=" << tidDeviation << "%\n");
            }
        }

        double result = criticalBasicFlows > 0 ? totalDeviation / criticalBasicFlows : 0.0;
        SAFE_LOG_IF(1, "📊 Critical Basic SLA Deviation: " << std::fixed << std::setprecision(2) 
                  << result << "% (from " << criticalBasicFlows << " flows)\n");
        return result;
    }

    void PrintDetailedSLADebug() const {
        SAFE_LOG_IF(1, "\n=== DETAILED SLA DEBUG ===\n");
        
        for (const auto& [tid, metrics] : m_flowMetrics) {
            SAFE_LOG_IF(1, "TID " << (uint32_t)tid << ":\n");
            SAFE_LOG_IF(1, "  Contract: " << metrics.assignedContract.contractName 
                      << " (threshold: " << metrics.assignedContract.delayThreshold << "ms)\n");
            SAFE_LOG_IF(1, "  Average delay: " << metrics.averageDelay << "ms\n");
            SAFE_LOG_IF(1, "  Current SLA deviation: " << metrics.slaDeviation << "%\n");
            
            if (metrics.averageDelay > metrics.assignedContract.delayThreshold) {
                SAFE_LOG_IF(1, "  ⚠️ SHOULD HAVE VIOLATION: " << metrics.averageDelay 
                          << "ms > " << metrics.assignedContract.delayThreshold << "ms\n");
            }
        }
    }
    
    // Getter for flow metrics (needed for validation)
    const std::map<uint8_t, FlowSLAMetrics>& GetFlowMetrics() const {
        return m_flowMetrics;
    }


private:
    uint32_t m_maxFlows;
    std::map<uint8_t, FlowSLAMetrics> m_flowMetrics;
    std::map<std::string, SLAContract> m_contractDefinitions;
    Time m_startTime;
    std::shared_ptr<ResultLogger> m_resultLogger;
    std::string m_strategy = "unknown";
    std::string m_protocol = "UDP";
    uint32_t m_nodeCount = 0;
    uint32_t m_tidCount = 0;
    uint32_t m_criticalTids = 0;
    uint32_t m_emergencyTids = 0;
    double m_simTime = 0.0;
    uint32_t m_payloadSize = 0;

    void InitializeContracts() {
        m_contractDefinitions["CriticalHigh"] = {1.0, 1.0, 10, "CriticalHigh"};
        m_contractDefinitions["CriticalBasic"] = {50.0, 5.0, 10, "CriticalBasic"};
        m_contractDefinitions["NonCritical"] = {100.0, 10.0, 10, "NonCritical"};
        
        SAFE_LOG_IF(2, "SLA Contracts initialized:\n");
        SAFE_LOG_IF(2, "  CriticalHigh: " << m_contractDefinitions["CriticalHigh"].delayThreshold << "ms\n");
        SAFE_LOG_IF(2, "  CriticalBasic: " << m_contractDefinitions["CriticalBasic"].delayThreshold << "ms\n");
        SAFE_LOG_IF(2, "  NonCritical: " << m_contractDefinitions["NonCritical"].delayThreshold << "ms\n");
    }

    void UpdateMovingAverage(uint8_t tid, Time currentTime) {
        FlowSLAMetrics& metrics = m_flowMetrics[tid];
        const SLAContract& contract = metrics.assignedContract;
        
        SAFE_LOG_IF(3, "Window check - TID:" << (uint32_t)tid << " has " << metrics.packetWindowResults.size() 
                  << "/" << contract.packetWindow << " packets in window\n");
        if (metrics.packetWindowResults.size() >= contract.packetWindow) {
            // Calculate error percentage for current window
            uint32_t exceedingCount = 0;
            for (bool result : metrics.packetWindowResults) {
                if (!result) exceedingCount++;
            }
            
            double windowErrorPercentage = (double)exceedingCount / contract.packetWindow * 100.0;
            SAFE_LOG_IF(3, "📊 SLA Window Complete - TID " << (uint32_t)tid 
                      << ": " << contract.packetWindow << " packets processed"
                      << ", " << exceedingCount << " exceeded SLA"
                      << ", Error Rate: " << std::fixed << std::setprecision(2) << windowErrorPercentage << "%"
                      << " (Contract: " << contract.contractName << ")"
                      << " at " << currentTime.GetSeconds() << "s\n");
            
            
            // Update history for moving average calculation
            metrics.errorPercentageHistory.push_back(windowErrorPercentage);
            
            while (metrics.errorPercentageHistory.size() > 100) {
                metrics.errorPercentageHistory.pop_front();
            }
            metrics.packetWindowResults.pop_front();
            
            // Calculate updated SLA deviation using direct method
            CalculateDirectSLADeviation(tid);
        }

        if (currentTime - metrics.lastWindowUpdate >= Seconds(1.0)) {
            metrics.packetsInCurrentWindow = 0;
            metrics.exceedingInCurrentWindow = 0;
            metrics.lastWindowUpdate = currentTime;
        }
    }

    SLAContract SelectContractForFlow(bool isCritical, const std::string& level) {
        if (level != "auto") {
            if (m_contractDefinitions.find(level) != m_contractDefinitions.end()) {
                return m_contractDefinitions[level];
            }
        }
        
        if (isCritical) {
            return m_contractDefinitions["CriticalHigh"];
        } else {
            return m_contractDefinitions["NonCritical"];
        }
    }
};

// ================== STRATEGY BASE CLASS ==================
class LinkMappingStrategy {
public:
    virtual ~LinkMappingStrategy() = default;
    
    virtual uint8_t SelectLink(uint8_t tid, bool isCritical = false) = 0;
    virtual void UpdateLinkMetrics(uint8_t linkId, uint32_t bytes, bool success, double delay = 0, uint8_t tid = 0) = 0;
    virtual std::vector<double> GetLinkUsage() const = 0;
    virtual std::vector<double> GetLinkThroughput() const = 0;
    virtual void PrintConfiguration() const = 0;
    virtual void SetLinkQualityMonitor(std::shared_ptr<LinkQualityMonitor> monitor) {
        m_linkMonitor = monitor;
    }
    
    virtual void SetSLADeviationMonitor(std::shared_ptr<UniversalSLADeviationMonitor> monitor) {
        m_slaMonitor = monitor;
    }
    
    
    virtual double GetOverallSLADeviation() const {
        return m_slaMonitor ? m_slaMonitor->GetOverallSLADeviation() : 0.0;
    }
    
    
    virtual double GetNonCriticalSLADeviation() const {
        return m_slaMonitor ? m_slaMonitor->GetNonCriticalSLADeviation() : 0.0;
    }

    virtual double GetCriticalHighSLADeviation() const {
        return m_slaMonitor ? m_slaMonitor->GetCriticalHighSLADeviation() : 0.0;
    }

    virtual double GetCriticalBasicSLADeviation() const {
        return m_slaMonitor ? m_slaMonitor->GetCriticalBasicSLADeviation() : 0.0;
    }
    
    struct SLAThresholds {
        double pdrThreshold;      // PDR requirement (0.0-1.0)
        double latencyThreshold;  // Max latency in ms
        double jitterThreshold;   // Max jitter in ms
    };
    
    // Get SLA tier-based thresholds based on TID and criticality
    SLAThresholds GetSLAThresholds(uint8_t tid, bool isCritical) const {
        SLAThresholds thresholds;
        
        if (tid < m_emergencyTids) {
            // Emergency TIDs: Ultra-critical (CriticalHigh)
            thresholds.pdrThreshold = 99;     // 99% PDR required
            thresholds.latencyThreshold = 1.0;   // 1ms max latency
            thresholds.jitterThreshold = 0.5;   // 0.5ms max jitter
        } else if (tid < (m_emergencyTids + m_criticalTids)) {
            // Critical TIDs: Standard critical (CriticalBasic)
            thresholds.pdrThreshold = 95;     // 95% PDR required
            thresholds.latencyThreshold = 50.0; // 50ms max latency
            thresholds.jitterThreshold = 10.0;  // 10ms max jitter
        } else {
            // Normal TIDs: Non-critical
            thresholds.pdrThreshold = 90;     // 90% PDR required (relaxed)
            thresholds.latencyThreshold = 100.0; // 100ms max latency (relaxed)
            thresholds.jitterThreshold = 20.0;   // 20ms max jitter (relaxed)
        }
        
        return thresholds;
    }

    // GLOBAL RELIABILITY SCORE FUNCTION - Used by all schedulers for consistent reporting
    double CalculateGlobalReliabilityScore(uint8_t linkId, uint8_t tid, bool isCritical, bool success, double delay) {
        if (!m_linkMonitor) return 0.5; // Default fallback
        
        auto metrics = m_linkMonitor->GetAllMetrics();
        if (linkId >= metrics.size()) return 0.5;
        
        const auto& metric = metrics[linkId];
        
        // Get appropriate thresholds for this TID type
        SLAThresholds thresholds = GetSLAThresholds(tid, isCritical);
        
        // Determine TID type based on global parameters
        bool isEmergencyTid = (tid < m_emergencyTids);
        bool isCriticalTid = (tid >= m_emergencyTids && tid < (m_emergencyTids + m_criticalTids));
        
        double reliabilityScore = 0.0;
        
        if (isEmergencyTid || isCriticalTid) {
            // Emergency and Critical TIDs: Multi-factor scoring using TID-specific thresholds
            double pdrScore = metric.pdr / thresholds.pdrThreshold; // Normalize PDR to 0-1 range
            double delayScore = (delay > 0) ?  (1.0 - delay / thresholds.latencyThreshold) : 1.0;
            double jitterScore = (1.0 - metric.jitter / thresholds.jitterThreshold);
            
            // Weighted combination: 0.6*PDR + 0.3*delay + 0.1*jitter
            reliabilityScore = pdrScore * 0.6 + delayScore * 0.3 + jitterScore * 0.1;
        } else {
            // Normal (Non-Critical) TIDs: Pure PDR scoring
            reliabilityScore = metric.pdr / thresholds.pdrThreshold; // 1.0 * PDR
        }
        
        return std::max(0.0, std::min(1.0, reliabilityScore)); // Clamp to [0,1] range
    }
    

public:
    // ADD this method to LinkMappingStrategy:
    virtual void SetGlobalTidParameters(uint32_t emergencyTids, uint32_t criticalTids) {
        m_emergencyTids = emergencyTids;
        m_criticalTids = criticalTids;
    }
    
protected:
    std::shared_ptr<LinkQualityMonitor> m_linkMonitor;
    std::shared_ptr<UniversalSLADeviationMonitor> m_slaMonitor;
    uint32_t m_emergencyTids = 0;
    uint32_t m_criticalTids = 0;
    mutable std::mutex m_tidMutex;
};

// ================== RELIABILITY-AWARE STRATEGY ==================
class ReliabilityAwareStrategy : public LinkMappingStrategy {
public:
    ReliabilityAwareStrategy(uint8_t numLinks) : m_numLinks(numLinks), m_lastLink(0) {
        // Initialize link load counters
        for(uint8_t i = 0; i < m_numLinks; i++) {
            m_linkLoad[i] = 0;
        }
        
        // Initialize dynamic frequency band weights - will be updated based on channel conditions
        UpdateChannelWeights(); // Set initial weights
    }

    uint8_t SelectLink(uint8_t tid, bool isCritical) override {
        std::lock_guard<std::mutex> lock(m_loadMutex);  // Thread-safe access
        
        if (!m_linkMonitor) {
            // Fallback with frequency priority: 6GHz (2) → 5GHz (1) → 2.4GHz (0)
            // Prefer higher frequency bands in round-robin fashion
            static const uint8_t preferredOrder[3] = {2, 1, 0}; // 6GHz, 5GHz, 2.4GHz
            static uint8_t orderIndex = 0;
            uint8_t selectedLink = preferredOrder[orderIndex % 3];
            orderIndex++;
            return m_lastLink = selectedLink;
        }
        
        auto metrics = m_linkMonitor->GetAllMetrics();
        uint8_t bestLink = 0;
        double bestScore = 0.0;
        
        // Calculate weighted reliability score for each link
        for (uint8_t linkId = 0; linkId < m_numLinks; linkId++) {
            if (linkId >= metrics.size()) continue;
            
            const auto& metric = metrics[linkId];
            double reliabilityScore = CalculateReliabilityScore(linkId, tid, isCritical, metric);
            
            // Apply dynamic channel weights based on current conditions
            UpdateChannelWeights(); // Update weights based on current metrics
            auto weightIt = m_linkWeights.find(linkId);
            double weight = (weightIt != m_linkWeights.end()) ? weightIt->second : 1.0;
            double weightedScore = reliabilityScore * weight;
            
            if (weightedScore > bestScore) {
                bestScore = weightedScore;
                bestLink = linkId;
            }
        }
        
        // Log weighted selection for debugging
        const char* frequencies[] = {"2.4GHz", "5GHz", "6GHz"};
        SAFE_LOG_IF(2, "ReliabilityAware: Selected " << frequencies[bestLink] << " (link " << (uint32_t)bestLink 
                  << ") with weighted score " << bestScore << " for TID " << (uint32_t)tid << "\n");
        
        return bestLink;
    }

    void UpdateLinkMetrics(uint8_t linkId, uint32_t bytes, bool success, double delay, uint8_t tid = 0) override {
        {
            std::lock_guard<std::mutex> lock(m_loadMutex);
            if (linkId < m_numLinks) {
                m_linkLoad[linkId] += bytes;
            }
        }
        
        if (m_linkMonitor) {
            bool isCritical = QoSUtils::IsCriticalTraffic(tid, m_emergencyTids, m_criticalTids);
            m_linkMonitor->UpdateLinkMetrics(linkId, success, delay, bytes, tid, false, isCritical);
        }
        
        if (m_slaMonitor && tid < 255) {
            double adjustedDelay = (delay > 0) ? delay : 0.1; // Ensure minimum delay
            m_slaMonitor->UpdateFlowMetrics(tid, success, adjustedDelay, Simulator::Now(), "Reliability");
        }
        
        // Use proper QoS-based criticality determination
        bool isCritical = QoSUtils::IsCriticalTraffic(tid, m_emergencyTids, m_criticalTids);
        
        // Calculate and log per-packet reliability score for analysis
        if (m_linkMonitor) {
            auto metrics = m_linkMonitor->GetAllMetrics();
            if (linkId < metrics.size()) {
                double reliabilityScore = CalculateReliabilityScore(linkId, tid, isCritical, metrics[linkId]);
                
                SAFE_LOG_IF(3, "[ReliabilityAware] TID=" << std::setw(2) << (uint32_t)tid 
                          << " → Link=" << (uint32_t)linkId 
                          << " | Critical=" << (isCritical ? "Yes" : "No ")
                          << " | Success=" << (success ? "Yes" : "No ")
                          << " | Delay=" << std::setw(6) << std::fixed << std::setprecision(2) << delay << "ms"
                          << " | Score=" << std::setprecision(3) << reliabilityScore << "\n");
            }
        }
    }

    std::vector<double> GetLinkUsage() const override {
        uint64_t total = 0;
        
        // Calculate total bytes across all links
        for(uint8_t i = 0; i < m_numLinks; i++) {
            auto it = m_linkLoad.find(i);
            if (it != m_linkLoad.end()) {
                total += it->second;
            }
        }
        
        std::vector<double> usage;
        for(uint8_t i = 0; i < m_numLinks; i++) {
            auto it = m_linkLoad.find(i);
            uint64_t linkBytes = (it != m_linkLoad.end()) ? it->second : 0;
            
            if (total > 0) {
                usage.push_back((static_cast<double>(linkBytes) / total) * 100.0);
            } else {
                usage.push_back(0.0);
            }
        }
        
        SAFE_LOG_IF(3, "ReliabilityAware Link usage: Link0=" << usage[0] << "%, Link1=" << usage[1] 
                  << "%, Link2=" << usage[2] << "% (total=" << total << " bytes)\n");
        
        return usage;
    }
    
    std::vector<double> GetLinkThroughput() const override {
        if (!m_linkMonitor) {
            return std::vector<double>(m_numLinks, 0.0);
        }
        
        auto metrics = m_linkMonitor->GetAllMetrics();
        std::vector<double> throughput(m_numLinks);
        
        for (uint8_t i = 0; i < m_numLinks; i++) {
            if (i < metrics.size()) {
                throughput[i] = metrics[i].throughputMbps; // Use actual throughput from LinkMetrics
            } else {
                throughput[i] = 0.0;
            }
        }
        return throughput;
    }

    void PrintConfiguration() const override {
        SAFE_LOG_IF(2, "┌─ ReliabilityAware Strategy Configuration ─────────┐\n");
        SAFE_LOG_IF(2, "│  Links: " << std::setw(2) << (uint32_t)m_numLinks 
                  << " | Emergency TIDs: " << std::setw(2) << m_emergencyTids 
                  << " | Critical TIDs: " << std::setw(2) << m_criticalTids << "   │\n");
        SAFE_LOG_IF(2, "└───────────────────────────────────────────────────┘\n");
    }

    void SetTidParameters(uint32_t emergencyTids, uint32_t criticalTids) {
        m_emergencyTids = emergencyTids;
        m_criticalTids = criticalTids;
    }

private:
    // Internal method that implements CalculateGlobalReliabilityScore formula
    double CalculateReliabilityScore(uint8_t linkId, uint8_t tid, bool isCritical, const LinkQualityMonitor::LinkMetrics& metric) {
        // Get appropriate thresholds for this TID type
        SLAThresholds thresholds = GetSLAThresholds(tid, isCritical);
        
        // Calculate normalized scores based on TID-specific thresholds
        double pdrScore = (metric.pdr / thresholds.pdrThreshold); // Normalize PDR to 0-1 range
        double delayScore = (metric.avgDelay > 0) ? std::max(0.0, 1.0 - metric.avgDelay / thresholds.latencyThreshold) : 1.0;
        double jitterScore = std::max(0.0, 1.0 - metric.jitter / thresholds.jitterThreshold);
        
        // Determine TID type for scoring approach
        bool isEmergencyTid = (tid < m_emergencyTids);
        bool isCriticalTid = (tid >= m_emergencyTids && tid < (m_emergencyTids + m_criticalTids));
        
        double reliabilityScore = 0.0;
        
        if (isEmergencyTid || isCriticalTid) {
            // Emergency and Critical TIDs: Multi-factor scoring using TID-specific thresholds
            // Weighted combination: 0.6*PDR + 0.3*delay + 0.1*jitter
            reliabilityScore = pdrScore * 0.6 + delayScore * 0.3 + jitterScore * 0.1;
        } else {
            // Normal (Non-Critical) TIDs: Pure PDR scoring
            reliabilityScore = pdrScore; // 1.0 * PDR
        }
        
        return std::max(0.0, std::min(1.0, reliabilityScore)); // Clamp to [0,1] range
    }
    
    void UpdateChannelWeights() {
        // Dynamic channel quality assessment based on real metrics
        if (!m_linkMonitor) {
            // Fallback to conservative static weights if no monitor available
            m_linkWeights[0] = 0.6;  // 2.4GHz baseline
            m_linkWeights[1] = 1.0;  // 5GHz baseline
            m_linkWeights[2] = 1.2;  // 6GHz baseline
            return;
        }
        
        auto metrics = m_linkMonitor->GetAllMetrics();
        
        for (uint8_t linkId = 0; linkId < m_numLinks && linkId < metrics.size(); linkId++) {
            const auto& metric = metrics[linkId];
            
            // Calculate dynamic weight based on actual channel performance
            double baseWeight = GetBaseChannelWeight(linkId);
            double qualityFactor = 1.0;
            
            // Adjust based on PDR (Packet Delivery Ratio)
            if (metric.pdr > 0.95) {
                qualityFactor *= 1.2; // Excellent PDR
            } else if (metric.pdr > 0.90) {
                qualityFactor *= 1.0; // Good PDR
            } else if (metric.pdr > 0.80) {
                qualityFactor *= 0.8; // Fair PDR
            } else {
                qualityFactor *= 0.5; // Poor PDR - penalize heavily
            }
            
            // Adjust based on delay performance
            if (metric.avgDelay <= 1.0) {
                qualityFactor *= 1.1; // Low delay
            } else if (metric.avgDelay <= 5.0) {
                qualityFactor *= 1.0; // Moderate delay
            } else {
                qualityFactor *= 0.7; // High delay - penalize
            }
            
            // Adjust based on jitter
            if (metric.jitter <= 0.5) {
                qualityFactor *= 1.05; // Low jitter
            } else if (metric.jitter > 2.0) {
                qualityFactor *= 0.9; // High jitter - slight penalty
            }
            
            // Final dynamic weight
            m_linkWeights[linkId] = baseWeight * qualityFactor;
            
            // Clamp to reasonable range
            m_linkWeights[linkId] = std::max(0.1, std::min(2.0, m_linkWeights[linkId]));
        }
    }
    
    double GetBaseChannelWeight(uint8_t linkId) const {
        // Base weights reflect theoretical channel capacity and typical congestion
        switch(linkId) {
            case 0: return 0.6;  // 2.4GHz: Lower capacity, more congested
            case 1: return 1.0;  // 5GHz: Balanced performance
            case 2: return 1.2;  // 6GHz: Higher capacity, less congested
            default: return 1.0; // Default balanced weight
        }
    }

private:
    uint8_t m_numLinks;
    uint8_t m_lastLink;
    std::map<uint8_t, uint64_t> m_linkLoad;
    std::map<uint8_t, double> m_linkWeights;  // Dynamic frequency band weights
    uint32_t m_emergencyTids = 0;
    uint32_t m_criticalTids = 0;
    mutable std::mutex m_loadMutex;  // Thread safety for load operations
};

// ================== ROUND ROBIN STRATEGY ==================
class RoundRobinStrategy : public LinkMappingStrategy {
public:
    RoundRobinStrategy(uint8_t numLinks) : m_numLinks(numLinks), m_lastLink(0), m_lastTidAssigned(0), m_maxTids(8) {
        for(uint8_t i = 0; i < m_numLinks; i++) {
            m_linkLoad[i] = 0;
        }
    }

    uint8_t SelectLink(uint8_t tid, bool isCritical) override {
        std::lock_guard<std::mutex> lock(m_loadMutex);  // Thread-safe access
        uint8_t link = m_lastLink;
        m_lastLink = (m_lastLink + 1) % m_numLinks;
        return link;
    }

    void UpdateLinkMetrics(uint8_t linkId, uint32_t bytes, bool success, double delay, uint8_t tid = 0) override {
        {
            std::lock_guard<std::mutex> lock(m_loadMutex);
            if (linkId < m_numLinks) {
                m_linkLoad[linkId] += bytes;
            }
        }
        
        if (m_linkMonitor) {
            bool isCritical = QoSUtils::IsCriticalTraffic(tid, m_emergencyTids, m_criticalTids);
            m_linkMonitor->UpdateLinkMetrics(linkId, success, delay, bytes, tid, false, isCritical);
        }
        
        if (m_slaMonitor && tid < 255) {  // Remove delay > 0 restriction
            double adjustedDelay = (delay > 0) ? delay : 0.1; // Ensure minimum delay
            m_slaMonitor->UpdateFlowMetrics(tid, success, adjustedDelay, Simulator::Now(), "RoundRobin");
        }
        
        // Use proper QoS-based criticality determination
        bool isCritical = QoSUtils::IsCriticalTraffic(tid, m_emergencyTids, m_criticalTids);
        
        // Calculate per-packet reliability score for analysis
        double reliabilityScore = CalculateGlobalReliabilityScore(linkId, tid, isCritical, success, delay);
        
        SAFE_LOG_IF(3, "[RoundRobin] TID=" << std::setw(2) << (uint32_t)tid
                  << " → Link=" << (uint32_t)linkId 
                  << " | Success=" << (success ? "Yes" : "No ")
                  << " | Delay=" << std::setw(6) << std::fixed << std::setprecision(2) << delay << "ms"
                  << " | Score=" << std::setprecision(3) << reliabilityScore << "\n");
    }

    std::vector<double> GetLinkUsage() const override {
        uint64_t total = 0;
        
        for(uint8_t i = 0; i < m_numLinks; i++) {
            auto it = m_linkLoad.find(i);
            if (it != m_linkLoad.end()) {
                total += it->second;
            }
        }
        
        std::vector<double> usage;
        for(uint8_t i = 0; i < m_numLinks; i++) {
            auto it = m_linkLoad.find(i);
            uint64_t linkBytes = (it != m_linkLoad.end()) ? it->second : 0;
            
            if (total > 0) {
                usage.push_back((static_cast<double>(linkBytes) / total) * 100.0);
            } else {
                // Equal distribution potential when no traffic
                usage.push_back(0.0);
            }
        }
        
        SAFE_LOG_IF(3, "RoundRobin Link Usage: Link0=" << usage[0] << "%, Link1=" << usage[1] 
                  << "%, Link2=" << usage[2] << "% (total=" << total << " bytes)\n");
        
        return usage;
    }

    std::vector<double> GetLinkThroughput() const override {
        std::vector<double> throughput;
        if (m_linkMonitor) {
            auto metrics = m_linkMonitor->GetAllMetrics();
            for (const auto& metric : metrics) {
                throughput.push_back(metric.throughputMbps);
            }
        } else {
            for(uint8_t i = 0; i < m_numLinks; i++) {
                throughput.push_back(0.0);
            }
        }
        return throughput;
    }

    void PrintConfiguration() const override {
        SAFE_LOG_IF(2, "┌─ RoundRobin Strategy Configuration ────────────────┐\n");
        SAFE_LOG_IF(2, "│  Links: " << std::setw(2) << (uint32_t)m_numLinks << " | Mode: Cyclic Distribution           │\n");
        SAFE_LOG_IF(2, "└───────────────────────────────────────────────────┘\n");
    }

private:
    uint8_t m_numLinks;
    uint8_t m_lastLink;
    mutable uint8_t m_lastTidAssigned;
    uint8_t m_maxTids;
    std::map<uint8_t, uint64_t> m_linkLoad;
    mutable std::mutex m_loadMutex;  // Thread safety for load operations
};

// ================== GREEDY LOAD BALANCER ==================
class GreedyLoadBalancer : public LinkMappingStrategy {
public:
    GreedyLoadBalancer(uint8_t numLinks) : m_numLinks(numLinks), m_lastTidAssigned(0), m_maxTids(8) {
        for(uint8_t i = 0; i < m_numLinks; i++) {
            m_linkLoad[i] = 0;
            m_pendingLoad[i] = 0;    // Track pending transmissions
        }
    }

    uint8_t SelectLink(uint8_t tid, bool isCritical) override {
        std::lock_guard<std::mutex> lock(m_loadMutex);  // Thread-safe access
        
        // TRUE GREEDY: Always select link with minimum normalized load
        uint8_t bestLink = 0;
        double minNormalizedLoad = GetNormalizedLoad(0);
        
        // Find link with minimum normalized load (greedy selection)
        for (uint8_t i = 1; i < m_numLinks; i++) {
            double currentNormalizedLoad = GetNormalizedLoad(i);
            
            // Pure greedy: choose minimum normalized load
            if (currentNormalizedLoad < minNormalizedLoad) {
                minNormalizedLoad = currentNormalizedLoad;
                bestLink = i;
            }
        }
        
        // Dynamic logging for variable number of links
        std::string loadStatus = "[";
        for(uint8_t i = 0; i < m_numLinks; i++) {
            if(i > 0) loadStatus += ", ";
            loadStatus += "Link" + std::to_string(i) + "=" + std::to_string(GetLinkLoad(i));
        }
        loadStatus += "]";
        
        SAFE_LOG_IF(3, "[GreedyLoadBalancer] TID=" << std::setw(2) << (uint32_t)tid << " → Link=" << (uint32_t)bestLink 
                  << " | Load=" << std::fixed << std::setprecision(3) << minNormalizedLoad 
                  << " " << loadStatus << "\n");
        
        return bestLink;
    }

    void UpdateLinkMetrics(uint8_t linkId, uint32_t bytes, bool success, double delay, uint8_t tid = 0) override {
        {
            std::lock_guard<std::mutex> lock(m_loadMutex);
            // Only add to actual load after transmission attempt
            m_linkLoad[linkId] += bytes;
        }
        
        if (m_linkMonitor) {
            bool isCritical = QoSUtils::IsCriticalTraffic(tid, m_emergencyTids, m_criticalTids);
            m_linkMonitor->UpdateLinkMetrics(linkId, success, delay, bytes, tid, false, isCritical);
        }
        
        // Fixed: Always update SLA metrics with adjusted delay
        if (m_slaMonitor && tid < 255) {  
            double adjustedDelay = (delay > 0) ? delay : 0.1; // Ensure minimum delay
            m_slaMonitor->UpdateFlowMetrics(tid, success, adjustedDelay, Simulator::Now(), "Greedy");
        }
        
        // Use proper QoS-based criticality determination
        bool isCritical = QoSUtils::IsCriticalTraffic(tid, m_emergencyTids, m_criticalTids);
        
        double reliabilityScore = CalculateGlobalReliabilityScore(linkId, tid, isCritical, success, delay);
        
        SAFE_LOG_IF(3, "[GreedyLoadBalancer] TID=" << std::setw(2) << (uint32_t)tid
                  << " → Link=" << (uint32_t)linkId 
                  << " | Success=" << (success ? "Yes" : "No ")
                  << " | Delay=" << std::setw(6) << std::fixed << std::setprecision(2) << delay << "ms"
                  << " | Score=" << std::setprecision(3) << reliabilityScore << "\n");
    }

    std::vector<double> GetLinkUsage() const override {
        uint64_t total = 0;
        
        for(uint8_t i = 0; i < m_numLinks; i++) {
            auto it = m_linkLoad.find(i);
            if (it != m_linkLoad.end()) {
                total += it->second;
            }
        }
        
        std::vector<double> usage;
        for(uint8_t i = 0; i < m_numLinks; i++) {
            auto it = m_linkLoad.find(i);
            uint64_t linkBytes = (it != m_linkLoad.end()) ? it->second : 0;
            
            if (total > 0) {
                usage.push_back((static_cast<double>(linkBytes) / total) * 100.0);
            } else {
                usage.push_back(0.0);
            }
        }
        
        SAFE_LOG_IF(3, "GreedyLoadBalancer Link Usage: Link0=" << usage[0] << "%, Link1=" << usage[1] 
                  << "%, Link2=" << usage[2] << "% (total=" << total << " bytes)\n");
        
        return usage;
    }

    std::vector<double> GetLinkThroughput() const override {
        std::vector<double> throughput;
        if (m_linkMonitor) {
            auto metrics = m_linkMonitor->GetAllMetrics();
            for (const auto& metric : metrics) {
                throughput.push_back(metric.throughputMbps);
            }
        } else {
            for(uint8_t i = 0; i < m_numLinks; i++) {
                throughput.push_back(0.0);
            }
        }
        return throughput;
    }

    void PrintConfiguration() const override {
        SAFE_LOG_IF(2, "┌─ GreedyLoadBalancer Strategy Configuration ───────┐\n");
        SAFE_LOG_IF(2, "│  Links: " << std::setw(2) << (uint32_t)m_numLinks << " | Mode: Minimum-Load Selection       │\n");
        SAFE_LOG_IF(2, "└───────────────────────────────────────────────────┘\n");
    }

private:
    uint8_t m_numLinks;
    mutable uint8_t m_lastTidAssigned;
    uint8_t m_maxTids;
    std::map<uint8_t, uint64_t> m_linkLoad;
    std::map<uint8_t, uint64_t> m_pendingLoad;    // Track pending transmissions
    mutable std::mutex m_loadMutex;               // Thread safety for load operations
    
    // Helper methods for greedy selection
    uint64_t GetLinkLoad(uint8_t linkId) const {
        if (linkId >= m_numLinks) {
            SAFE_LOG_IF(1, "ERROR: Invalid linkId " << (uint32_t)linkId << " >= " << (uint32_t)m_numLinks << "\n");
            return 0; // Safe fallback
        }
        auto loadIt = m_linkLoad.find(linkId);
        return (loadIt != m_linkLoad.end()) ? loadIt->second : 0;
    }
    
    double GetLinkCapacity(uint8_t linkId) const {
        if (linkId >= m_numLinks) {
            SAFE_LOG_IF(1, "ERROR: Invalid linkId " << (uint32_t)linkId << " >= " << (uint32_t)m_numLinks << "\n");
            return 200.0e6; // Safe fallback capacity
        }
        
        // Channel capacity based on frequency band (realistic IEEE 802.11be values)
        // Link mapping: 0=2.4GHz, 1=5GHz, 2=6GHz
        switch(linkId) {
            case 0: return 100.0e6;  // 2.4GHz: ~100 Mbps (more congested)
            case 1: return 300.0e6;  // 5GHz: ~300 Mbps (balanced)
            case 2: return 500.0e6;  // 6GHz: ~500 Mbps (less congested)
            default: return 200.0e6; // Default capacity
        }
    }
    
    double GetNormalizedLoad(uint8_t linkId) const {
        if (linkId >= m_numLinks) {
            SAFE_LOG_IF(1, "ERROR: Invalid linkId " << (uint32_t)linkId << " >= " << (uint32_t)m_numLinks << "\n");
            return 1.0; // Safe fallback - treat as fully loaded
        }
        
        uint64_t rawLoad = GetLinkLoad(linkId);
        double capacity = GetLinkCapacity(linkId);
        
        // Comprehensive error handling
        if (capacity <= 0) {
            SAFE_LOG_IF(2, "WARNING: Invalid capacity " << capacity << " for link " << (uint32_t)linkId << "\n");
            return 1.0; // Treat as fully loaded if capacity unknown
        }
        
        // Normalize: current_load_bytes / (capacity_bits_per_sec * time_window)
        // For simplicity, use a 1-second time window
        double normalizedLoad = static_cast<double>(rawLoad * 8) / capacity; // Convert bytes to bits
        
        // Clamp to [0,1] range with bounds checking
        if (normalizedLoad < 0.0) {
            SAFE_LOG_IF(2, "WARNING: Negative normalized load " << normalizedLoad << " for link " << (uint32_t)linkId << "\n");
            return 0.0;
        }
        
        return std::min(1.0, normalizedLoad);
    }
};

// ================== SLA-MLO STRATEGY (PAPER IMPLEMENTATION) ==================
class SLA_MLO_Strategy : public LinkMappingStrategy {
public:
    struct FlowSLA {
        double delayThreshold;    // DTH_f in milliseconds
        double errorThreshold;    // ErrorTH_f as percentage (0-100)
        uint32_t packetWindow;   // T_SLA measurement window in packets
        
        // Legacy time window support
        Time slaWindow;          // Deprecated T_SLA measurement window
    };
    
    struct FlowMetrics {
        uint32_t slaFollowed = 0;
        uint32_t slaNotFollowed = 0;
        double slaBreach = 0.0;
        std::map<uint8_t, double> avgDelayPerLink;      // AvgDelay_Link_f,i
        std::map<uint8_t, double> instantDelayPerLink;  // Delay_f,i
        Time lastUpdate = Seconds(0);
    };

    SLA_MLO_Strategy(uint8_t numLinks) : m_numLinks(numLinks), m_alpha(0.8) {
        for(uint8_t i = 0; i < m_numLinks; i++) {
            m_linkLoad[i] = 0;
        }
        m_uniformRandom = CreateObject<UniformRandomVariable>();
        m_uniformRandom->SetAttribute("Min", DoubleValue(0.0));
        m_uniformRandom->SetAttribute("Max", DoubleValue(1.0));
    }

    void SetFlowSLA(uint8_t tid, double delayTh, double errorTh, uint32_t packetWindow) {
        m_flowSLAs[tid] = {delayTh, errorTh, packetWindow, Seconds(1.0)};
        if (m_flowMetrics.find(tid) == m_flowMetrics.end()) {
            m_flowMetrics[tid] = FlowMetrics();
            for(uint8_t i = 0; i < m_numLinks; i++) {
                m_flowMetrics[tid].avgDelayPerLink[i] = 0.0;
                m_flowMetrics[tid].instantDelayPerLink[i] = 0.0;
            }
        }
    }

    uint8_t SelectLink(uint8_t tid, bool isCritical) override {
        std::lock_guard<std::mutex> lock(m_metricsMutex);  // Thread-safe access
        
        if (m_flowSLAs.find(tid) == m_flowSLAs.end()) {
            uint8_t link = m_lastLink;
            m_lastLink = (m_lastLink + 1) % m_numLinks;
            return link;
        }

        const FlowSLA& sla = m_flowSLAs[tid];
        const FlowMetrics& metrics = m_flowMetrics[tid];
        
        std::vector<double> probabilities(m_numLinks, 0.0);
        
        std::vector<uint8_t> L_below, L_above;
        for (uint8_t i = 0; i < m_numLinks; i++) {
            double avgDelay = metrics.avgDelayPerLink.at(i);
            if (avgDelay < sla.delayThreshold) {
                L_below.push_back(i);
            } else {
                L_above.push_back(i);
            }
        }
        
        if (metrics.slaBreach <= sla.errorThreshold) {
            for (uint8_t i = 0; i < m_numLinks; i++) {
                probabilities[i] = 1.0 / m_numLinks;
            }
        } else {
            if (!L_below.empty()) {
                for (uint8_t i : L_below) {
                    probabilities[i] = 1.0 / L_below.size();
                }
                for (uint8_t i : L_above) {
                    probabilities[i] = 0.0;
                }
            } else {
                double sum_inv_delay = 0.0;
                for (uint8_t i = 0; i < m_numLinks; i++) {
                    double avgDelay = metrics.avgDelayPerLink.at(i);
                    if (avgDelay > 0) {
                        sum_inv_delay += 1.0 / avgDelay;
                    }
                }
                
                if (sum_inv_delay > 0) {
                    for (uint8_t i = 0; i < m_numLinks; i++) {
                        double avgDelay = metrics.avgDelayPerLink.at(i);
                        if (avgDelay > 0) {
                            probabilities[i] = (1.0 / avgDelay) / sum_inv_delay;
                        } else {
                            probabilities[i] = 1.0 / m_numLinks;
                        }
                    }
                } else {
                    for (uint8_t i = 0; i < m_numLinks; i++) {
                        probabilities[i] = 1.0 / m_numLinks;
                    }
                }
            }
        }
        
        double random = m_uniformRandom->GetValue();
        double cumProb = 0.0;
        for (uint8_t i = 0; i < m_numLinks; i++) {
            cumProb += probabilities[i];
            if (random < cumProb) {
                return i;
            }
        }
        return m_numLinks - 1;
    }

    void UpdateLinkMetrics(uint8_t linkId, uint32_t bytes, bool success, double delay, uint8_t tid = 0) override {
        {
            std::lock_guard<std::mutex> lock(m_metricsMutex);
            if (linkId < m_numLinks) {
                m_linkLoad[linkId] += bytes;
            }
        }
        
        if (m_linkMonitor) {
            bool isCritical = QoSUtils::IsCriticalTraffic(tid, m_emergencyTids, m_criticalTids);
            m_linkMonitor->UpdateLinkMetrics(linkId, success, delay, bytes, tid, false, isCritical);
        }
        
        if (m_slaMonitor && tid < 255) {  // Remove delay > 0 restriction
            double adjustedDelay = (delay > 0) ? delay : 0.1; // Ensure minimum delay
            m_slaMonitor->UpdateFlowMetrics(tid, success, adjustedDelay, Simulator::Now(), "SLA-MLO");
        }
        
        // Update SLA metrics for the specific TID (not all flows)
        if (success && delay > 0 && tid < 255) {
            if (m_flowSLAs.find(tid) != m_flowSLAs.end() && m_flowMetrics.find(tid) != m_flowMetrics.end()) {
                const FlowSLA& sla = m_flowSLAs[tid];
                FlowMetrics& metrics = m_flowMetrics[tid];
                
                // Update instantaneous delay for this link
                metrics.instantDelayPerLink[linkId] = delay;
                
                // Update exponential weighted moving average of delay
                if (metrics.avgDelayPerLink[linkId] == 0.0) {
                    metrics.avgDelayPerLink[linkId] = delay;
                } else {
                    metrics.avgDelayPerLink[linkId] = 
                        metrics.avgDelayPerLink[linkId] * m_alpha + delay * (1.0 - m_alpha);
                }
                
                // Track SLA compliance for this packet
                if (delay <= sla.delayThreshold) {
                    metrics.slaFollowed++;
                } else {
                    metrics.slaNotFollowed++;
                }
                
                // Calculate SLA breach percentage
                uint32_t total = metrics.slaFollowed + metrics.slaNotFollowed;
                if (total > 0) {
                    metrics.slaBreach = (static_cast<double>(metrics.slaNotFollowed) / total) * 100.0;
                } else {
                    metrics.slaBreach = 0.0; // No packets processed yet
                }
                
                // Update last measurement time
                metrics.lastUpdate = Simulator::Now();
                
                SAFE_LOG_IF(4, "[SLA-MLO] TID=" << std::setw(2) << (uint32_t)tid
                          << " → Link=" << (uint32_t)linkId 
                          << " | Delay=" << std::setw(6) << std::fixed << std::setprecision(2) << delay << "ms"
                          << " | Threshold=" << std::setw(6) << std::setprecision(1) << sla.delayThreshold << "ms"
                          << " | Breach=" << std::setprecision(2) << metrics.slaBreach << "%"
                          << " | AvgDelay=" << std::setprecision(2) << metrics.avgDelayPerLink[linkId] << "ms\n");
            }
        }
        
        // Use proper QoS-based criticality determination
        bool isCritical = QoSUtils::IsCriticalTraffic(tid, m_emergencyTids, m_criticalTids);
        
        // Calculate per-packet reliability score for analysis
        // Use first available TID or default to 0
        uint8_t activeTid = 0;
        if (!m_flowSLAs.empty()) {
            activeTid = m_flowSLAs.begin()->first;
        }
        double reliabilityScore = CalculateGlobalReliabilityScore(linkId, activeTid, isCritical, success, delay);
        
        SAFE_LOG_IF(3, "[SLA-MLO] TID=" << std::setw(2) << (uint32_t)activeTid 
                  << " → Link=" << (uint32_t)linkId 
                  << " | Success=" << (success ? "Yes" : "No ")
                  << " | Delay=" << std::setw(6) << std::fixed << std::setprecision(2) << delay << "ms"
                  << " | Score=" << std::setprecision(3) << reliabilityScore << "\n");
    }

    std::vector<double> GetLinkUsage() const override {
        uint64_t total = 0;
        
        for(uint8_t i = 0; i < m_numLinks; i++) {
            auto it = m_linkLoad.find(i);
            if (it != m_linkLoad.end()) {
                total += it->second;
            }
        }
        
        std::vector<double> usage;
        for(uint8_t i = 0; i < m_numLinks; i++) {
            auto it = m_linkLoad.find(i);
            uint64_t linkBytes = (it != m_linkLoad.end()) ? it->second : 0;
            
            if (total > 0) {
                usage.push_back((static_cast<double>(linkBytes) / total) * 100.0);
            } else {
                usage.push_back(0.0);
            }
        }
        
        SAFE_LOG_IF(3, "SLA_MLO_Strategy Link Usage: Link0=" << usage[0] << "%, Link1=" << usage[1] 
                  << "%, Link2=" << usage[2] << "% (total=" << total << " bytes)\n");
        
        return usage;
    }

    std::vector<double> GetLinkThroughput() const override {
        std::vector<double> throughput;
        if (m_linkMonitor) {
            auto metrics = m_linkMonitor->GetAllMetrics();
            for (const auto& metric : metrics) {
                throughput.push_back(metric.throughputMbps);
            }
        } else {
            for(uint8_t i = 0; i < m_numLinks; i++) {
                throughput.push_back(0.0);
            }
        }
        return throughput;
    }

    void PrintConfiguration() const override {
        SAFE_LOG_IF(2, "┌─ SLA-MLO Strategy Configuration ──────────────────┐\n");
        SAFE_LOG_IF(2, "│  Links: " << std::setw(2) << (uint32_t)m_numLinks 
                  << " | SLA Flows: " << std::setw(2) << m_flowSLAs.size() 
                  << " | Emergency: " << std::setw(2) << m_emergencyTids 
                  << " | Critical: " << std::setw(2) << m_criticalTids << "  │\n");
        SAFE_LOG_IF(2, "└───────────────────────────────────────────────────┘\n");
    }

private:
    uint8_t m_numLinks;
    double m_alpha;
    uint8_t m_lastLink = 0;
    std::map<uint8_t, uint64_t> m_linkLoad;
    std::map<uint8_t, FlowSLA> m_flowSLAs;
    std::map<uint8_t, FlowMetrics> m_flowMetrics;
    Ptr<UniformRandomVariable> m_uniformRandom;
    uint32_t m_emergencyTids = 0;
    uint32_t m_criticalTids = 0;
    mutable std::mutex m_metricsMutex;  // Thread safety for metrics operations
};

// ================== ENHANCED UDP CLIENT - PROPER DELAY INJECTION ==================
class EnhancedUdpClient : public Application {
public:
    static TypeId GetTypeId() {
        static TypeId tid = TypeId("ns3::EnhancedUdpClient")
            .SetParent<Application>()
            .AddConstructor<EnhancedUdpClient>()
            .AddAttribute("RemoteAddress", "Destination address",
                        AddressValue(),
                        MakeAddressAccessor(&EnhancedUdpClient::m_peerAddress),
                        MakeAddressChecker())
            .AddAttribute("RemotePort", "Destination port",
                        UintegerValue(9),
                        MakeUintegerAccessor(&EnhancedUdpClient::m_port),
                        MakeUintegerChecker<uint16_t>())
            .AddAttribute("PacketSize", "Payload size in bytes",
                        UintegerValue(1024),
                        MakeUintegerAccessor(&EnhancedUdpClient::m_pktSize),
                        MakeUintegerChecker<uint32_t>())
            .AddAttribute("Interval", "Packet interval",
                        TimeValue(MilliSeconds(10)),
                        MakeTimeAccessor(&EnhancedUdpClient::m_interval),
                        MakeTimeChecker())
            .AddAttribute("MaxPackets", "Maximum packets to send",
                        UintegerValue(0),
                        MakeUintegerAccessor(&EnhancedUdpClient::m_maxPackets),
                        MakeUintegerChecker<uint32_t>())
            .AddAttribute("Duplication", "Enable packet duplication",
                        BooleanValue(false),
                        MakeBooleanAccessor(&EnhancedUdpClient::m_enableDuplication),
                        MakeBooleanChecker());
        return tid;
    }

    EnhancedUdpClient() : m_packetsSent(0), m_tid(0), m_sequenceNumber(0) {}

    void SetStrategy(std::shared_ptr<LinkMappingStrategy> strategy) { 
        m_strategy = strategy; 
    }
    
    void SetTid(uint8_t tid) { 
        m_tid = tid; 
    }
    
    void SetLinkMonitor(std::shared_ptr<LinkQualityMonitor> linkMonitor) {
        m_linkMonitor = linkMonitor;
    }

private:
    void StartApplication() override {
        if (!m_socket) {
            m_socket = Socket::CreateSocket(GetNode(), UdpSocketFactory::GetTypeId());
            m_socket->Bind();
            m_socket->Connect(m_peerAddress);
        }
        m_startTime = Simulator::Now();
        ScheduleTransmit(Seconds(0));
    }

    void StopApplication() override {
        if (m_sendEvent.IsPending()) {
            Simulator::Cancel(m_sendEvent);
        }
        if (m_socket) {
            m_socket->Close();
        }
    }

    void ScheduleTransmit(Time dt) {
        m_sendEvent = Simulator::Schedule(dt, &EnhancedUdpClient::Send, this);
    }

    void Send() {
        if (m_maxPackets == 0 || m_packetsSent < m_maxPackets) {
            uint8_t linkId = m_strategy->SelectLink(m_tid);
            
            Ptr<Packet> packet = Create<Packet>(m_pktSize);
            
            // Enhanced packet tagging with proper sequence number and timestamp
            TidTag tidTag;
            tidTag.SetTid(m_tid);
            packet->AddPacketTag(tidTag);
            
            MLOLinkTag linkTag;
            linkTag.SetLinkId(linkId);
            packet->AddPacketTag(linkTag);
            
            
            // Add sequence number and timestamp for proper tracking
            SeqTsHeader seqTs;
            seqTs.SetSeq(m_sequenceNumber++);
            packet->AddHeader(seqTs);
            
            // Send packet 
            Time currentTime = Simulator::Now();
            int actualSent = m_socket->Send(packet);
            bool packetTransmitted = (actualSent > 0);
            
            // CRITICAL FIX: Track transmitted packets in LinkQualityMonitor for proper PDR calculation
            if (m_linkMonitor) {
                bool isCritical = QoSUtils::IsCriticalTraffic(m_tid, 0, 0); // Will be updated by strategy
                // Record packet as transmitted (not yet received)
                m_linkMonitor->UpdateLinkMetrics(linkId, false, -1.0, m_pktSize, m_tid, false, isCritical);
                SAFE_LOG_IF(3, "EnhancedUdpClient: Recorded packet transmission - TID=" << (uint32_t)m_tid
                          << ", Link=" << (uint32_t)linkId << ", Size=" << m_pktSize << "\n");
            }
            
            // Update strategy metrics for link usage calculation
            if (m_strategy) {
                m_strategy->UpdateLinkMetrics(linkId, m_pktSize, packetTransmitted, 0, m_tid);
            }
            
            SAFE_LOG_IF(3, "EnhancedUdpClient: Sent packet " << m_sequenceNumber-1 
                      << " on link " << (uint32_t)linkId 
                      << " (TID: " << (uint32_t)m_tid 
                      << ") at " << currentTime.GetSeconds() << "s\n");
            
            // Enhanced duplication logic for reliability improvement
            if (m_enableDuplication) {
                // Duplicate based on packet type and criticality
                bool shouldDuplicate = false;
                uint8_t duplicateCount = 1; // Default: single duplicate
                
                // Determine duplication strategy based on TID criticality
                if (m_tid < 2) {
                    // Emergency TIDs: Duplicate every packet on 2 backup links
                    shouldDuplicate = true;
                    duplicateCount = 2;
                } else if (m_tid < 6) {
                    // Critical TIDs: Duplicate every other packet on 1 backup link
                    shouldDuplicate = (m_packetsSent % 2 == 0);
                    duplicateCount = 1;
                } else {
                    // Normal TIDs: Duplicate every 4th packet for light redundancy
                    shouldDuplicate = (m_packetsSent % 4 == 0);
                    duplicateCount = 1;
                }
                
                if (shouldDuplicate) {
                    for (uint8_t i = 0; i < duplicateCount; i++) {
                        uint8_t dupLink = (linkId + i + 1) % 3;
                        Ptr<Packet> dupPacket = packet->Copy();
                        
                        MLOLinkTag dupLinkTag;
                        dupLinkTag.SetLinkId(dupLink);
                        dupPacket->ReplacePacketTag(dupLinkTag);
                        
                        // Add duplicate marker tag
                        DuplicationTag dupTag;
                        dupTag.SetOriginalLink(linkId);
                        dupTag.SetDuplicateLink(dupLink);
                        dupPacket->AddPacketTag(dupTag);
                        
                        m_socket->Send(dupPacket);
                        
                        // Update strategy metrics for duplicate
                        m_strategy->UpdateLinkMetrics(dupLink, m_pktSize, true, 0.1, m_tid);
                        
                        SAFE_LOG_IF(3, "EnhancedUdpClient: TID " << (uint32_t)m_tid 
                                  << " duplicated packet " << m_packetsSent 
                                  << " from link " << (uint32_t)linkId 
                                  << " to backup link " << (uint32_t)dupLink << "\n");
                    }
                }
            }
            
            m_packetsSent++;
            
            if (m_packetsSent < m_maxPackets || m_maxPackets == 0) {
                ScheduleTransmit(m_interval);
            }
        }
    }

    Ptr<Socket> m_socket;
    Address m_peerAddress;
    uint16_t m_port;
    uint32_t m_pktSize;
    Time m_interval;
    uint32_t m_maxPackets;
    bool m_enableDuplication;
    uint32_t m_packetsSent;
    uint8_t m_tid;
    uint32_t m_sequenceNumber;
    EventId m_sendEvent;
    Time m_startTime;
    std::shared_ptr<LinkMappingStrategy> m_strategy;
    std::shared_ptr<LinkQualityMonitor> m_linkMonitor;
};

NS_OBJECT_ENSURE_REGISTERED(EnhancedUdpClient);


// ================== ENHANCED PACKET SINK - PROPER DELAY CALCULATION ==================
class EnhancedPacketSink : public Application {
public:
    static TypeId GetTypeId() {
        static TypeId tid = TypeId("ns3::EnhancedPacketSink")
            .SetParent<Application>()
            .AddConstructor<EnhancedPacketSink>()
            .AddAttribute("Local", "Local address to bind to",
                        AddressValue(),
                        MakeAddressAccessor(&EnhancedPacketSink::m_local),
                        MakeAddressChecker())
            .AddAttribute("Protocol", "Socket protocol",
                        TypeIdValue(UdpSocketFactory::GetTypeId()),
                        MakeTypeIdAccessor(&EnhancedPacketSink::m_protocol),
                        MakeTypeIdChecker());
        return tid;
    }

    EnhancedPacketSink() : m_socket(nullptr), m_totalRx(0), m_expectedTid(0) {}

    void SetLinkMonitor(std::shared_ptr<LinkQualityMonitor> monitor) {
        m_linkMonitor = monitor;
    }
    
    void SetSLAMonitor(std::shared_ptr<UniversalSLADeviationMonitor> monitor) {
        m_slaMonitor = monitor;
    }
    
    void SetResultLogger(std::shared_ptr<ResultLogger> logger) {
        m_resultLogger = logger;
    }
    
    void SetExpectedTid(uint8_t tid) {
        m_expectedTid = tid;
        SAFE_LOG_IF(2, "EnhancedPacketSink: Set expected TID to " << (uint32_t)tid << "\n");
    }

private:
    void StartApplication() override {
        if (!m_socket) {
            m_socket = Socket::CreateSocket(GetNode(), m_protocol);
            m_socket->Bind(m_local);
            
            // CRITICAL FIX: TCP sockets need to Listen() for incoming connections!
            if (m_protocol == TcpSocketFactory::GetTypeId()) {
                m_socket->Listen();
                SAFE_LOG_IF(2, "EnhancedPacketSink: TCP socket listening on " 
                          << InetSocketAddress::ConvertFrom(m_local).GetIpv4() 
                          << ":" << InetSocketAddress::ConvertFrom(m_local).GetPort() << "\n");
            }
            
            m_socket->SetRecvCallback(MakeCallback(&EnhancedPacketSink::HandleRead, this));
        }
        m_startTime = Simulator::Now();
    }

    void StopApplication() override {
        if (m_socket) {
            m_socket->Close();
            m_socket->SetRecvCallback(MakeNullCallback<void, Ptr<Socket>>());
        }
    }

    void HandleRead(Ptr<Socket> socket) {
        Ptr<Packet> packet;
        Address from;
        
        while ((packet = socket->RecvFrom(from))) {
            m_totalRx += packet->GetSize();
            
            // Enhanced packet tag extraction and processing
            Time currentTime = Simulator::Now();
            
            // Extract packet tags
            uint8_t linkId = 0;
            uint8_t tid = 0;
            bool isCritical = false;
            uint32_t sequenceNumber = 0;
            Time sendTime = currentTime; 
            
            // Extract MLO Link Tag
            MLOLinkTag linkTag;
            if (packet->RemovePacketTag(linkTag)) {
                linkId = linkTag.GetLinkId();
            }
            
            // Extract TID Tag
            TidTag tidTag;
            if (packet->RemovePacketTag(tidTag)) {
                tid = tidTag.GetTid();
            }
            
            // CRITICAL FIX: TID validation - only process packets for this sink's TID
            // Special case: expectedTid = 255 means accept all TIDs (UDP sinks)
            if (m_expectedTid != 255 && tid != m_expectedTid) {
                SAFE_LOG_IF(3, "EnhancedPacketSink: Ignoring packet with TID " << (uint32_t)tid 
                          << " (expected TID " << (uint32_t)m_expectedTid << ")\n");
                continue; // Skip this packet as it's not for this sink
            }
            
            // Extract Criticality Tag - MISSING CRITICAL CODE!
            CriticalityTag criticalityTag;
            if (packet->RemovePacketTag(criticalityTag)) {
                isCritical = criticalityTag.GetIsCritical();
            }
            
            // Check for duplication tag
            bool isDuplicate = false;
            uint8_t originalLink = 0;
            DuplicationTag dupTag;
            if (packet->RemovePacketTag(dupTag)) {
                isDuplicate = true;
                originalLink = dupTag.GetOriginalLink();
                SAFE_LOG_IF(3, "EnhancedPacketSink: Received duplicate packet - TID " << (uint32_t)tid 
                          << " from original link " << (uint32_t)originalLink 
                          << " via backup link " << (uint32_t)linkId << "\n");
            }
            
            
            // Proper timestamp extraction based on protocol type
            SeqTsHeader seqTs;
            TimestampTag timestampTag;
            bool foundTimestamp = false;
            
            // Determine protocol based on sink configuration and extract timestamp accordingly
            bool isTcpProtocol = (m_protocol == TcpSocketFactory::GetTypeId());
            
            if (isTcpProtocol) {
                // TCP packets use TimestampTag
                if (packet->RemovePacketTag(timestampTag)) {
                    sendTime = timestampTag.GetTimestamp();
                    foundTimestamp = true;
                    SAFE_LOG_IF(3, "TCP packet timestamp from tag: " << sendTime.GetSeconds() << "s\n");
                }
            } else {
                // UDP packets use SeqTsHeader  
                if (packet->RemoveHeader(seqTs)) {
                    sequenceNumber = seqTs.GetSeq();
                    sendTime = seqTs.GetTs();
                    foundTimestamp = true;
                    SAFE_LOG_IF(3, "UDP packet timestamp from header: " << sendTime.GetSeconds() << "s\n");
                }
            }
            
            // Calculate actual network delay with proper fallback handling
            double totalDelayMs = 0.1; // Default minimum delay
            if (foundTimestamp) {
                double computedDelay = static_cast<double>((currentTime - sendTime).GetMilliSeconds());
                if (computedDelay > 0.0) {
                    totalDelayMs = computedDelay;
                    // Validate computed delay is reasonable
                    if (computedDelay < 0.05) {
                        SAFE_LOG_IF(3, "INFO: Very low delay measured: " << computedDelay << "ms for TID " << (uint32_t)tid << "\n");
                    } else if (computedDelay > 500.0) {
                        SAFE_LOG_IF(2, "WARNING: Very high delay measured: " << computedDelay << "ms for TID " << (uint32_t)tid << "\n");
                    }
                } else {
                    // Handle edge case where computed delay is negative or zero
                    totalDelayMs = 0.1;
                    SAFE_LOG_IF(3, "WARNING: Computed delay was <= 0 (" << computedDelay << "ms), using minimum 0.1ms\n");
                }
            } else {
                // Timestamp not found - use network-based estimation
                totalDelayMs = 1.0 + (rand() % 50) / 10.0; // Random delay between 1-6ms as realistic fallback
                SAFE_LOG_IF(2, "WARNING: No timestamp found for packet TID " << (uint32_t)tid << ", using estimated delay: " << totalDelayMs << "ms\n");
            }
            
            SAFE_LOG_IF(3, "EnhancedPacketSink: Received packet " << sequenceNumber 
                      << " on link " << (uint32_t)linkId 
                      << " (TID: " << (uint32_t)tid 
                      << ", Critical: " << (isCritical ? "Yes" : "No") 
                      << ", Delay: " << totalDelayMs << "ms) at " 
                      << currentTime.GetSeconds() << "s\n");
            
            // Update both LinkQualityMonitor and SLA monitoring
            bool packetSuccess = true; // Packet received successfully
            
            // Update LinkQualityMonitor with proper packet information
            if (m_linkMonitor && tid < 255) {
                m_linkMonitor->UpdateLinkMetrics(linkId, packetSuccess, totalDelayMs, packet->GetSize(), tid, isDuplicate, isCritical);
                SAFE_LOG_IF(2, "EnhancedPacketSink: Updated LinkQualityMonitor - Protocol=" 
                          << (isTcpProtocol ? "TCP" : "UDP") << ", Link=" << (uint32_t)linkId 
                          << ", TID=" << (uint32_t)tid << ", Size=" << packet->GetSize() 
                          << ", Delay=" << totalDelayMs << "ms, Success=true\n");
            }
            
            // Update SLA deviation monitoring for each received packet
            if (m_slaMonitor && tid < 255) {
                m_slaMonitor->UpdateFlowMetrics(tid, packetSuccess, totalDelayMs, currentTime, 
                                              foundTimestamp ? (m_protocol == TcpSocketFactory::GetTypeId() ? "TCP-Sink" : "UDP-Sink") : "Unknown-Sink");
                SAFE_LOG_IF(3, "Updated SLA monitoring: TID=" << (uint32_t)tid 
                          << ", Success=" << (packetSuccess ? "Y" : "N") 
                          << ", Delay=" << totalDelayMs << "ms\n");
            }
            
            // ENHANCED LOGGING: TID-based and Window-based data collection
            if (m_resultLogger) {
                // Calculate metrics for logging
                double currentPDR = m_linkMonitor ? 
                    (m_linkMonitor->GetAllMetrics().size() > linkId ? m_linkMonitor->GetAllMetrics()[linkId].pdr * 100.0 : 0.0) : 0.0;
                double currentAvgDelay = m_linkMonitor ? 
                    (m_linkMonitor->GetAllMetrics().size() > linkId ? m_linkMonitor->GetAllMetrics()[linkId].avgDelay : 0.0) : 0.0;
                double currentJitter = m_linkMonitor ? 
                    (m_linkMonitor->GetAllMetrics().size() > linkId ? m_linkMonitor->GetAllMetrics()[linkId].jitter : 0.0) : 0.0;
                double currentSLADeviation = m_slaMonitor ? m_slaMonitor->CalculateSLADeviation(tid) : 0.0;
                
                // TID-based detailed logging (per packet)
                m_resultLogger->LogTIDData(tid, 
                                         "CurrentStrategy", // Will be set by strategy
                                         m_protocol == TcpSocketFactory::GetTypeId() ? "TCP" : "UDP",
                                         packetSuccess, totalDelayMs, packet->GetSize(), linkId,
                                         currentPDR, currentAvgDelay, currentJitter,
                                         currentSLADeviation, isCritical, 0); // runNumber will be set
                
                // Update window metrics and log if window is complete
                m_resultLogger->UpdateWindowMetrics(tid, packetSuccess, totalDelayMs, packet->GetSize());
                m_resultLogger->LogWindowedData(100); // Default 100-packet window
            }
            
            m_packetsReceived++;
        }
    }

    Ptr<Socket> m_socket;
    Address m_local;
    TypeId m_protocol;
    uint64_t m_totalRx;
    uint32_t m_packetsReceived = 0;
    Time m_startTime;
    uint8_t m_expectedTid;
    std::shared_ptr<LinkQualityMonitor> m_linkMonitor;
    std::shared_ptr<UniversalSLADeviationMonitor> m_slaMonitor;
    std::shared_ptr<ResultLogger> m_resultLogger;
};

NS_OBJECT_ENSURE_REGISTERED(EnhancedPacketSink);

// ================== TCP MLO CONNECTION MANAGER ==================

/**
 * @brief Manages TCP connections across multiple links in MLO setup
 * 
 * This class handles the assignment of TCP connections to specific links
 * and ensures proper packet flow without violating TCP ordering requirements.
 */
class TcpMLOConnectionManager {
public:
    TcpMLOConnectionManager(std::shared_ptr<LinkMappingStrategy> strategy, uint8_t numLinks) 
        : m_strategy(strategy), m_numLinks(numLinks) {}
    
    /**
     * @brief Assigns or reassigns a TID to a link with periodic optimization
     * @param tid Traffic Identifier
     * @param isCritical Whether this is critical traffic
     * @param forceReassignment Force reassignment even if already assigned
     * @return Assigned link ID
     */
    uint8_t AssignConnectionToLink(uint8_t tid, bool isCritical, bool forceReassignment = false) {
        auto it = m_tidToLink.find(tid);
        
        // Check if reassignment is needed
        bool needsReassignment = forceReassignment || (it == m_tidToLink.end()) || 
                                ShouldReassignConnection(tid, isCritical);
        
        if (!needsReassignment && it != m_tidToLink.end()) {
            return it->second; // Return existing assignment
        }
        
        // Release existing assignment if any
        if (it != m_tidToLink.end()) {
            uint8_t oldLink = it->second;
            m_linkConnectionCount[oldLink]--;
            SAFE_LOG_IF(3, "TcpMLOConnectionManager: Reassigning TID " << (uint32_t)tid 
                      << " from Link " << (uint32_t)oldLink << "\n");
        }
        
        // Use strategy to select best link for this connection
        uint8_t selectedLink = m_strategy ? m_strategy->SelectLink(tid, isCritical) : (tid % m_numLinks);
        m_tidToLink[tid] = selectedLink;
        m_linkConnectionCount[selectedLink]++;
        m_lastAssignmentTime[tid] = Simulator::Now();
        
        SAFE_LOG_IF(2, "TcpMLOConnectionManager: Assigned TID " << (uint32_t)tid 
                  << " to Link " << (uint32_t)selectedLink 
                  << " (Critical: " << (isCritical ? "Yes" : "No") << ")\n");
        
        return selectedLink;
    }
    
    /**
     * @brief Gets the assigned link for a TID
     * @param tid Traffic Identifier  
     * @return Assigned link ID or 0 if not found
     */
    uint8_t GetAssignedLink(uint8_t tid) const {
        auto it = m_tidToLink.find(tid);
        return (it != m_tidToLink.end()) ? it->second : 0;
    }
    
    /**
     * @brief Releases a connection when TCP connection closes
     * @param tid Traffic Identifier
     */
    void ReleaseConnection(uint8_t tid) {
        auto it = m_tidToLink.find(tid);
        if (it != m_tidToLink.end()) {
            uint8_t linkId = it->second;
            m_linkConnectionCount[linkId]--;
            m_tidToLink.erase(it);
            SAFE_LOG_IF(2, "TcpMLOConnectionManager: Released TID " << (uint32_t)tid 
                      << " from Link " << (uint32_t)linkId << "\n");
        }
    }
    
    /**
     * @brief Gets connection statistics
     */
    std::map<uint8_t, uint32_t> GetConnectionStats() const {
        return m_linkConnectionCount;
    }
    
private:
    std::shared_ptr<LinkMappingStrategy> m_strategy;
    uint8_t m_numLinks;
    std::map<uint8_t, uint8_t> m_tidToLink;  // TID -> Link mapping
    std::map<uint8_t, uint32_t> m_linkConnectionCount;  // Link -> Connection count
    std::map<uint8_t, Time> m_lastAssignmentTime;  // Track assignment times for reassignment logic
    
    /**
     * @brief Check if a connection should be reassigned to a different link
     * @param tid Traffic Identifier
     * @param isCritical Whether this is critical traffic
     * @return true if reassignment is beneficial
     */
    bool ShouldReassignConnection(uint8_t tid, bool isCritical) const {
        auto timeIt = m_lastAssignmentTime.find(tid);
        if (timeIt == m_lastAssignmentTime.end()) {
            return true; // Never assigned, needs assignment
        }
        
        Time timeSinceAssignment = Simulator::Now() - timeIt->second;
        
        // Reassign critical connections less frequently to maintain stability
        double reassignmentInterval = isCritical ? 5.0 : 2.0; // seconds
        
        if (timeSinceAssignment.GetSeconds() < reassignmentInterval) {
            return false; // Too recent, maintain stability
        }
        
        // For non-critical flows, allow periodic reassignment for load balancing
        return !isCritical;
    }
};

// ================== MLO-AWARE TCP TRAFFIC GENERATOR ==================
/**
 * @brief MLO-aware TCP traffic generator with proper connection management
 * 
 * This implementation uses TcpMLOConnectionManager to assign each TCP connection
 * to a single link for the entire connection duration, ensuring proper TCP ordering,
 * avoiding performance issues, and providing accurate monitoring.
 */
// ================== ENHANCED TCP TRAFFIC GENERATOR ==================
class TcpMLOTrafficGenerator : public Application {
public:
    static TypeId GetTypeId() {
        static TypeId tid = TypeId("ns3::TcpMLOTrafficGenerator")
            .SetParent<Application>()
            .AddConstructor<TcpMLOTrafficGenerator>()
            .AddAttribute("Remote", "The address of the destination",
                        AddressValue(),
                        MakeAddressAccessor(&TcpMLOTrafficGenerator::m_peer),
                        MakeAddressChecker())
            .AddAttribute("MaxBytes", "The total number of bytes to send",
                        UintegerValue(1000000),
                        MakeUintegerAccessor(&TcpMLOTrafficGenerator::m_maxBytes),
                        MakeUintegerChecker<uint64_t>())
            .AddAttribute("SendSize", "The amount of data to send each time",
                        UintegerValue(1460),
                        MakeUintegerAccessor(&TcpMLOTrafficGenerator::m_sendSize),
                        MakeUintegerChecker<uint32_t>(1))
            .AddAttribute("Duplication", "Enable packet duplication for reliability",
                        BooleanValue(false),
                        MakeBooleanAccessor(&TcpMLOTrafficGenerator::m_enableDuplication),
                        MakeBooleanChecker());
        return tid;
    }

    TcpMLOTrafficGenerator() : m_socket(nullptr), m_connected(false), 
                           m_totBytes(0), m_tid(0), m_linkId(0), m_isCritical(false) {}

    ~TcpMLOTrafficGenerator() override {
        // Proper resource cleanup
        if (m_socket) {
            if (m_connected) {
                m_socket->Close();
            }
            m_socket = nullptr;
        }
    }

    void SetTid(uint8_t tid) { m_tid = tid; }
    void SetLinkId(uint8_t linkId) { m_linkId = linkId; }
    void SetIsCritical(bool critical) { m_isCritical = critical; }
    void SetStrategy(std::shared_ptr<LinkMappingStrategy> strategy) { m_strategy = strategy; }
    void SetLinkMonitor(std::shared_ptr<LinkQualityMonitor> monitor) { m_linkMonitor = monitor; }
    void SetSLAMonitor(std::shared_ptr<UniversalSLADeviationMonitor> monitor) { m_slaMonitor = monitor; }

protected:
    void StartApplication() override {
        try {
            m_socket = Socket::CreateSocket(GetNode(), TcpSocketFactory::GetTypeId());
            if (!m_socket) {
                SAFE_LOG_IF(1, "TcpMLOTrafficGenerator: Failed to create socket for TID " << (uint32_t)m_tid << "\n");
                return;
            }
            
            // Simple socket configuration
            m_socket->SetAttribute("SegmentSize", UintegerValue(m_sendSize));
            m_socket->SetAttribute("SndBufSize", UintegerValue(65536));
            m_socket->SetAttribute("RcvBufSize", UintegerValue(65536));
            
            if (m_socket->Bind() == -1) {
                SAFE_LOG_IF(1, "TcpMLOTrafficGenerator: Failed to bind socket for TID " << (uint32_t)m_tid << "\n");
                // Clean up socket on failure
                m_socket->Close();
                m_socket = nullptr;
                return;
            }
            
            // Direct connection first - like working version
            m_socket->Connect(m_peer);
            
            // Set callbacks AFTER connection - key fix!
            m_socket->SetConnectCallback(
                MakeCallback(&TcpMLOTrafficGenerator::ConnectionSucceeded, this),
                MakeCallback(&TcpMLOTrafficGenerator::ConnectionFailed, this));
            m_socket->SetSendCallback(
                MakeCallback(&TcpMLOTrafficGenerator::DataSend, this));
            m_socket->SetCloseCallbacks(
                MakeCallback(&TcpMLOTrafficGenerator::ConnectionClosed, this),
                MakeCallback(&TcpMLOTrafficGenerator::ConnectionClosed, this));
            
            SAFE_LOG_IF(1, "TcpMLOTrafficGenerator: TID " << (uint32_t)m_tid << " started and connecting\n");
            
        } catch (const std::exception& e) {
            SAFE_LOG_IF(1, "TcpMLOTrafficGenerator: Exception in StartApplication for TID " 
                      << (uint32_t)m_tid << ": " << e.what() << "\n");
            // Clean up on exception
            if (m_socket) {
                m_socket->Close();
                m_socket = nullptr;
            }
        }
    }

    void StopApplication() override {
        m_connected = false;
        if (m_socket) {
            m_socket->Close();
            m_socket = nullptr;
        }
    }

private:
    void SendData() {
        if (!m_connected || !m_socket) {
            return;
        }
        
        uint32_t attempts = 0;
        const uint32_t maxAttempts = MLOConstants::LOAD_BALANCER_MAX_ATTEMPTS;
        
        while (m_connected && m_totBytes < m_maxBytes && attempts < maxAttempts) {
            uint32_t toSend = std::min(m_sendSize, static_cast<uint32_t>(m_maxBytes - m_totBytes));
            uint32_t txAvailable = m_socket->GetTxAvailable();
            
            if (txAvailable == 0) {
                break;
            }
            
            toSend = std::min(toSend, txAvailable);
            
            Ptr<Packet> packet = Create<Packet>(toSend);
            
            // Add comprehensive tagging like working version
            TidTag tidTag;
            tidTag.SetTid(m_tid);
            packet->AddPacketTag(tidTag);
            
            MLOLinkTag linkTag;
            linkTag.SetLinkId(m_linkId);
            packet->AddPacketTag(linkTag);
            
            CriticalityTag criticalTag;
            criticalTag.SetIsCritical(m_isCritical);
            packet->AddPacketTag(criticalTag);
            
            // CRITICAL FIX: Add timestamp tag for TCP packets to enable delay measurement
            TimestampTag timestampTag;
            timestampTag.SetTimestamp(Simulator::Now());
            packet->AddPacketTag(timestampTag);
            
            int actual = m_socket->Send(packet);
            if (actual > 0) {
                m_totBytes += actual;
                m_packetsSent++;
                SAFE_LOG_IF(2, "TcpMLOTrafficGenerator: TID " << (uint32_t)m_tid << " sent " << actual << " bytes\n");
                
                if (m_strategy) {
                    // Update link metrics for proper link usage calculation
                    m_strategy->UpdateLinkMetrics(m_linkId, actual, true, 0, m_tid);
                }
            } else {
                break;
            }
            
            attempts++;
        }
        
        if (m_totBytes >= m_maxBytes) {
            SAFE_LOG_IF(1, "TcpMLOTrafficGenerator: TID " << (uint32_t)m_tid << " transfer complete\n");
            m_socket->Close();
            m_connected = false;
        }
    }

    void ConnectionSucceeded(Ptr<Socket> socket) {
        m_connected = true;
        SAFE_LOG_IF(1, "✅ TcpMLOTrafficGenerator: TID " << (uint32_t)m_tid << " connection established at " << Simulator::Now().GetSeconds() << "s\n");
        SendData();
    }

    void ConnectionFailed(Ptr<Socket> socket) {
        m_connected = false;
        SAFE_LOG_IF(1, "❌ TcpMLOTrafficGenerator: TID " << (uint32_t)m_tid << " connection failed at " << Simulator::Now().GetSeconds() << "s\n");
    }

    void ConnectionClosed(Ptr<Socket> socket) {
        m_connected = false;
        SAFE_LOG_IF(2, "TcpMLOTrafficGenerator: TID " << (uint32_t)m_tid << " connection closed\n");
    }

    void DataSend(Ptr<Socket> socket, uint32_t available) {
        if (m_connected && m_totBytes < m_maxBytes) {
            SendData();
        }
    }

    // Member variables
    Ptr<Socket> m_socket;
    Address m_peer;
    bool m_connected;
    uint64_t m_maxBytes;
    uint32_t m_sendSize;
    uint64_t m_totBytes;
    uint8_t m_tid;
    uint8_t m_linkId;
    uint32_t m_packetsSent;
    bool m_isCritical;
    bool m_enableDuplication;
    std::shared_ptr<LinkMappingStrategy> m_strategy;
    
    // Monitors
    std::shared_ptr<LinkQualityMonitor> m_linkMonitor;
    std::shared_ptr<UniversalSLADeviationMonitor> m_slaMonitor;
};

NS_OBJECT_ENSURE_REGISTERED(TcpMLOTrafficGenerator);


// ================== ENHANCED INTERFERENCE GENERATOR ==================
class InterferenceGenerator : public Application {
public:
    static TypeId GetTypeId() {
        static TypeId tid = TypeId("ns3::InterferenceGenerator")
            .SetParent<Application>()
            .AddConstructor<InterferenceGenerator>()
            .AddAttribute("DataRate", "Data rate for interference",
                        DataRateValue(DataRate("10Mbps")),
                        MakeDataRateAccessor(&InterferenceGenerator::m_dataRate),
                        MakeDataRateChecker())
            .AddAttribute("PacketSize", "Packet size",
                        UintegerValue(1500),
                        MakeUintegerAccessor(&InterferenceGenerator::m_packetSize),
                        MakeUintegerChecker<uint32_t>())
            .AddAttribute("OnTime", "On time duration",
                        TimeValue(Seconds(1.5)),
                        MakeTimeAccessor(&InterferenceGenerator::m_onTime),
                        MakeTimeChecker())
            .AddAttribute("OffTime", "Off time duration",
                        TimeValue(Seconds(1.5)),
                        MakeTimeAccessor(&InterferenceGenerator::m_offTime),
                        MakeTimeChecker())
            .AddAttribute("RemoteAddress", "Destination address",
                        AddressValue(),
                        MakeAddressAccessor(&InterferenceGenerator::m_peerAddress),
                        MakeAddressChecker());
        return tid;
    }

    InterferenceGenerator() : m_isOn(false) {}

private:
    void StartApplication() override {
        m_socket = Socket::CreateSocket(GetNode(), UdpSocketFactory::GetTypeId());
        m_socket->Bind();
        m_socket->Connect(m_peerAddress);
        
        SAFE_LOG_IF(2, "InterferenceGenerator started at " << Simulator::Now().GetSeconds() << "s\n");
        ScheduleNextTransition();
    }

    void StopApplication() override {
        if (m_sendEvent.IsPending()) {
            Simulator::Cancel(m_sendEvent);
        }
        if (m_transitionEvent.IsPending()) {
            Simulator::Cancel(m_transitionEvent);
        }
        if (m_socket) {
            m_socket->Close();
        }
        SAFE_LOG_IF(2, "InterferenceGenerator stopped at " << Simulator::Now().GetSeconds() << "s\n");
    }

    void ScheduleNextTransition() {
        if (m_isOn) {
            m_transitionEvent = Simulator::Schedule(m_onTime, 
                &InterferenceGenerator::StopSending, this);
        } else {
            m_transitionEvent = Simulator::Schedule(m_offTime, 
                &InterferenceGenerator::StartSending, this);
        }
    }

    void StartSending() {
        m_isOn = true;
        ScheduleNextPacket();
        ScheduleNextTransition();
        SAFE_LOG_IF(2, "🔥 Interference started at " << Simulator::Now().GetSeconds() << "s\n");
    }

    void StopSending() {
        m_isOn = false;
        if (m_sendEvent.IsPending()) {
            Simulator::Cancel(m_sendEvent);
        }
        ScheduleNextTransition();
        SAFE_LOG_IF(2, "🔇 Interference stopped at " << Simulator::Now().GetSeconds() << "s\n");
    }

    void ScheduleNextPacket() {
        if (m_isOn) {
            Time nextTime = Seconds(m_packetSize * 8.0 / m_dataRate.GetBitRate());
            m_sendEvent = Simulator::Schedule(nextTime, 
                &InterferenceGenerator::SendPacket, this);
        }
    }

    void SendPacket() {
        Ptr<Packet> packet = Create<Packet>(m_packetSize);
        m_socket->Send(packet);
        ScheduleNextPacket();
    }

    Ptr<Socket> m_socket;
    Address m_peerAddress;
    DataRate m_dataRate;
    uint32_t m_packetSize;
    Time m_onTime;
    Time m_offTime;
    bool m_isOn;
    EventId m_sendEvent;
    EventId m_transitionEvent;
};

NS_OBJECT_ENSURE_REGISTERED(InterferenceGenerator);

// ================== HELPER FUNCTIONS ==================
double CalculatePercentile(std::vector<double>& data, double percentile) {
    if (data.empty()) return 0.0;
    
    std::sort(data.begin(), data.end());
    size_t index = static_cast<size_t>(percentile * data.size() / 100.0);
    if (index >= data.size()) index = data.size() - 1;
    
    return data[index];
}


// ================== SLA ASSIGNMENT VALIDATION ==================
void ValidateSLAAssignments(std::shared_ptr<UniversalSLADeviationMonitor> slaMonitor,
                           uint32_t tidCount, uint32_t emergencyTids, uint32_t criticalTids) {
    SAFE_LOG_IF(1, "\n=== SLA ASSIGNMENT VALIDATION ===\n");
    SAFE_LOG_IF(1, "Total TIDs: " << tidCount << "\n");
    if (emergencyTids > 0) {
        SAFE_LOG_IF(1, "Emergency TIDs: " << emergencyTids << " (0 to " << (emergencyTids-1) << ")\n");
    } else {
        SAFE_LOG_IF(1, "Emergency TIDs: 0 (none)\n");
    }
    
    if (criticalTids > 0) {
        SAFE_LOG_IF(1, "Critical TIDs: " << criticalTids << " (" << emergencyTids << " to " << (emergencyTids+criticalTids-1) << ")\n");
    } else {
        SAFE_LOG_IF(1, "Critical TIDs: 0 (none)\n");
    }
    
    uint32_t normalTids = tidCount - emergencyTids - criticalTids;
    if (normalTids > 0) {
        SAFE_LOG_IF(1, "Normal TIDs: " << normalTids << " (" << (emergencyTids+criticalTids) << " to " << (tidCount-1) << ")\n");
    } else {
        SAFE_LOG_IF(1, "Normal TIDs: 0 (none)\n");
    }
    
    // Validate that we have proper distribution
    if (emergencyTids + criticalTids > tidCount) {
        NS_FATAL_ERROR("Emergency TIDs (" << emergencyTids << ") + Critical TIDs (" << criticalTids 
                      << ") exceeds total TID count (" << tidCount << ")");
    }
    
    SAFE_LOG_IF(1, "✅ SLA assignment validation completed\n");
}

// ================== SLA VALIDATION FUNCTIONS ==================
void ValidateSLAResults(std::shared_ptr<UniversalSLADeviationMonitor> slaMonitor,
                       uint32_t tidCount, uint32_t emergencyTids, uint32_t criticalTids) {
    SAFE_LOG_IF(1, "\n=== SLA RESULTS VALIDATION ===\n");
    
    SAFE_LOG_IF(1, "Total flows with SLA data: " << tidCount << "\n");
    
    SAFE_LOG_IF(1, "Expected vs Actual Contract Results:\n");
    if (emergencyTids > 0) {
        double criticalHighDeviation = slaMonitor->GetCriticalHighSLADeviation();
        // Fix: Check if we have actual emergency flows with data
        uint32_t emergencyFlowsWithData = 0;
        for (const auto& [tid, metrics] : slaMonitor->GetFlowMetrics()) {
            if (metrics.assignedContract.contractName == "CriticalHigh" && metrics.delayMeasurements > 0) {
                emergencyFlowsWithData++;
            }
        }
        bool hasData = emergencyFlowsWithData > 0;
        SAFE_LOG_IF(1, "  CriticalHigh (Emergency): Expected=" << emergencyTids 
                  << ", Has Data=" << (hasData ? "Yes" : "No") << " (" << emergencyFlowsWithData << " flows)");
        if (hasData) SAFE_LOG_IF(1, " (Deviation: " << criticalHighDeviation << "%)");
        SAFE_LOG_IF(1, "\n");
    }
    
    if (criticalTids > 0) {
        double criticalBasicDeviation = slaMonitor->GetCriticalBasicSLADeviation();
        // Fix: Check if we have actual critical flows with data, not just deviation > 0
        uint32_t criticalFlowsWithData = 0;
        for (const auto& [tid, metrics] : slaMonitor->GetFlowMetrics()) {
            if (metrics.assignedContract.contractName == "CriticalBasic" && metrics.delayMeasurements > 0) {
                criticalFlowsWithData++;
            }
        }
        bool hasData = criticalFlowsWithData > 0;
        SAFE_LOG_IF(1, "  CriticalBasic (Critical): Expected=" << criticalTids 
                  << ", Has Data=" << (hasData ? "Yes" : "No") << " (" << criticalFlowsWithData << " flows)");
        if (hasData) SAFE_LOG_IF(1, " (Deviation: " << criticalBasicDeviation << "%)");
        SAFE_LOG_IF(1, "\n");
    }
    
    uint32_t normalTids = tidCount - emergencyTids - criticalTids;
    if (normalTids > 0) {
        double nonCriticalDeviation = slaMonitor->GetNonCriticalSLADeviation();
        // Fix: Check if we have actual non-critical flows with data
        uint32_t nonCriticalFlowsWithData = 0;
        for (const auto& [tid, metrics] : slaMonitor->GetFlowMetrics()) {
            if (metrics.assignedContract.contractName == "NonCritical" && metrics.delayMeasurements > 0) {
                nonCriticalFlowsWithData++;
            }
        }
        bool hasData = nonCriticalFlowsWithData > 0;
        SAFE_LOG_IF(1, "  NonCritical (Normal): Expected=" << normalTids 
                  << ", Has Data=" << (hasData ? "Yes" : "No") << " (" << nonCriticalFlowsWithData << " flows)");
        if (hasData) SAFE_LOG_IF(1, " (Deviation: " << nonCriticalDeviation << "%)");
        SAFE_LOG_IF(1, "\n");
    }
}


// ================== MAIN SIMULATION ==================
int main(int argc, char* argv[]) {
    // Default parameters
    std::string strategyName = "Reliability";
    std::string protocol = "UDP";
    uint32_t nAP = 1, nWifi = 2, payloadSize = 1000;
    double distance = 10, simtime = 10.0;
    uint32_t tidCount = 4;
    uint32_t criticalTids = 0;
    bool enableMobility = false;
    bool enableDuplicates = false;
    bool enableInterference = false;
    uint32_t seed = 1;
    // Use named constants instead of magic numbers
    uint32_t tcpSegmentSize = MLOConstants::DEFAULT_TCP_SEGMENT_SIZE;
    uint32_t mcs = MLOConstants::DEFAULT_MCS;
    uint32_t channelWidth = MLOConstants::DEFAULT_CHANNEL_WIDTH;
    uint32_t guardInterval = 800; // nanoseconds - EHT standard
    uint16_t mpduBufferSize = 256; // MPDU buffer size
    std::string dlAckSeqType = "NO-OFDMA";
    bool enableUlOfdma = false;
    bool enableBsrp = false;
    bool enablePcap = false;
    double interferenceDataRate = 5.0; // Mbps
    std::string scenarioName = "default";
    uint32_t runNumber = 1;
    
    // Additional parameters from test scripts
    std::string dataRate = "54Mbps";
    std::string csvFile = "";
    double interferenceIntensity = 0.0;
    uint32_t emergencyTids = 0;
    double linkFailureRate = 0.0;
    std::string mobilityPattern = "none";
    double mobilitySpeed = 0.0;
    std::string interferencePattern = "none";
    double maxInterference = 0.0;
    
    // Enhanced logging options
    bool enableTIDLogging = false;
    bool enableWindowLogging = false;
    uint32_t windowSize = 100;
    std::string interferenceFrequency = "medium";
    std::string congestionLevel = "low";
    double failureRate = 0.0;
    bool measureRecovery = false;
    uint32_t failureDuration = 1000;
    
    // Local variable for command line parsing
    uint32_t verbosityLevel = 1;
    g_verbosityLevel = 1;
    
    // Command line parsing
    CommandLine cmd(__FILE__);
    cmd.AddValue("strategy", "Strategy: RoundRobin, Greedy, Reliability, SLA-MLO", strategyName);
    cmd.AddValue("protocol", "Protocol: UDP, TCP, or Mixed", protocol);
    cmd.AddValue("nAP", "Number of APs", nAP);
    cmd.AddValue("nWifi", "Number of Wifi STAs", nWifi);
    cmd.AddValue("distance", "Distance between nodes", distance);
    cmd.AddValue("payloadSize", "Payload Size", payloadSize);
    cmd.AddValue("simtime", "Simulation Time", simtime);
    cmd.AddValue("tidCount", "Number of TID flows", tidCount);
    cmd.AddValue("emergencyTids", "Number of emergency TIDs", emergencyTids);
    cmd.AddValue("criticalTids", "Number of critical TIDs", criticalTids);
    cmd.AddValue("mobility", "Enable STA mobility", enableMobility);
    cmd.AddValue("duplicates", "Enable packet duplication", enableDuplicates);
    cmd.AddValue("interference", "Enable interference nodes", enableInterference);
    cmd.AddValue("seed", "Random seed", seed);
    cmd.AddValue("tcpSegmentSize", "TCP segment size", tcpSegmentSize);
    cmd.AddValue("mcs", "EHT MCS value (0-13)", mcs);
    cmd.AddValue("channelWidth", "Channel width in MHz", channelWidth);
    cmd.AddValue("guardInterval", "Guard interval in ns", guardInterval);
    cmd.AddValue("mpduBufferSize", "MPDU buffer size", mpduBufferSize);
    cmd.AddValue("dlAckType", "DL ack sequence type", dlAckSeqType);
    cmd.AddValue("enableUlOfdma", "Enable UL OFDMA", enableUlOfdma);
    cmd.AddValue("enableBsrp", "Enable BSRP", enableBsrp);
    cmd.AddValue("pcap", "Enable PCAP traces", enablePcap);
    cmd.AddValue("interferenceRate", "Interference data rate in Mbps", interferenceDataRate);
    cmd.AddValue("verbose", "Verbosity level (0=quiet, 1=summary, 2=detailed, 3=debug)", verbosityLevel);
    cmd.AddValue("scenario", "Scenario name for logging", scenarioName);
    cmd.AddValue("runNumber", "Run number for multiple iterations", runNumber);
    cmd.AddValue("dataRate", "Application data rate", dataRate);
    cmd.AddValue("tidLogging", "Enable per-TID detailed logging", enableTIDLogging);
    cmd.AddValue("windowLogging", "Enable window-based logging", enableWindowLogging);
    cmd.AddValue("windowSize", "Window size for window-based logging (packets)", windowSize);
    cmd.AddValue("csvFile", "CSV output file path", csvFile);
    cmd.AddValue("interferenceIntensity", "Interference intensity (0-1)", interferenceIntensity);
    cmd.AddValue("linkFailureRate", "Link failure rate (0-1)", linkFailureRate);
    cmd.AddValue("mobilityPattern", "Mobility pattern: none, gradual, random", mobilityPattern);
    cmd.AddValue("mobilitySpeed", "Mobility speed in m/s", mobilitySpeed);
    cmd.AddValue("interferencePattern", "Interference pattern: none, gradual, random, burst_2.4ghz, burst_5ghz, burst_all", interferencePattern);
    cmd.AddValue("maxInterference", "Maximum interference level (0-1)", maxInterference);
    cmd.AddValue("interferenceFrequency", "Interference frequency: low, medium, high", interferenceFrequency);
    cmd.AddValue("congestionLevel", "Congestion level: low, medium, high", congestionLevel);
    cmd.AddValue("failureRate", "Failure rate (0-1)", failureRate);
    cmd.AddValue("measureRecovery", "Enable recovery measurement", measureRecovery);
    cmd.AddValue("failureDuration", "Failure duration in ms", failureDuration);
    
    // Burst interference parameters
    uint32_t burstDuration = 1000;
    uint32_t burstInterval = 3000;
    bool enableFailureRecovery = false;
    bool trackRecoveryMetrics = false;
    
    cmd.AddValue("burstDuration", "Burst duration in ms", burstDuration);
    cmd.AddValue("burstInterval", "Burst interval in ms", burstInterval);
    cmd.AddValue("enableFailureRecovery", "Enable failure recovery", enableFailureRecovery);
    cmd.AddValue("trackRecoveryMetrics", "Track recovery metrics", trackRecoveryMetrics);    
    cmd.Parse(argc, argv);
    
    // Copy parsed verbosity level to atomic variable
    g_verbosityLevel = verbosityLevel;
    
    // Keep protocol string as provided (testing_script.py uses "Mixed")
    // std::transform(protocol.begin(), protocol.end(), protocol.begin(), ::toupper);

    // Set random seed
    RngSeedManager::SetSeed(seed);
    RngSeedManager::SetRun(runNumber);
    
    // Validate parameters
    if (emergencyTids + criticalTids > tidCount) {
        NS_FATAL_ERROR("Emergency TIDs (" << emergencyTids << ") + Critical TIDs (" << criticalTids << ") exceeds total TID count (" << tidCount << ")");
    }
    if (channelWidth != 20 && channelWidth != 40 && channelWidth != 80 && channelWidth != 160 && channelWidth != 320) {
        NS_FATAL_ERROR("Invalid channel width: " << channelWidth << ". Must be 20, 40, 80, 160, or 320 MHz");
    }
    if (mcs > 13) {
        NS_FATAL_ERROR("Invalid MCS value: " << mcs << ". Must be 0-13 for EHT");
    }
    if (simtime <= 0) {
        NS_FATAL_ERROR("Simulation time must be positive: " << simtime);
    }
    if (nWifi == 0) {
        NS_FATAL_ERROR("Number of WiFi nodes must be positive: " << nWifi);
    }
    
    // Process additional parameters from test scripts
    if (emergencyTids > 0) {
        SAFE_LOG_IF(1, "Emergency TIDs enabled: " << emergencyTids << "\n");
        // Emergency TIDs are separate from critical TIDs
    }
    
    // Ensure total TID count accommodates all priority levels
    tidCount = std::max(tidCount, emergencyTids + criticalTids);
    
    // Comprehensive parameter validation
    ValidationUtils::ValidateSimulationParameters(mcs, simtime, nWifi, tidCount, 
                                                  emergencyTids, criticalTids, distance, 
                                                  channelWidth, interferenceIntensity);
    
    // Apply interference intensity to existing interference parameters
    if (interferenceIntensity > 0.0) {
        enableInterference = true;
        interferenceDataRate = std::max(interferenceDataRate, interferenceIntensity * 50.0);
        SAFE_LOG_IF(2, "Interference intensity " << interferenceIntensity << " -> data rate " << interferenceDataRate << " Mbps\n");
    }
    
    // Handle mobility patterns - enableMobility should reflect actual mobility status
    if (enableMobility && mobilityPattern != "none") {
        SAFE_LOG_IF(2, "Mobility pattern: " << mobilityPattern << " at " << mobilitySpeed << " m/s\n");
    } else {
        // If mobility pattern is "none" or mobility is disabled, ensure enableMobility is false
        enableMobility = false;
    }
    
    // Handle various failure and degradation patterns
    if (linkFailureRate > 0.0) {
        SAFE_LOG_IF(2, "Link failure simulation enabled: rate=" << linkFailureRate << "\n");
    }
    
    
    if (measureRecovery) {
        SAFE_LOG_IF(2, "Recovery measurement enabled\n");
    }
    
    // Handle burst interference patterns
    if (interferencePattern.find("burst") != std::string::npos) {
        enableInterference = true;
        enableFailureRecovery = true;
        measureRecovery = true;
        trackRecoveryMetrics = true;
        
        // Adjust interference parameters based on burst type
        if (interferencePattern == "burst_2.4ghz") {
            interferenceDataRate = std::max(interferenceDataRate, 30.0);
            SAFE_LOG_IF(2, "Burst 2.4GHz interference: duration=" << burstDuration << "ms, interval=" << burstInterval << "ms\n");
        } else if (interferencePattern == "burst_5ghz") {
            interferenceDataRate = std::max(interferenceDataRate, 40.0);
            SAFE_LOG_IF(2, "Burst 5GHz interference: duration=" << burstDuration << "ms, interval=" << burstInterval << "ms\n");
        } else if (interferencePattern == "burst_all") {
            interferenceDataRate = std::max(interferenceDataRate, 50.0);
            SAFE_LOG_IF(2, "Burst multi-band interference: duration=" << burstDuration << "ms, interval=" << burstInterval << "ms\n");
        }
    }

    // ================== IMPLEMENT UNUSED PARAMETER LOGIC ==================
    
    
    // Set up emergency TIDs if specified (separate from critical TIDs)
    if (emergencyTids > 0) {
        // Emergency TIDs are handled separately and do not count as critical TIDs
        SAFE_LOG_IF(2, "Emergency TIDs enabled: " << emergencyTids << " (critical TIDs: " << criticalTids << ")\n");
    }
    
    
    // Log all advanced parameters if verbose
    if (g_verbosityLevel >= 2) {
        SAFE_LOG_IF(2, "Advanced parameter summary:\n");
        SAFE_LOG_IF(2, "  Data rate: " << dataRate << "\n");
        SAFE_LOG_IF(2, "  CSV file: " << (csvFile.empty() ? "default" : csvFile) << "\n");
        SAFE_LOG_IF(2, "  Interference pattern: " << interferencePattern << " (frequency: " << interferenceFrequency << ")\n");
    }

    // Global configurations - Applied early for all components
    SAFE_LOG_IF(2, "Applying global configurations...\n");
    Config::SetDefault("ns3::TcpSocket::SegmentSize", UintegerValue(tcpSegmentSize));
    Config::SetDefault("ns3::TcpSocket::DelAckCount", UintegerValue(1));

    // Configure DL acknowledgment
    if (dlAckSeqType == "ACK-SU-FORMAT") {
        Config::SetDefault("ns3::WifiDefaultAckManager::DlMuAckSequenceType",
                           EnumValue(WifiAcknowledgment::DL_MU_BAR_BA_SEQUENCE));
    } else if (dlAckSeqType == "MU-BAR") {
        Config::SetDefault("ns3::WifiDefaultAckManager::DlMuAckSequenceType",
                           EnumValue(WifiAcknowledgment::DL_MU_TF_MU_BAR));
    } else if (dlAckSeqType == "AGGR-MU-BAR") {
        Config::SetDefault("ns3::WifiDefaultAckManager::DlMuAckSequenceType",
                           EnumValue(WifiAcknowledgment::DL_MU_AGGREGATE_TF));
    }

    // ================== NODE CREATION ==================
    SAFE_LOG_IF(1, "\n╔═══════════════════════════════════════════════════╗\n");
    SAFE_LOG_IF(1, "║              MLO Simulation Setup                 ║\n");
    SAFE_LOG_IF(1, "╚═══════════════════════════════════════════════════╝\n");
    SAFE_LOG_IF(1, "Creating network topology...\n");
    
    NodeContainer wifiStaNodes;
    wifiStaNodes.Create(nWifi);
    NodeContainer wifiApNode;
    wifiApNode.Create(nAP);
    
    NodeContainer interferenceNodes;
    if (enableInterference) {
        interferenceNodes.Create(2);
        SAFE_LOG_IF(1, "  - Created " << interferenceNodes.GetN() << " interference nodes\n");
    }

    // ================== WIFI CONFIGURATION ==================
    SAFE_LOG_IF(1, "Configuring Wi-Fi 7 MLO...\n");
    
    WifiHelper wifi;
    wifi.SetStandard(WIFI_STANDARD_80211be);
    
    // Configure three links
    const uint8_t nLinks = 3;
    std::array<std::string, 3> channelStr;
    std::array<FrequencyRange, 3> freqRanges;
    std::string dataModeStr = "EhtMcs" + std::to_string(mcs);
    std::string ctrlRateStr;
    uint64_t nonHtRefRateMbps = EhtPhy::GetNonHtReferenceRate(mcs) / 1e6;

    // Link 0: 2.4GHz - Use validated channel width for compatibility
    uint32_t bw_2_4 = std::min(channelWidth, 40U); // Keep 2.4GHz compatible for now
    channelStr[0] = "{0, " + std::to_string(bw_2_4) + ", BAND_2_4GHZ, 0}";
    freqRanges[0] = WIFI_SPECTRUM_2_4_GHZ;
    ctrlRateStr = "ErpOfdmRate" + std::to_string(nonHtRefRateMbps) + "Mbps";
    wifi.SetRemoteStationManager<std::string, StringValue, std::string, StringValue>(uint8_t(0), "ns3::ConstantRateWifiManager",
                                "DataMode", StringValue(dataModeStr),
                                "ControlMode", StringValue(ctrlRateStr));

    // Link 1: 5GHz - Use compatible channel width
    uint32_t bw_5 = std::min(channelWidth, 160U); // 5GHz supports up to 160MHz reliably
    channelStr[1] = "{0, " + std::to_string(bw_5) + ", BAND_5GHZ, 0}";
    freqRanges[1] = WIFI_SPECTRUM_5_GHZ;
    ctrlRateStr = "OfdmRate" + std::to_string(nonHtRefRateMbps) + "Mbps";
    wifi.SetRemoteStationManager<std::string, StringValue, std::string, StringValue>(uint8_t(1), "ns3::ConstantRateWifiManager",
                                "DataMode", StringValue(dataModeStr),
                                "ControlMode", StringValue(ctrlRateStr));

    // Link 2: 6GHz - Full channel width supported
    channelStr[2] = "{0, " + std::to_string(channelWidth) + ", BAND_6GHZ, 0}";
    freqRanges[2] = WIFI_SPECTRUM_6_GHZ;
    wifi.SetRemoteStationManager<std::string, StringValue, std::string, StringValue>(uint8_t(2), "ns3::ConstantRateWifiManager",
                                "DataMode", StringValue(dataModeStr),
                                "ControlMode", StringValue(dataModeStr));

    // Disable EMLSR to avoid timing issues
    wifi.ConfigEhtOptions("EmlsrActivated", BooleanValue(false));

    Ssid ssid = Ssid("ns3-80211be-mlo");

    // ================== PHY CONFIGURATION ==================
    SAFE_LOG_IF(2, "Setting up PHY layer...\n");
    
    SpectrumWifiPhyHelper phy(nLinks);
    phy.SetPcapDataLinkType(WifiPhyHelper::DLT_IEEE802_11_RADIO);
    
    phy.Set("ChannelSwitchDelay", TimeValue(MicroSeconds(250)));

    std::vector<Ptr<MultiModelSpectrumChannel>> spectrumChannels;
    
    for (uint8_t linkId = 0; linkId < nLinks; linkId++) {
        phy.Set(linkId, "ChannelSettings", StringValue(channelStr[linkId]));
        
        auto spectrumChannel = CreateObject<MultiModelSpectrumChannel>(); //Channel Model that understands frequency and supports multiple channel 
        auto lossModel = CreateObject<FriisPropagationLossModel>();  //Path Loss Model
        spectrumChannel->AddPropagationLossModel(lossModel);
        
        auto delayModel = CreateObject<ConstantSpeedPropagationDelayModel>(); //Propagation Delay Model
        spectrumChannel->SetPropagationDelayModel(delayModel);
        
        phy.AddChannel(spectrumChannel, freqRanges[linkId]);
        spectrumChannels.push_back(spectrumChannel);
    }

    // ================== MAC CONFIGURATION ==================
    SAFE_LOG_IF(2, "Configuring MAC layer...\n");
    
    WifiMacHelper mac;

    mac.SetType("ns3::StaWifiMac", "Ssid", SsidValue(ssid));
    NetDeviceContainer staDevices = wifi.Install(phy, mac, wifiStaNodes);

    if (dlAckSeqType != "NO-OFDMA") {
        mac.SetMultiUserScheduler("ns3::RrMultiUserScheduler",
                                 "EnableUlOfdma", BooleanValue(enableUlOfdma),
                                 "EnableBsrp", BooleanValue(enableBsrp),
                                 "AccessReqInterval", TimeValue(MilliSeconds(20)));
    }
    
    mac.SetType("ns3::ApWifiMac",
               "EnableBeaconJitter", BooleanValue(false),
               "Ssid", SsidValue(ssid));
    
    NetDeviceContainer apDevices = wifi.Install(phy, mac, wifiApNode);

    // Install WiFi on interference nodes
    NetDeviceContainer interferenceDevices;
    if (enableInterference) {
        SAFE_LOG_IF(2, "Setting up interference nodes...\n");
        
        WifiHelper interferenceWifi;
        interferenceWifi.SetStandard(WIFI_STANDARD_80211be);
        
        for (uint8_t linkId = 0; linkId < nLinks; linkId++) {
            interferenceWifi.SetRemoteStationManager(static_cast<uint32_t>(linkId), "ns3::ConstantRateWifiManager",
                "DataMode", StringValue(dataModeStr),
                "ControlMode", StringValue(ctrlRateStr));
        }
        
        Ssid interferenceSSID = Ssid("interference-net");
        mac.SetType("ns3::AdhocWifiMac", "Ssid", SsidValue(interferenceSSID));
        
        SpectrumWifiPhyHelper interferencePhy(nLinks);
        for (uint8_t linkId = 0; linkId < nLinks; linkId++) {
            interferencePhy.Set(linkId, "ChannelSettings", StringValue(channelStr[linkId]));
            interferencePhy.AddChannel(spectrumChannels[linkId], freqRanges[linkId]);
        }
        
        interferenceDevices = interferenceWifi.Install(interferencePhy, mac, interferenceNodes);
    }

    // Assign streams
    int64_t streamNumber = 100;
    streamNumber += WifiHelper::AssignStreams(apDevices, streamNumber);
    streamNumber += WifiHelper::AssignStreams(staDevices, streamNumber);
    if (enableInterference) {
        streamNumber += WifiHelper::AssignStreams(interferenceDevices, streamNumber);
    }

    // Set parameters
    Config::Set("/NodeList/*/DeviceList/*/$ns3::WifiNetDevice/HeConfiguration/GuardInterval",
               TimeValue(NanoSeconds(guardInterval)));
    Config::Set("/NodeList/*/DeviceList/*/$ns3::WifiNetDevice/Mac/MpduBufferSize",
               UintegerValue(mpduBufferSize));

    // ================== MOBILITY ==================
    SAFE_LOG_IF(2, "Setting up mobility model...\n");
    
    MobilityHelper mobility;
    Ptr<ListPositionAllocator> positionAlloc = CreateObject<ListPositionAllocator>();

    positionAlloc->Add(Vector(0.0, 0.0, 0.0));
    
    for (uint32_t i = 0; i < nWifi; ++i) {
        double angle = 2 * M_PI * i / nWifi;
        positionAlloc->Add(Vector(distance * cos(angle), distance * sin(angle), 0.0));
    }
    
    mobility.SetPositionAllocator(positionAlloc);

    if (enableMobility) {
        std::string speedStr;
        if (mobilityPattern == "gradual") {
            // Gradual mobility with configurable speed
            double minSpeed = std::max(0.1, mobilitySpeed * 0.5);
            double maxSpeed = mobilitySpeed * 1.5;
            speedStr = "ns3::UniformRandomVariable[Min=" + std::to_string(minSpeed) + "|Max=" + std::to_string(maxSpeed) + "]";
        } else {
            // Default random mobility
            speedStr = "ns3::UniformRandomVariable[Min=0.5|Max=" + std::to_string(mobilitySpeed) + "]";
        }
        
        mobility.SetMobilityModel("ns3::RandomWalk2dMobilityModel",
                                "Bounds", RectangleValue(Rectangle(-30, 30, -30, 30)),
                                "Speed", StringValue(speedStr));
        mobility.Install(wifiStaNodes);
        
        mobility.SetMobilityModel("ns3::ConstantPositionMobilityModel");
        mobility.Install(wifiApNode);
    } else {
        mobility.SetMobilityModel("ns3::ConstantPositionMobilityModel");
        mobility.Install(wifiApNode);
        mobility.Install(wifiStaNodes);
    }

    if (enableInterference) {
        Ptr<ListPositionAllocator> interferencePos = CreateObject<ListPositionAllocator>();
        interferencePos->Add(Vector(distance * 1.2, 0.0, 0.0));
        interferencePos->Add(Vector(-distance * 1.2, 0.0, 0.0));
        
        MobilityHelper interferenceMobility;
        interferenceMobility.SetPositionAllocator(interferencePos);
        interferenceMobility.SetMobilityModel("ns3::ConstantPositionMobilityModel");
        interferenceMobility.Install(interferenceNodes);
    }

    // ================== INTERNET STACK ==================
    SAFE_LOG_IF(2, "Installing Internet stack...\n");
    
    InternetStackHelper stack;
    stack.Install(wifiApNode);
    stack.Install(wifiStaNodes);
    if (enableInterference) {
        stack.Install(interferenceNodes);
    }
    
    // Assign streams for all nodes in single pass
    streamNumber += stack.AssignStreams(wifiApNode, streamNumber);
    streamNumber += stack.AssignStreams(wifiStaNodes, streamNumber);
    if (enableInterference) {
        streamNumber += stack.AssignStreams(interferenceNodes, streamNumber);
    }

    Ipv4AddressHelper address;
    address.SetBase("192.168.1.0", "255.255.255.0");
    Ipv4InterfaceContainer staNodeInterfaces = address.Assign(staDevices);
    Ipv4InterfaceContainer apNodeInterface = address.Assign(apDevices);
    
    Ipv4InterfaceContainer interferenceInterfaces;
    if (enableInterference) {
        address.SetBase("192.168.2.0", "255.255.255.0");
        interferenceInterfaces = address.Assign(interferenceDevices);
    }

    // ================== STRATEGY & MONITORING SETUP ==================
    SAFE_LOG_IF(1, "Setting up MLO strategy and monitoring components...\n");
    
    // Create link quality monitor with aggressive thresholds for fast recovery detection
    std::shared_ptr<LinkQualityMonitor> linkMonitor = 
        std::make_shared<LinkQualityMonitor>(nLinks, 0.90);
    linkMonitor->SetGlobalTidParameters(emergencyTids, criticalTids);
    
    std::shared_ptr<LinkMappingStrategy> strategy;
    if (strategyName == "RoundRobin") {
        strategy = std::make_shared<RoundRobinStrategy>(nLinks);
    } else if (strategyName == "Greedy") {
        strategy = std::make_shared<GreedyLoadBalancer>(nLinks);
    } else if (strategyName == "Reliability") {
        auto reliabilityStrategy = std::make_shared<ReliabilityAwareStrategy>(nLinks);
        reliabilityStrategy->SetTidParameters(emergencyTids, criticalTids);
        strategy = reliabilityStrategy;
    } else if (strategyName == "SLA-MLO") {
        auto slaStrategy = std::make_shared<SLA_MLO_Strategy>(nLinks);
        
        // Configure SLAs based on criticality
        
        for (uint32_t tid = 0; tid < tidCount; ++tid) {
            if (tid < emergencyTids) {
                // Emergency TIDs: Ultra-critical (1ms, 1% error)
                slaStrategy->SetFlowSLA(tid, 1.0, 1.0, 10);  
                SAFE_LOG_IF(3, "  SLA-MLO TID " << std::setw(2) << tid << " → Emergency (1ms, 1%)\n");
            } else if (tid < (emergencyTids + criticalTids)) {  // ✅ FIXED!
                // Critical TIDs: Standard critical (50ms, 5% error)
                slaStrategy->SetFlowSLA(tid, 50.0, 5.0, 10);
                SAFE_LOG_IF(3, "  SLA-MLO TID " << std::setw(2) << tid << " → Critical (50ms, 5%)\n");
            } else {
                // Normal TIDs: Non-critical (100ms, 10% error)
                slaStrategy->SetFlowSLA(tid, 100.0, 10.0, 10); 
                SAFE_LOG_IF(3, "  SLA-MLO TID " << std::setw(2) << tid << " → Normal (100ms, 10%)\n");
            }
        }
        
        strategy = slaStrategy;
    } else  {
        NS_FATAL_ERROR("Unknown strategy: " << strategyName);
    }
    
    // Configure strategy with global TID parameters and link monitor
    strategy->SetGlobalTidParameters(emergencyTids, criticalTids);
    strategy->SetLinkQualityMonitor(linkMonitor);
    
    // SLA Deviation Monitoring Setup
    SAFE_LOG_IF(1, "Setting up Universal SLA Deviation Monitoring...\n");
    
    std::shared_ptr<UniversalSLADeviationMonitor> slaDeviationMonitor = 
        std::make_shared<UniversalSLADeviationMonitor>(tidCount);
    
    // Configure SLA contracts for all TIDs (single assignment)
    SAFE_LOG_IF(2, "Configuring SLA contracts for " << tidCount << " TIDs...\n");
    for (uint32_t tid = 0; tid < tidCount; ++tid) {
        if (tid < emergencyTids) {
            // Emergency TIDs (0 to emergencyTids-1): Ultra-critical requirements
            slaDeviationMonitor->SetFlowContract(tid, "CriticalHigh");   
            SAFE_LOG_IF(3, "  TID " << std::setw(2) << tid << " → CriticalHigh (Emergency)\n");
        } else if (tid < (emergencyTids + criticalTids)) {
            // Critical TIDs (emergencyTids to emergencyTids+criticalTids-1): Standard critical requirements
            slaDeviationMonitor->SetFlowContract(tid, "CriticalBasic");  
            SAFE_LOG_IF(3, "  TID " << std::setw(2) << tid << " → CriticalBasic (Critical)\n");
        } else {
            // Normal TIDs (remaining): Non-critical requirements
            slaDeviationMonitor->SetFlowContract(tid, "NonCritical");      
            SAFE_LOG_IF(3, "  TID " << std::setw(2) << tid << " → NonCritical (Normal)\n");
        }
    }
    
    // VALIDATE SLA SETUP
    SAFE_LOG_IF(1, "✅ SLA contracts configured for " << tidCount << " TIDs\n");
    
    // Validate SLA assignments
    ValidateSLAAssignments(slaDeviationMonitor, tidCount, emergencyTids, criticalTids);
    
    // Set simulation parameters in SLA monitor for packet-level logging
    slaDeviationMonitor->SetSimulationParameters(strategyName, protocol, nWifi, tidCount,
                                                 criticalTids, emergencyTids, simtime, payloadSize);
    
    // Attach universal SLA monitor to strategy
    strategy->SetSLADeviationMonitor(slaDeviationMonitor);

    // TCP MLO Connection Manager Setup
    std::shared_ptr<TcpMLOConnectionManager> tcpConnectionManager = nullptr;
    if (protocol == "TCP" || protocol == "Mixed") {
        tcpConnectionManager = std::make_shared<TcpMLOConnectionManager>(strategy, nWifi);
        SAFE_LOG_IF(2, "TCP MLO connection manager created for proper link assignment\n");
    }

    // ================== RESULT LOGGING SETUP ==================
    SAFE_LOG_IF(2, "Setting up result logger and failure injection...\n");
    
    // Create result logger with scenario name, custom CSV file, and number of links
    auto logger = std::make_shared<ResultLogger>(scenarioName, csvFile, nLinks);
    
    // Configure enhanced logging
    logger->SetLoggingMode(enableTIDLogging, enableWindowLogging, windowSize);
    
    // Initialize simulation parameters
    logger->InitializeSimulationParameters(strategyName, protocol, nWifi, simtime,
                                          tidCount, criticalTids, distance, enableMobility, runNumber,
                                          enableDuplicates, enableInterference, interferencePattern,
                                          interferenceIntensity, mobilityPattern, emergencyTids);
    
    // Connect the logger to monitoring components
    linkMonitor->SetResultLogger(logger);
    slaDeviationMonitor->SetResultLogger(logger);

    // ================== LINK FAILURE INJECTION ==================
    if (linkFailureRate > 0.0) {
        SAFE_LOG_IF(2, "Implementing link failure injection with rate: " << linkFailureRate << "\n");
        
        // Create failure injection events
        Ptr<UniformRandomVariable> failureRand = CreateObject<UniformRandomVariable>();
        
        for (uint8_t linkId = 0; linkId < nLinks; linkId++) {
            // Schedule periodic failure checks
            for (double time = 1.0; time < simtime; time += 1.0) {
                if (failureRand->GetValue(0, 1) < linkFailureRate) {
                    // Schedule failure event
                    Simulator::Schedule(Seconds(time), [linkMonitor, linkId]() {
                        // Inject failure by forcing poor metrics
                        linkMonitor->UpdateLinkMetrics(linkId, false, 100.0, 0, false);
                        SAFE_LOG_IF(3, "Injected failure on link " << (uint32_t)linkId << " at " 
                                  << Simulator::Now().GetSeconds() << "s\n");
                    });
                    
                    // Schedule recovery after failure duration
                    double recoveryTime = time + (failureDuration / 1000.0);
                    if (recoveryTime < simtime) {
                        Simulator::Schedule(Seconds(recoveryTime), [linkMonitor, linkId]() {
                            // Restore link with good metrics
                            linkMonitor->UpdateLinkMetrics(linkId, true, 1.0, 1000, false);
                            SAFE_LOG_IF(3, "Restored link " << (uint32_t)linkId << " at " 
                                      << Simulator::Now().GetSeconds() << "s\n");
                        });
                    }
                }
            }
        }
    }

    // ================== APPLICATION SETUP ==================
    SAFE_LOG_IF(1, "Setting up application containers...\n");
    
    ApplicationContainer serverApps;
    ApplicationContainer clientApps;

    // Enhanced server applications with proper packet sink
    if (protocol == "UDP" || protocol == "Mixed") {
        for (uint32_t i = 0; i < nWifi; ++i) {
            Ptr<EnhancedPacketSink> sink = CreateObject<EnhancedPacketSink>();
            sink->SetAttribute("Local", AddressValue(InetSocketAddress(Ipv4Address::GetAny(), 9)));
            sink->SetLinkMonitor(linkMonitor);  // Proper link monitor integration
            sink->SetSLAMonitor(slaDeviationMonitor);  // CRITICAL FIX: Add SLA monitoring
            sink->SetResultLogger(logger);     // Enable result logging
            sink->SetExpectedTid(255);  // UDP sink accepts all TIDs (255 = any)
            wifiStaNodes.Get(i)->AddApplication(sink);
            sink->SetStartTime(Seconds(0));
            sink->SetStopTime(Seconds(simtime + 1));
            serverApps.Add(sink);
        }
    }
    
    if (protocol == "TCP" || protocol == "Mixed") {
        // CREATE TCP SINKS - One per TID with unique ports
        for (uint32_t tid = 0; tid < tidCount; ++tid) {
            std::string tidProtocol = protocol;
            if (protocol == "Mixed") {
                tidProtocol = (tid % 2 == 0) ? "UDP" : "TCP";
            }
            
            if (tidProtocol == "TCP") {
                uint32_t staIdx = tid % nWifi;
                uint16_t tcpPort = 50000 + tid; // UNIQUE PORT PER TID
                
                Ptr<EnhancedPacketSink> tcpSink = CreateObject<EnhancedPacketSink>();
                tcpSink->SetAttribute("Local", AddressValue(InetSocketAddress(Ipv4Address::GetAny(), tcpPort)));
                tcpSink->SetAttribute("Protocol", TypeIdValue(TcpSocketFactory::GetTypeId()));
                tcpSink->SetLinkMonitor(linkMonitor);
                tcpSink->SetSLAMonitor(slaDeviationMonitor);
                tcpSink->SetResultLogger(logger);
                tcpSink->SetExpectedTid(tid);  // CRITICAL: Set expected TID
                wifiStaNodes.Get(staIdx)->AddApplication(tcpSink);
                tcpSink->SetStartTime(Seconds(0));
                tcpSink->SetStopTime(Seconds(simtime + 1));
                serverApps.Add(tcpSink);
                SAFE_LOG_IF(2, "TCP sink created for TID " << tid << " on STA " << staIdx << " port " << tcpPort << "\n");
            }
        }
    }

    serverApps.Start(Seconds(0));
    serverApps.Stop(Seconds(simtime + 1));

    // Enhanced client applications with proper tagging and strategy integration
    for (uint32_t tid = 0; tid < tidCount; ++tid) {
        uint32_t staIdx = tid % nWifi;
        
        // ENHANCED CRITICALITY ASSIGNMENT
        bool isCritical = false;
        
        if (tid < emergencyTids) {
            isCritical = true;
            SAFE_LOG_IF(3, "TID " << tid << " -> Emergency (CriticalHigh)\n");
        } else if (tid < (emergencyTids + criticalTids)) {  // ✅ FIXED!
            isCritical = true;
            SAFE_LOG_IF(3, "TID " << tid << " -> Critical (CriticalBasic)\n");
        } else {
            isCritical = false;
            SAFE_LOG_IF(3, "TID " << tid << " -> Normal (NonCritical)\n");
        }
        
        std::string tidProtocol = protocol;
        if (protocol == "Mixed") {
            tidProtocol = (tid % 2 == 0) ? "UDP" : "TCP";
            SAFE_LOG_IF(3, "  -> TID " << tid << " protocol assignment: " << protocol << " -> " << tidProtocol << " (tid%2=" << (tid%2) << ")\n");
        }
        
        SAFE_LOG_IF(3, "Creating application for TID " << tid << " (" << tidProtocol << ", " 
                  << (isCritical ? "critical" : "normal") << ") to STA " << staIdx << " [protocol=" << protocol << "]\n");
        
        if (tidProtocol == "UDP") {
            SAFE_LOG_IF(3, "  -> Creating UDP client for TID " << tid << "\n");
            Ptr<EnhancedUdpClient> client = CreateObject<EnhancedUdpClient>();
            
            if (!client) {
                SAFE_LOG_IF(1, "Error: Failed to create UDP client for TID " << tid << "\n");
                continue;
            }
            
            client->SetAttribute("RemoteAddress", 
                               AddressValue(InetSocketAddress(staNodeInterfaces.GetAddress(staIdx), 9)));
            client->SetAttribute("RemotePort", UintegerValue(9));
            client->SetAttribute("PacketSize", UintegerValue(payloadSize));
            
            // Calculate interval based on dataRate parameter
            Time interval;
            if (!dataRate.empty() && dataRate != "54Mbps") {
                // Parse dataRate (e.g., "100Mbps" -> 100)
                std::string rateStr = dataRate;
                rateStr.erase(rateStr.find("Mbps"), 4); 
                double rateMbps = std::stod(rateStr);
                double intervalSec = (payloadSize * 8.0) / (rateMbps * 1e6);
                interval = Seconds(intervalSec);
                
                SAFE_LOG_IF(3, "Using dataRate " << dataRate << " -> interval " << interval.GetMilliSeconds() << " ms\n");
            } else {
                // Default intervals for better failure detection
                interval = isCritical ? MilliSeconds(10) : MilliSeconds(25);
            }
            client->SetAttribute("Interval", TimeValue(interval));
            
            uint32_t maxPackets = static_cast<uint32_t>(simtime * 1000 / interval.GetMilliSeconds());
            maxPackets = std::min(maxPackets, 3000U); 
            client->SetAttribute("MaxPackets", UintegerValue(maxPackets));
            client->SetAttribute("Duplication", BooleanValue(enableDuplicates && isCritical));
            
            client->SetTid(tid);
            client->SetStrategy(strategy);
            client->SetLinkMonitor(linkMonitor);  // CRITICAL FIX: Enable transmission tracking
            
            wifiApNode.Get(0)->AddApplication(client);
            client->SetStartTime(Seconds(1.0));
            client->SetStopTime(Seconds(simtime));
            
            clientApps.Add(client);
        } else {
            // TCP TIDs will be handled by shared connections (created outside this loop)
            SAFE_LOG_IF(3, "  -> TID " << tid << " will use shared TCP connection to STA " << staIdx << " (tidProtocol=" << tidProtocol << ")\n");
        }
    }
    
    // CREATE TCP CONNECTIONS - One per TID like working version
    for (uint32_t tid = 0; tid < tidCount; ++tid) {
        uint32_t staIdx = tid % nWifi;
        
        // Use same criticality logic as UDP creation above
        bool isCritical = false;
        if (tid < emergencyTids) {
            isCritical = true;
        } else if (tid < (emergencyTids + criticalTids)) {
            isCritical = true;
        }
        
        std::string tidProtocol = protocol;
        if (protocol == "Mixed") {
            tidProtocol = (tid % 2 == 0) ? "UDP" : "TCP";
        }
        
        if (tidProtocol == "TCP") {
            Ptr<TcpMLOTrafficGenerator> tcpClient = CreateObject<TcpMLOTrafficGenerator>();
            
            if (!tcpClient) {
                SAFE_LOG_IF(1, "Error: Failed to create TCP client for TID " << tid << "\n");
                continue;
            }
            
            uint16_t tcpPort = 50000 + tid; // Match the TID-specific sink port
            tcpClient->SetAttribute("Remote", 
                AddressValue(InetSocketAddress(staNodeInterfaces.GetAddress(staIdx), tcpPort)));
            
            uint64_t maxBytes = static_cast<uint64_t>(simtime * 100000);
            maxBytes = std::min(maxBytes, static_cast<uint64_t>(10000000));
            tcpClient->SetAttribute("MaxBytes", UintegerValue(maxBytes));
            tcpClient->SetAttribute("SendSize", UintegerValue(tcpSegmentSize));
            
            tcpClient->SetTid(tid);
            tcpClient->SetIsCritical(isCritical);
            tcpClient->SetStrategy(strategy);
            
            // Let the strategy select the link dynamically like working version
            uint8_t initialLinkId = strategy->SelectLink(tid, isCritical);
            tcpClient->SetLinkId(initialLinkId);
            
            wifiApNode.Get(0)->AddApplication(tcpClient);
            tcpClient->SetStartTime(Seconds(1.0));
            tcpClient->SetStopTime(Seconds(simtime - 0.5));
            
            clientApps.Add(tcpClient);
            
            SAFE_LOG_IF(2, "TCP client created for TID " << tid << " → STA " << staIdx << " port " << tcpPort << " (link " << (uint32_t)initialLinkId << ")\n");
        }
    }
    
    SAFE_LOG_IF(2, "Successfully created " << clientApps.GetN() << " client applications\n");
    

    // ================== ENHANCED INTERFERENCE APPLICATIONS ==================
    if (enableInterference) {
        SAFE_LOG_IF(2, "Setting up interference generators with pattern: " << interferencePattern << "\n");
        
        for (uint32_t i = 0; i < interferenceNodes.GetN(); i++) {
            Ptr<InterferenceGenerator> interferer = CreateObject<InterferenceGenerator>();
            
            // Configure interference parameters based on pattern
            double aggressiveRate = std::max(interferenceDataRate, 25.0); // At least 25Mbps
            interferer->SetAttribute("DataRate", 
                DataRateValue(DataRate(std::to_string(aggressiveRate) + "Mbps")));
            interferer->SetAttribute("PacketSize", UintegerValue(1472)); // Larger packets
            interferer->SetAttribute("RemoteAddress", 
                AddressValue(InetSocketAddress(Ipv4Address("255.255.255.255"), 9999)));
            
            // Configure timing based on interference pattern
            if (interferencePattern.find("burst") != std::string::npos) {
                // Burst interference: short bursts with longer quiet periods
                double burstSec = burstDuration / 1000.0;
                double intervalSec = burstInterval / 1000.0;
                
                interferer->SetAttribute("OnTime", TimeValue(Seconds(burstSec)));
                interferer->SetAttribute("OffTime", TimeValue(Seconds(intervalSec - burstSec)));
                
                SAFE_LOG_IF(2, "Burst interference node " << i << ": " << aggressiveRate << " Mbps, "
                          << burstSec << "s ON, " << (intervalSec - burstSec) << "s OFF\n");
            } else if (interferencePattern == "gradual") {
                // Gradual interference: slowly increasing intensity
                interferer->SetAttribute("OnTime", TimeValue(Seconds(2.0))); 
                interferer->SetAttribute("OffTime", TimeValue(Seconds(1.0)));
                SAFE_LOG_IF(2, "Gradual interference node " << i << ": " << aggressiveRate << " Mbps\n");
            } else if (interferencePattern == "random") {
                // Random interference: random on/off periods
                interferer->SetAttribute("OnTime", TimeValue(Seconds(1.0 + i * 0.5))); 
                interferer->SetAttribute("OffTime", TimeValue(Seconds(0.5 + i * 0.3)));
                SAFE_LOG_IF(2, "Random interference node " << i << ": " << aggressiveRate << " Mbps\n");
            } else {
                // Default continuous interference
                interferer->SetAttribute("OnTime", TimeValue(Seconds(1.5))); 
                interferer->SetAttribute("OffTime", TimeValue(Seconds(1.5)));
                SAFE_LOG_IF(2, "Continuous interference node " << i << ": " << aggressiveRate << " Mbps\n");
            }
            
            interferenceNodes.Get(i)->AddApplication(interferer);
            interferer->SetStartTime(Seconds(2.0));
            interferer->SetStopTime(Seconds(simtime));
        }
        
        // Additional burst interference simulation through link quality degradation
        if (interferencePattern.find("burst") != std::string::npos) {
            SAFE_LOG_IF(2, "Implementing burst link quality degradation simulation\n");
            
            // Determine which links to affect based on burst pattern
            std::vector<uint8_t> affectedLinks;
            if (interferencePattern == "burst_2.4ghz") {
                affectedLinks.push_back(0); // Only 2.4GHz link (link 0)
                SAFE_LOG_IF(2, "Burst interference targeting 2.4GHz link only\n");
            } else if (interferencePattern == "burst_5ghz") {
                affectedLinks.push_back(1); // Only 5GHz link (link 1)
                SAFE_LOG_IF(2, "Burst interference targeting 5GHz link only\n");
            } else if (interferencePattern == "burst_all") {
                for (uint8_t i = 0; i < nLinks; i++) {
                    affectedLinks.push_back(i); // All links
                }
                SAFE_LOG_IF(2, "Burst interference targeting all links\n");
            }
            
            // Schedule burst interference events that affect specific link quality
            Ptr<UniformRandomVariable> burstRand = CreateObject<UniformRandomVariable>();
            
            for (double time = 3.0; time < simtime; time += (burstInterval / 1000.0)) {
                // Schedule burst start - capture affectedLinks by value
                Simulator::Schedule(Seconds(time), [linkMonitor, interferenceIntensity, burstRand, affectedLinks]() {
                    for (uint8_t linkId : affectedLinks) {
                        // Apply burst interference effect only to targeted links
                        bool success = burstRand->GetValue(0, 1) > interferenceIntensity;
                        double burstDelay = success ? 1.0 : 10.0 + (interferenceIntensity * 20.0);
                        
                        linkMonitor->UpdateLinkMetrics(linkId, success, burstDelay, success ? 1000 : 0, 0, false, false);
                        
                        SAFE_LOG_IF(3, "Burst interference affecting link " << (uint32_t)linkId 
                                  << " at " << Simulator::Now().GetSeconds() << "s\n");
                    }
                });
                
                // Schedule burst end (recovery) - only recover affected links
                double recoveryTime = time + (burstDuration / 1000.0);
                if (recoveryTime < simtime) {
                    Simulator::Schedule(Seconds(recoveryTime), [linkMonitor, affectedLinks]() {
                        for (uint8_t linkId : affectedLinks) {
                            // Restore normal link quality only for affected links
                            linkMonitor->UpdateLinkMetrics(linkId, true, 1.0, 1000, false);
                            
                            SAFE_LOG_IF(3, "Link " << (uint32_t)linkId << " recovered from burst at " 
                                      << Simulator::Now().GetSeconds() << "s\n");
                        }
                    });
                }
            }
        }
    }

    // ================== TRACING AND MONITORING ==================
    if (enablePcap) {
        phy.EnablePcap("mlo-ap", apDevices.Get(0), 0);
        phy.EnablePcap("mlo-sta", staDevices.Get(0), 0);
    }

    // ================== FLOW MONITOR ==================
    FlowMonitorHelper flowHelper;
    Ptr<FlowMonitor> flowMonitor = flowHelper.InstallAll();

    // ================== RUN SIMULATION ==================
    strategy->PrintConfiguration();
    
    std::cout << "\n╔═══════════════════════════════════════════════════╗\n";
    std::cout << "║             MLO Simulation Starting               ║\n";
    std::cout << "╚═══════════════════════════════════════════════════╝\n";
    SAFE_LOG_IF(1, "Scenario: " << scenarioName << " | Run: " << runNumber 
              << " | Strategy: " << strategyName << " | Protocol: " << protocol << "\n");
    SAFE_LOG_IF(1, "TIDs: " << tidCount << " (Critical: " << criticalTids << ") | Time: " << simtime << "s\n");
    SAFE_LOG_IF(1, "Nodes: " << nWifi << " STAs, " << nAP << " AP | Interference: " << (enableInterference ? "Yes" : "No") << "\n");
    
    // Enhanced monitoring with more frequent updates for recovery detection
    if (g_verbosityLevel >= 2 && simtime > 5) {
        Time statsInterval = Seconds(simtime / 8.0); // More frequent monitoring
        for (Time t = statsInterval; t < Seconds(simtime); t += statsInterval) {
            Simulator::Schedule(t, [linkMonitor, t]() {
                auto metrics = linkMonitor->GetAllMetrics();
                SAFE_LOG_IF(2, "[" << t.GetSeconds() << "s] Links PDR: ");
                for (size_t i = 0; i < metrics.size(); i++) {
                    SAFE_LOG_IF(2, i << "=" << (metrics[i].pdr * 100) << "% ");
                }
                
                // Show failure states
                bool anyFailures = false;
                for (size_t i = 0; i < metrics.size(); i++) {
                    if (metrics[i].isInFailureState) {
                        anyFailures = true;
                        break;
                    }
                }
                if (anyFailures) {
                    SAFE_LOG_IF(2, " [FAILURES DETECTED]");
                }
                SAFE_LOG_IF(2, "\n");
            });
        }
    }
    
    // Progress bar functionality removed to simplify output
    
    Simulator::Stop(Seconds(simtime + 1));
    Simulator::Run();
    
    if (g_verbosityLevel >= 1) {
        std::cout << "\n";
    }

    // Results Collection
    SAFE_LOG_IF(1, "\n=== ENHANCED SIMULATION RESULTS WITH RECOVERY ANALYSIS ===\n");
    
    // Print debug info to validate data collection
    linkMonitor->PrintDebugInfo();
    
    Ptr<Ipv4FlowClassifier> classifier = 
        DynamicCast<Ipv4FlowClassifier>(flowHelper.GetClassifier());
    FlowMonitor::FlowStatsContainer stats = flowMonitor->GetFlowStats();
    
    std::vector<double> latencyValues;
    double totalThroughput = 0;
    uint64_t totalTxPackets = 0;
    uint64_t totalRxPackets = 0;
    uint64_t totalRxBytes = 0;
    double totalDelay = 0;
    double totalJitter = 0;
    uint32_t flowsWithJitter = 0;
    
    SAFE_LOG_IF(2, "\n=== Per-Flow Statistics ===\n");
    for (const auto& flow : stats) {
        const FlowMonitor::FlowStats& flowStats = flow.second;
        
        double flowDuration = (flowStats.timeLastRxPacket - flowStats.timeFirstTxPacket).GetSeconds();
        double flowThroughput = 0.0;
        
        // Improved throughput calculation with better error handling
        if (flowStats.rxBytes > 0 && flowStats.txPackets > 0) {
            if (flowDuration > 0) {
                // Standard calculation using actual flow duration
                flowThroughput = (flowStats.rxBytes * 8.0) / (flowDuration * 1e6);
            } else if (flowStats.timeLastRxPacket.GetSeconds() > 0 && flowStats.timeFirstTxPacket.GetSeconds() > 0) {
                // Use individual timestamps if duration calculation failed
                double lastRxTime = flowStats.timeLastRxPacket.GetSeconds();
                double firstTxTime = flowStats.timeFirstTxPacket.GetSeconds();
                if (lastRxTime > firstTxTime) {
                    flowThroughput = (flowStats.rxBytes * 8.0) / ((lastRxTime - firstTxTime) * 1e6);
                }
            }
            
            // If still zero but we have data, use simulation time as fallback
            if (flowThroughput == 0.0 && flowStats.rxBytes > 0) {
                double simTime = Simulator::Now().GetSeconds();
                if (simTime > 0) {
                    flowThroughput = (flowStats.rxBytes * 8.0) / (simTime * 1e6);
                    SAFE_LOG_IF(2, "Warning: Using simulation time for throughput calculation for flow " 
                              << flow.first << " (rxBytes=" << flowStats.rxBytes << ", simTime=" << simTime << "s)\n");
                }
            }
        }
        
        totalThroughput += flowThroughput;
        totalTxPackets += flowStats.txPackets;
        totalRxPackets += flowStats.rxPackets;
        totalRxBytes += flowStats.rxBytes;
        
        if (flowStats.rxPackets > 0) {
            double avgDelay = flowStats.delaySum.GetMilliSeconds() / flowStats.rxPackets;
            totalDelay += flowStats.delaySum.GetMilliSeconds();
            
            if (latencyValues.size() < 10000) {
                for (uint32_t i = 0; i < std::min(flowStats.rxPackets, 100U); ++i) {
                    latencyValues.push_back(avgDelay);
                }
            }
            
            if (flowStats.rxPackets > 1) {
                double avgJitter = flowStats.jitterSum.GetMilliSeconds() / (flowStats.rxPackets - 1);
                totalJitter += flowStats.jitterSum.GetMilliSeconds();
                flowsWithJitter++;
                (void)avgJitter; // Suppress unused variable warning
            }
        }
        
        if (g_verbosityLevel >= 2) {
            Ipv4FlowClassifier::FiveTuple fiveTuple = classifier->FindFlow(flow.first);
            SAFE_LOG_IF(2, "Flow " << flow.first << " (" << fiveTuple.sourceAddress 
                      << " -> " << fiveTuple.destinationAddress << "): "
                      << flowThroughput << " Mbps, PDR: " 
                      << (flowStats.rxPackets * 100.0 / flowStats.txPackets) << "%"
                      << " [Duration: " << flowDuration << "s, TxPkts: " << flowStats.txPackets 
                      << ", RxPkts: " << flowStats.rxPackets << ", RxBytes: " << flowStats.rxBytes << "]\n");
        }
    }
    
    // Calculate aggregate metrics using physical layer statistics
    uint64_t totalPhysicalTx = 0;
    uint64_t totalPhysicalRx = 0;
    
    // Aggregate physical layer statistics from all links
    for (uint8_t linkId = 0; linkId < nLinks; linkId++) {
        LinkQualityMonitor::LinkMetrics metrics = linkMonitor->GetLinkMetrics(linkId);
        totalPhysicalTx += metrics.packetsTransmitted;
        totalPhysicalRx += metrics.packetsReceived;
    }
    
    // Calculate overall PDR based on physical layer statistics
    double overallPdr = (totalPhysicalTx > 0) ? 
        (totalPhysicalRx * 100.0 / totalPhysicalTx) : 0;
    double avgDelay = (totalRxPackets > 0) ? 
        (totalDelay / totalRxPackets) : 0;
    double avgJitter = (flowsWithJitter > 0) ? 
        (totalJitter / flowsWithJitter) : 0;
    double tailLatency99 = CalculatePercentile(latencyValues, 99);
    double tailLatency99_9 = CalculatePercentile(latencyValues, 99.9);
    
    // Get enhanced metrics from strategy and monitor
    std::vector<double> linkUsage = strategy->GetLinkUsage();
    std::vector<double> linkThroughput = strategy->GetLinkThroughput();
    
    // Get recovery and critical metrics with proper validation
    double avgRecoveryTime = linkMonitor->GetAverageRecoveryTime().GetMilliSeconds();
    double criticalPdr = linkMonitor->GetCriticalPDR();
    double criticalAvgDelay = linkMonitor->GetCriticalAvgDelay();
    double nonCriticalPdr = linkMonitor->GetNonCriticalPDR();
    double nonCriticalAvgDelay = linkMonitor->GetNonCriticalAvgDelay();
    
    // CRITICAL FIX: Handle sentinel values (-1.0) for cases with no data
    if (criticalPdr < 0) {
        // If no critical packets were transmitted, Critical PDR should be 0% (not 100%)
        // This correctly represents "no successful critical packet delivery"
        criticalPdr = 0.0;
        SAFE_LOG_IF(3, "No critical packets transmitted - setting Critical PDR to 0%\n");
    }
    if (nonCriticalPdr < 0) {
        // If no non-critical packets were transmitted, set to 0%
        nonCriticalPdr = 0.0;
        SAFE_LOG_IF(3, "No non-critical packets transmitted - setting Non-Critical PDR to 0%\n");
    }
    if (criticalAvgDelay < 0) criticalAvgDelay = 0.0;
    if (nonCriticalAvgDelay < 0) nonCriticalAvgDelay = 0.0;
    
    // ================== CALCULATE 3-TIER SLA DEVIATION METRICS ==================
    double overallSLADeviation = strategy->GetOverallSLADeviation();
    double nonCriticalSLADeviation = strategy->GetNonCriticalSLADeviation();
    double criticalHighSLADeviation = strategy->GetCriticalHighSLADeviation();
    double criticalBasicSLADeviation = strategy->GetCriticalBasicSLADeviation();
    
    // SLA Performance Classification
    std::string slaPerformance;
    if (overallSLADeviation < 1.0) {
        slaPerformance = "EXCELLENT";
    } else if (overallSLADeviation < 5.0) {
        slaPerformance = "GOOD";
    } else if (overallSLADeviation < 10.0) {
        slaPerformance = "FAIR";
    } else {
        slaPerformance = "POOR";
    }
    
    
    // Level 0: Essential results only
    std::cout << "\n=== SIMULATION RESULTS ===\n"
              << "Strategy: " << strategyName << " | Protocol: " << protocol << "\n"
              << "Total Throughput: " << totalThroughput << " Mbps\n"
              << "Overall PDR: " << overallPdr << "%\n"
              << "Average Delay: " << avgDelay << " ms\n"
              << "Critical PDR: " << criticalPdr << "% | Non-Critical PDR: " << nonCriticalPdr << "%\n"
              << "SLA Performance: " << slaPerformance << " (Deviation: " << overallSLADeviation << "%)\n";

    // Level 1: More specific information  
    SAFE_LOG_IF(1, "\n=== DETAILED METRICS ===\n"
              << "Scenario: " << scenarioName << ", Run: " << runNumber << "\n"
              << "99th Percentile Latency: " << tailLatency99 << " ms\n"
              << "99.9th Percentile Latency: " << tailLatency99_9 << " ms\n"
              << "Average Jitter: " << avgJitter << " ms\n"
              << "Average Recovery Time: " << avgRecoveryTime << " ms\n"
              << "Critical Traffic Avg Delay: " << criticalAvgDelay << " ms\n"
              << "Non-Critical Traffic Avg Delay: " << nonCriticalAvgDelay << " ms\n"
              << "\n=== LINK USAGE ===\n"
              << "2.4GHz: " << std::fixed << std::setprecision(1) << linkUsage[0] 
              << "% | 5GHz: " << linkUsage[1] << "% | 6GHz: " << linkUsage[2] << "%\n"
              << "\n=== LINK THROUGHPUT ===\n"
              << "2.4GHz: " << linkThroughput[0] << " Mbps | 5GHz: " << linkThroughput[1] 
              << " Mbps | 6GHz: " << linkThroughput[2] << " Mbps\n");
    // Level 1: Duplication statistics if enabled
    if (enableDuplicates) {
        auto linkMetrics = linkMonitor->GetAllMetrics();
        uint64_t totalDuplicatesTx = 0, totalDuplicatesRx = 0;
        for (uint8_t i = 0; i < 3; i++) {
            totalDuplicatesTx += linkMetrics[i].duplicatesTransmitted;
            totalDuplicatesRx += linkMetrics[i].duplicatesReceived;
        }
        
        SAFE_LOG_IF(1, "\n=== DUPLICATION STATISTICS ===\n"
                  << "Total Duplicates: " << totalDuplicatesTx << " TX, " << totalDuplicatesRx << " RX\n"
                  << "2.4GHz: " << linkMetrics[0].duplicatesTransmitted << "/" << linkMetrics[0].duplicatesReceived 
                  << " | 5GHz: " << linkMetrics[1].duplicatesTransmitted << "/" << linkMetrics[1].duplicatesReceived
                  << " | 6GHz: " << linkMetrics[2].duplicatesTransmitted << "/" << linkMetrics[2].duplicatesReceived << "\n");
    }
    
    
    // ================== SLA RESULTS VALIDATION ==================
    ValidateSLAResults(slaDeviationMonitor, tidCount, emergencyTids, criticalTids);

    slaDeviationMonitor->PrintDetailedSLADebug();
    
    // Level 1: SLA analysis details
    SAFE_LOG_IF(1, "\n=== SLA ANALYSIS ===\n");
    if (overallSLADeviation < 1.0) {
        SAFE_LOG_IF(1, "✅ EXCELLENT: " << strategyName << " meets SLA requirements\n");
    } else if (overallSLADeviation < 5.0) {
        SAFE_LOG_IF(1, "✓ GOOD: " << strategyName << " mostly meets SLA\n");
    } else if (overallSLADeviation < 10.0) {
        SAFE_LOG_IF(1, "⚠️ FAIR: " << strategyName << " partially violates SLA\n");
    } else {
        SAFE_LOG_IF(1, "❌ POOR: " << strategyName << " significantly violates SLA\n");
    }
    
    // Calculate additional derived metrics
    double loadBalanceVariance = 0.0;
    double avgLinkUsage = (linkUsage[0] + linkUsage[1] + linkUsage[2]) / 3.0;
    for (int i = 0; i < 3; i++) {
        loadBalanceVariance += std::pow(linkUsage[i] - avgLinkUsage, 2);
    }
    loadBalanceVariance = std::sqrt(loadBalanceVariance / 3.0);
    
    // Log enhanced results to unified CSV with comprehensive parameters
    logger->Log(strategyName, protocol, nAP, nWifi, payloadSize, simtime, 
               totalThroughput, overallPdr, avgDelay, tailLatency99_9, avgJitter,
               linkUsage, linkThroughput, enableDuplicates, enableInterference, 
               avgRecoveryTime, criticalPdr, criticalAvgDelay, nonCriticalPdr, 
               nonCriticalAvgDelay, tidCount, criticalTids, distance, enableMobility, runNumber,
               // Enhanced parameters
               interferencePattern, interferenceIntensity, mobilityPattern, emergencyTids,
               // 3-Tier SLA Deviation metrics
               overallSLADeviation, nonCriticalSLADeviation,
               criticalHighSLADeviation, criticalBasicSLADeviation);
    
    SAFE_LOG_IF(1, "\n=== SIMULATION COMPLETED SUCCESSFULLY ===\n");
    SAFE_LOG_IF(1, "Enhanced results saved to: scratch/output_files_csv/mlo_unified_results.csv\n");
    
    // Performance summary with recovery analysis
    if (g_verbosityLevel >= 2) {
        SAFE_LOG_IF(2, "\n=== Performance Summary with Recovery Analysis ===\n");
        SAFE_LOG_IF(2, "Best performing link: ");
        auto bestThroughputIdx = std::max_element(linkThroughput.begin(), linkThroughput.end()) - linkThroughput.begin();
        SAFE_LOG_IF(2, (bestThroughputIdx == 0 ? "2.4GHz" : (bestThroughputIdx == 1 ? "5GHz" : "6GHz")) 
                  << " (" << linkThroughput[bestThroughputIdx] << " Mbps)\n");
        
        SAFE_LOG_IF(2, "Most used link: ");
        auto mostUsedIdx = std::max_element(linkUsage.begin(), linkUsage.end()) - linkUsage.begin();
        SAFE_LOG_IF(2, (mostUsedIdx == 0 ? "2.4GHz" : (mostUsedIdx == 1 ? "5GHz" : "6GHz")) 
                  << " (" << linkUsage[mostUsedIdx] << "%)\n");
        
        if (avgRecoveryTime > 0) {
            SAFE_LOG_IF(2, "🔄 Link recovery observed: " << avgRecoveryTime << " ms average\n");
            SAFE_LOG_IF(2, "✅ Recovery mechanism is functioning correctly\n");
        } else {
            if (enableInterference) {
                SAFE_LOG_IF(2, "⚠️  No link recovery events detected despite interference\n");
                SAFE_LOG_IF(2, "   Consider: more aggressive interference, longer simulation, or lower PDR threshold\n");
            } else {
                SAFE_LOG_IF(2, "ℹ️  No recovery events (interference disabled)\n");
            }
        }
        
        if (criticalTids > 0) {
            SAFE_LOG_IF(2, "Critical traffic performance: " << criticalPdr << "% PDR, " 
                      << criticalAvgDelay << " ms delay\n");
        }
        
        // Recovery effectiveness analysis
        auto finalMetrics = linkMonitor->GetAllMetrics();
        uint32_t totalFailures = 0;
        uint32_t totalRecoveries = 0;
        for (const auto& metric : finalMetrics) {
            totalFailures += metric.failureCount;
            totalRecoveries += metric.recoveryCount;
        }
        
        if (totalFailures > 0) {
            double recoveryRate = (double)totalRecoveries / totalFailures * 100;
            SAFE_LOG_IF(2, "Recovery effectiveness: " << recoveryRate << "% (" 
                      << totalRecoveries << "/" << totalFailures << " failures recovered)\n");
        }
    }
    Simulator::Destroy();
    
    return 0;
}