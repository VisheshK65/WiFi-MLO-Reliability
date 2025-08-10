
# Multi-Link Operation Resource Allocation for Wi-Fi 8 Research

🚀 **A comprehensive ns-3 simulation framework for evaluating Multi-Link Operation (MLO) strategies in next-generation Wi-Fi networks**

## 📚 Table of Contents

1. [🎯 What This Project Does](#-what-this-project-does)
2. [⚡ Quick Start (5 Minutes)](#-quick-start-5-minutes)
3. [🛠️ Available MLO Strategies](#️-available-mlo-strategies)
4. [🔧 Simulation Setup and Usage](#-simulation-setup-and-usage)
   - [Environment Setup](#environment-setup)
   - [Running Simulations](#running-simulations)
   - [Testing Framework](#testing-framework)
   - [Analysis Tools](#analysis-tools)
5. [📊 Understanding the Results](#-understanding-the-results)
6. [🧪 Advanced Usage](#-advanced-usage)
7. [🔬 Research Background](#-research-background)
8. [📈 Performance Metrics](#-performance-metrics)
9. [🚀 Troubleshooting](#-troubleshooting)
10. [📝 Research Context](#-research-context)

## 🎯 What This Project Does

This simulation framework helps researchers evaluate and compare different **resource allocation strategies** for Multi-Link Operation in Wi-Fi 8 (IEEE 802.11bn). The project focuses on:

- **📡 Multi-Link WiFi Simulation**: Simulate simultaneous operation across 2.4GHz, 5GHz, and 6GHz bands
- **🎛️ Strategy Comparison**: Compare 4 different MLO strategies (RoundRobin, Greedy, Reliability, SLA-MLO)
- **📊 Performance Analysis**: Comprehensive analysis of reliability, throughput, and latency metrics
- **🔬 Research-Ready**: Built for Wi-Fi 8 Ultra High Reliability (UHR) research

## ⚡ Quick Start (5 Minutes)

**Prerequisites**: Ubuntu 18.04+, Python 3.8+, 8GB RAM minimum

```bash
# 1. Navigate to the ns-3.44 directory (this should be your current location)
cd ns-allinone-3.44/ns-3.44/

# 2. Set up the environment (one-time setup)
chmod +x setup_environment.sh
./setup_environment.sh

# 3. Activate the environment
source mlo_venv/bin/activate

# 4. Build ns-3 (if not already built)
./ns3 configure --enable-examples --enable-tests
./ns3 build

# 5. Run your first simulation!
./ns3 run "scratch/mlo_simulator --strategy=RoundRobin --nWifi=4 --tidCount=8 --simtime=30"

# 6. Analyze results
python3 mlo_analyser_script.py
```

**🎉 Done!** Check `scratch/visualizations/` for your analysis results.

## 🛠️ Available MLO Strategies

The simulator implements four distinct resource allocation strategies:

### 1. **RoundRobin** (Baseline)
- **What it does**: Distributes traffic flows across links in a simple round-robin fashion
- **Best for**: Understanding baseline performance, simple load distribution
- **Use case**: Benchmarking other strategies

### 2. **Greedy** (Load Balancing)
- **What it does**: Assigns new traffic flows to the link with the least current load
- **Best for**: Maximizing total throughput, avoiding link overload
- **Use case**: High-throughput scenarios

### 3. **Reliability** (Reliability-Focused)
- **What it does**: Prioritizes link quality and reliability over raw throughput
- **Best for**: Critical applications requiring consistent performance
- **Use case**: Emergency services, industrial control systems

### 4. **SLA-MLO** (SLA-Optimized)
- **What it does**: Optimizes resource allocation based on Service Level Agreement requirements
- **Best for**: Mixed traffic with different QoS requirements
- **Use case**: Enterprise networks, mixed critical/non-critical traffic

## 🔧 Simulation Setup and Usage

### Environment Setup

The project includes an automated setup script that handles all dependencies:

**Step 1: Run Setup Script**
```bash
chmod +x setup_environment.sh
./setup_environment.sh
```

The setup script will:
- ✅ Check Python version (3.8+ required)
- ✅ Create virtual environment
- ✅ Install all Python dependencies from requirements.txt
- ✅ Create output directories
- ✅ Display comprehensive usage guide

**Step 2: Activate Environment**
```bash
source mlo_venv/bin/activate
```

**Step 3: Build ns-3 (if needed)**
```bash
./ns3 configure --enable-examples --enable-tests
./ns3 build
```

### Running Simulations

**Basic Simulation Examples:**

```bash
# Basic simulation with default parameters
./ns3 run "scratch/mlo_simulator --strategy=RoundRobin --nWifi=4 --tidCount=8 --simtime=30"

# Reliability-focused simulation
./ns3 run "scratch/mlo_simulator --strategy=Reliability --nWifi=8 --tidCount=16 --simtime=60"

# Large network test
./ns3 run "scratch/mlo_simulator --strategy=SLA-MLO --nWifi=20 --tidCount=24 --simtime=45"
```

**Key Parameters:**
- `--strategy`: Choose strategy (RoundRobin|Greedy|Reliability|SLA-MLO)
- `--nWifi`: Number of WiFi stations (2-80)
- `--tidCount`: Number of traffic flows (4-96)
- `--criticalTids`: Number of critical traffic flows
- `--simtime`: Simulation duration in seconds
- `--mobility`: Enable node mobility (true|false)
- `--interference`: Add interference sources (true|false)

### Testing Framework

**Basic Testing (Recommended for Most Systems):**
```bash
python3 testing_script.py
```

**Extreme Testing (High-Performance Systems):**
```bash
python3 testing_script_extreme.py
```

The testing scripts include:
- **Baseline scenarios**: Normal operation conditions
- **Interference tests**: Various interference patterns  
- **Mobility tests**: Static and mobile node configurations
- **Scalability tests**: Different network sizes (10-80 nodes)
- **Critical traffic tests**: Emergency and critical traffic handling

### Analysis Tools

**Run Analysis:**
```bash
python3 mlo_analyser_script.py
```

**Output Locations:**
- 📊 **CSV Results**: `scratch/output_files_csv/`
- 📈 **Visualizations**: `MLO_Output_Plots`
- 📋 **Log Files**: `scratch/logs/`

## Prerequisites and System Requirements

**Minimum Requirements:**
- Ubuntu 18.04+ (or compatible Linux distribution)
- Python 3.8 or higher
- 8GB RAM minimum
- GCC/G++ compiler
- ns-3.44 (included in project structure)

**Recommended:**
- Ubuntu 20.04+
- Python 3.10+
- 16GB RAM or more
- Multi-core processor for parallel testing

### Key Metrics to Look For:

**📈 Performance Metrics:**
- **PDR (Packet Delivery Ratio)**: Higher is better (aim for >95%)
- **Average Delay**: Lower is better (measured in milliseconds)
- **Throughput**: Higher is better (measured in Mbps)

**🎯 Reliability Metrics:**
- **Reliability Score**: Combined metric (0-100, higher is better)
- **SLA Compliance**: Whether traffic meets service level requirements
- **Recovery Time**: How quickly system recovers from failures

**🔗 MLO-Specific Metrics:**
- **Link Utilization**: How well each frequency band is used
- **Load Balancing Efficiency**: How evenly traffic is distributed

## 🧪 Advanced Usage

### Custom Simulation Scenarios

Create your own test scenarios by modifying parameters:

```bash
# High-density scenario (many nodes, high traffic)
./ns3 run "scratch/mlo_simulator --strategy=SLA-MLO --nWifi=40 --tidCount=48 --simtime=60 --dataRate=200Mbps"

# Mobility scenario (moving nodes)
./ns3 run "scratch/mlo_simulator --strategy=Reliability --nWifi=15 --mobility=true --mobilitySpeed=5.0 --simtime=45"

# Interference scenario (challenging conditions)  
./ns3 run "scratch/mlo_simulator --strategy=Greedy --nWifi=20 --interference=true --interferenceIntensity=0.8 --simtime=40"
```

### Batch Testing

Run multiple scenarios automatically:

```bash
# Test all strategies with same parameters
for strategy in RoundRobin Greedy Reliability SLA-MLO; do
    ./ns3 run "scratch/mlo_simulator --strategy=$strategy --nWifi=20 --tidCount=24 --simtime=30"
done
```

### Advanced Analysis

The analysis script supports various options:
```bash
# Analyze specific CSV files
python3 mlo_analyser_script.py --input custom_results.csv

# Generate specific visualizations only
python3 mlo_analyser_script.py --plots reliability,throughput

# Export results to different formats
python3 mlo_analyser_script.py --format pdf,svg
```

**Simulation Issues:**
```bash
# ns-3 not found - make sure you're in the right directory
pwd  # Should show /path/to/ns-3.44

# Compilation errors - clean and rebuild
./ns3 clean
./ns3 build

# Memory issues - reduce simulation scale
./ns3 run "scratch/mlo_simulator --strategy=RoundRobin --nWifi=4 --tidCount=8 --simtime=15"
```

**Analysis Issues:**
```bash
# Missing visualization files - check if simulations completed
ls scratch/output_files_csv/

# Python package errors - reinstall dependencies
source mlo_venv/bin/activate
pip install --upgrade -r requirements.txt
```

### Performance Tips

**For Limited Resources:**
- Start with `testing_script.py` instead of `testing_script_extreme.py`
- Use smaller values: `--nWifi=4 --tidCount=8 --simtime=15`
- Run single simulations instead of batch tests

**For High-Performance Systems:**
- Use `testing_script_extreme.py` for comprehensive testing
- Run parallel simulations with different seeds
- Increase simulation scale: `--nWifi=80 --tidCount=96 --simtime=60`

## 📝 Research Context

### What This Project Investigates

This simulation framework addresses critical challenges in next-generation Wi-Fi networks:

**🎯 Research Question:** How can different resource allocation strategies optimize Multi-Link Operation for Ultra High Reliability in Wi-Fi 8?

**🔬 Key Research Areas:**
1. **MLO Strategy Comparison**: Quantitative comparison of 4 different approaches
2. **Reliability Analysis**: Focus on Ultra High Reliability requirements for Wi-Fi 8
3. **Performance Trade-offs**: Understanding throughput vs reliability vs latency trade-offs
4. **Scalability Assessment**: How strategies perform under different network sizes and conditions

**📊 Research Contributions:**
- Comprehensive MLO simulation framework for Wi-Fi 8 research
- Comparative analysis of resource allocation strategies
- Performance benchmarks under various network conditions
- Insights for future Wi-Fi 8 protocol development

### Wi-Fi Technology Evolution

**Wi-Fi 7 (IEEE 802.11be) - Current:**
- Introduces Multi-Link Operation (MLO) 
- Simultaneous operation across multiple bands
- Basic reliability improvements

**Wi-Fi 8 (IEEE 802.11bn) - Future:**
- **Ultra High Reliability (UHR)** focus
- Enhanced MLO with better coordination
- Advanced interference management
- Critical for XR, industrial automation, autonomous systems

### Technical Implementation

**Simulation Approach:**
- Built on ns-3.44 network simulator
- Uses IEEE 802.11be (Wi-Fi 7) as foundation for Wi-Fi 8 research
- Multi-band spectrum modeling (2.4/5/6 GHz)
- Traffic differentiation (Emergency, Critical, Best Effort)
- Comprehensive performance monitoring

**Key Innovation:**
The project bridges the gap between current Wi-Fi 7 MLO capabilities and future Wi-Fi 8 UHR requirements through systematic strategy evaluation and performance analysis.

---

## 📄 Project Structure

```
ns-3.44/
├── scratch/
│   ├── mlo_simulator.cc          # Main simulation code
│   ├── output_files_csv/         # Simulation results
|── MLO_Output_Plots/            #Visualization Results
├── requirements.txt             # Python dependencies
├── setup_environment.sh         # Environment setup
├── testing_script.py           # Basic test suite
├── testing_script_extreme.py   # Advanced test suite
├── mlo_analyser_script.py      # Analysis and visualization
└── README.md                   # This file
```

**🎓 For Researchers:**
This framework provides a solid foundation for Wi-Fi 8 MLO research. The modular design allows easy extension for custom scenarios and new allocation strategies.

**💡 For Students:**
Start with the Quick Start guide, run basic simulations, and examine the results. The comprehensive testing framework helps understand different network scenarios and their impact on performance.

**🏢 For Industry:**
The simulation framework can inform the development of next-generation Wi-Fi products and help evaluate MLO strategies for specific use cases and deployment scenarios.

```
