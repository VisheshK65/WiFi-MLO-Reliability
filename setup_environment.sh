#!/bin/bash
# MLO Project Virtual Environment Setup Script
# Simple and straightforward setup for MLO simulations

# Color codes for output
RED='\033[0;31m'
GREEN='\033[0;32m'
YELLOW='\033[1;33m'
BLUE='\033[0;34m'
NC='\033[0m' # No Color

# Get the directory where this script is located
SCRIPT_DIR="$( cd "$( dirname "${BASH_SOURCE[0]}" )" && pwd )"

echo -e "${BLUE}=== MLO Project Virtual Environment Setup ===${NC}"
echo "Setting up in: ${SCRIPT_DIR}"
echo ""

# Check Python version
echo -e "${YELLOW}Checking Python installation...${NC}"
if ! command -v python3 &> /dev/null; then
    echo -e "${RED}Error: Python 3 is not installed${NC}"
    echo "Please install Python 3.8 or higher"
    exit 1
fi

PYTHON_VERSION=$(python3 -c 'import sys; print(".".join(map(str, sys.version_info[:2])))')
PYTHON_MAJOR=$(python3 -c 'import sys; print(sys.version_info[0])')
PYTHON_MINOR=$(python3 -c 'import sys; print(sys.version_info[1])')

echo "Found Python version: ${PYTHON_VERSION}"

# Check if Python version is 3.8 or higher
if [ "$PYTHON_MAJOR" -lt 3 ] || ([ "$PYTHON_MAJOR" -eq 3 ] && [ "$PYTHON_MINOR" -lt 8 ]); then
    echo -e "${RED}Error: Python 3.8 or higher is required${NC}"
    echo "Current version: ${PYTHON_VERSION}"
    exit 1
fi

# Check if python3-venv is installed
if ! python3 -m venv --help &> /dev/null; then
    echo -e "${RED}Error: python3-venv is not installed${NC}"
    echo "Please install it with:"
    echo "  sudo apt-get update"
    echo "  sudo apt-get install -y python3-venv python3-pip"
    exit 1
fi

# Create virtual environment
VENV_NAME="mlo_venv"
VENV_PATH="${SCRIPT_DIR}/${VENV_NAME}"

if [ -d "${VENV_PATH}" ]; then
    echo -e "${YELLOW}Virtual environment already exists at ${VENV_PATH}${NC}"
    read -p "Do you want to recreate it? (y/n): " -n 1 -r
    echo
    if [[ $REPLY =~ ^[Yy]$ ]]; then
        echo "Removing existing virtual environment..."
        rm -rf "${VENV_PATH}"
    else
        echo "Keeping existing virtual environment"
    fi
fi

if [ ! -d "${VENV_PATH}" ]; then
    echo -e "${YELLOW}Creating virtual environment...${NC}"
    python3 -m venv "${VENV_PATH}"
    echo -e "${GREEN}✓ Virtual environment created${NC}"
fi

# Activate virtual environment
echo -e "${YELLOW}Activating virtual environment...${NC}"
source "${VENV_PATH}/bin/activate"

# Upgrade pip
echo -e "${YELLOW}Upgrading pip...${NC}"
python -m pip install --upgrade pip setuptools wheel

# Install requirements
if [ -f "${SCRIPT_DIR}/requirements.txt" ]; then
    echo -e "${YELLOW}Installing Python dependencies...${NC}"
    pip install -r "${SCRIPT_DIR}/requirements.txt"
    echo -e "${GREEN}✓ All Python dependencies installed${NC}"
else
    echo -e "${RED}Error: requirements.txt not found in ${SCRIPT_DIR}${NC}"
    exit 1
fi

# Create necessary directories for output
echo -e "${YELLOW}Creating output directories...${NC}"
mkdir -p "${SCRIPT_DIR}/scratch/output_files_csv"
mkdir -p "${SCRIPT_DIR}/scratch/visualizations"
mkdir -p "${SCRIPT_DIR}/scratch/test_results"
echo -e "${GREEN}✓ Output directories created${NC}"

# Display comprehensive summary
echo ""
echo -e "${GREEN}=== MLO Project Setup Complete! ===${NC}"
echo ""
echo "🎯 MLO Research Environment Successfully Configured"
echo "📁 Virtual environment: ${VENV_PATH}"
echo "🔬 Project: Multi-Link Operation Resource Allocation for Wi-Fi 8"
echo ""
echo -e "${BLUE}=== QUICK START GUIDE ===${NC}"
echo ""
echo -e "${YELLOW}1. ACTIVATE ENVIRONMENT:${NC}"
echo "   source ${VENV_PATH}/bin/activate"
echo ""
echo -e "${YELLOW}2. BUILD NS-3 (if not already built):${NC}"
echo "   ./ns3 configure --enable-examples --enable-tests"
echo "   ./ns3 build"
echo ""
echo -e "${YELLOW}3. RUN BASIC SIMULATION:${NC}"
echo "   ./ns3 run \"scratch/mlo_simulator --strategy=RoundRobin --nWifi=4 --tidCount=8 --simtime=30\""
echo ""
echo -e "${YELLOW}4. RUN COMPREHENSIVE ANALYSIS:${NC}"
echo "   python3 mlo_analyser_script.py"
echo ""
echo -e "${YELLOW}5. RUN TEST SUITES:${NC}"
echo "   # Basic testing (recommended for most systems):"
echo "   python3 testing_script.py"
echo "   # Extreme testing (for high-performance systems):"
echo "   python3 testing_script_extreme.py"
echo ""
echo -e "${BLUE}=== AVAILABLE STRATEGIES ===${NC}"
echo "  • RoundRobin    - Simple round-robin TID mapping"
echo "  • Greedy        - Load-balancing based assignment"
echo "  • Reliability   - Reliability-focused allocation"
echo "  • SLA-MLO       - SLA-optimized resource allocation"
echo ""
echo -e "${BLUE}=== SIMULATION PARAMETERS ===${NC}"
echo "  --strategy      : Strategy type (RoundRobin|Greedy|Reliability|SLA-MLO)"
echo "  --nWifi         : Number of Wi-Fi stations (default: 2)"
echo "  --tidCount      : Number of Traffic IDs (default: 4)"
echo "  --criticalTids  : Critical traffic flows (default: 1)"
echo "  --simtime       : Simulation duration in seconds (default: 10)"
echo "  --mobility      : Enable mobility (true|false)"
echo "  --interference  : Enable interference (true|false)"
echo ""
echo -e "${BLUE}=== OUTPUT LOCATIONS ===${NC}"
echo "  📊 CSV Results: scratch/output_files_csv/"
echo "  📈 Visualizations: scratch/visualizations/"
echo "  🧪 Test Results: scratch/test_results/"
echo ""
echo -e "${YELLOW}6. DEACTIVATE WHEN DONE:${NC}"
echo "   deactivate"
echo ""
echo -e "${GREEN}🚀 Ready to run MLO simulations for Wi-Fi 8 research!${NC}"
echo -e "${YELLOW}💡 Tip: Start with basic simulation, then run analysis for insights${NC}"