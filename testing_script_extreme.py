#!/usr/bin/env python3
"""
MLO Simulation Extreme Test Runner
=================================
This script contains all original test scenarios plus additional extreme test cases
that combine mobility and interference for comprehensive stress testing of the
ns-3 mlo_simulator. It includes the most challenging scenarios to test system
robustness under severe conditions.
"""

import subprocess
import time
import multiprocessing
from datetime import datetime
from concurrent.futures import ProcessPoolExecutor, as_completed
from pathlib import Path

class MLOExtremeSimulationRunner:
    """
    A framework to run comprehensive MLO simulation scenarios including extreme test cases.
    """
    
    def __init__(self, ns3_dir="."):
        """Initializes the extreme test runner, setting up paths and scenarios."""
        self.ns3_dir = Path(ns3_dir)
        self.output_dir = self.ns3_dir / "scratch" / "output_files_csv"
        self.log_dir = self.ns3_dir / "scratch" / "logs"
        self.output_csv_path = self.output_dir / "mlo_unified_results.csv"
        self.timestamp = datetime.now().strftime("%Y%m%d_%H%M%S")
        
        self.setup_directory_structure()
        
        # Extended test scenarios including extreme cases
        self.scenarios = self.define_test_scenarios()
        self.strategies = ['RoundRobin', 'Greedy', 'Reliability', 'SLA-MLO']
        self.protocols = ['UDP', 'TCP', 'Mixed']
        
    def setup_directory_structure(self):
        """Creates the necessary directories for the output CSV and log files."""
        self.output_dir.mkdir(parents=True, exist_ok=True)
        self.log_dir.mkdir(parents=True, exist_ok=True)
        print(f"📁 Extreme Test Output CSV will be saved to: {self.output_csv_path}")
        print(f"📁 Log files will be saved in: {self.log_dir}")

    def define_test_scenarios(self):
        """
        Defines the comprehensive set of test scenarios including extreme cases and
        new mobility+interference combinations.
        """
        scenarios = {
            # BASIC PERFORMANCE SCENARIOS
            'baseline': {
                'description': 'Basic performance under normal conditions',
                'tests': [
                    {'name': 'normal_load_critical_high', 'params': '--nWifi=20 --tidCount=24 --criticalTids=0 --emergencyTids=24 --simtime=30 --dataRate=100Mbps', 'description': 'Normal load - Critical High SLA (All Emergency TIDs)'},
                    {'name': 'normal_load_critical_basic', 'params': '--nWifi=20 --tidCount=24 --criticalTids=24 --emergencyTids=0 --simtime=30 --dataRate=100Mbps', 'description': 'Normal load - Critical Basic SLA (All Critical TIDs)'},
                    {'name': 'normal_load_non_critical', 'params': '--nWifi=20 --tidCount=24 --criticalTids=0 --emergencyTids=0 --simtime=30 --dataRate=100Mbps', 'description': 'Normal load - Non Critical SLA (All Non-Critical TIDs)'},
                    {'name': 'normal_load_mixed_tid', 'params': '--nWifi=20 --tidCount=24 --criticalTids=8 --emergencyTids=8 --simtime=30 --dataRate=100Mbps', 'description': 'Normal load - Mixed TID (Emergency + Critical + Non-Critical)'},
                    {'name': 'high_load_critical_high', 'params': '--nWifi=30 --tidCount=36 --criticalTids=0 --emergencyTids=36 --simtime=30 --dataRate=150Mbps', 'description': 'High load - Critical High SLA (All Emergency TIDs)'},
                    {'name': 'high_load_critical_basic', 'params': '--nWifi=30 --tidCount=36 --criticalTids=36 --emergencyTids=0 --simtime=30 --dataRate=150Mbps', 'description': 'High load - Critical Basic SLA (All Critical TIDs)'},
                    {'name': 'high_load_non_critical', 'params': '--nWifi=30 --tidCount=36 --criticalTids=0 --emergencyTids=0 --simtime=30 --dataRate=150Mbps', 'description': 'High load - Non Critical SLA (All Non-Critical TIDs)'},
                    {'name': 'high_load_mixed_tid', 'params': '--nWifi=30 --tidCount=36 --criticalTids=12 --emergencyTids=12 --simtime=30 --dataRate=150Mbps', 'description': 'High load - Mixed TID (Emergency + Critical + Non-Critical)'},
                    {'name': 'mixed_traffic_critical_high', 'params': '--nWifi=25 --tidCount=32 --criticalTids=0 --emergencyTids=32 --simtime=30 --dataRate=120Mbps', 'description': 'Mixed traffic - Critical High SLA (All Emergency TIDs)'},
                    {'name': 'mixed_traffic_critical_basic', 'params': '--nWifi=25 --tidCount=32 --criticalTids=32 --emergencyTids=0 --simtime=30 --dataRate=120Mbps', 'description': 'Mixed traffic - Critical Basic SLA (All Critical TIDs)'},
                    {'name': 'mixed_traffic_non_critical', 'params': '--nWifi=25 --tidCount=32 --criticalTids=0 --emergencyTids=0 --simtime=30 --dataRate=120Mbps', 'description': 'Mixed traffic - Non Critical SLA (All Non-Critical TIDs)'},
                    {'name': 'mixed_traffic_mixed_tid', 'params': '--nWifi=25 --tidCount=32 --criticalTids=11 --emergencyTids=11 --simtime=30 --dataRate=120Mbps', 'description': 'Mixed traffic - Mixed TID (Emergency + Critical + Non-Critical)'}
                ]
            },
            # INTERFERENCE SCENARIOS
            'interference': {
                'description': 'Performance under different interference patterns',
                'tests': [
                    {'name': 'low_interference_critical_high', 'params': '--nWifi=20 --tidCount=24 --criticalTids=0 --emergencyTids=24 --simtime=30 --interference=true --interferenceIntensity=0.3', 'description': 'Low interference - Critical High SLA (All Emergency TIDs)'},
                    {'name': 'low_interference_critical_basic', 'params': '--nWifi=20 --tidCount=24 --criticalTids=24 --emergencyTids=0 --simtime=30 --interference=true --interferenceIntensity=0.3', 'description': 'Low interference - Critical Basic SLA (All Critical TIDs)'},
                    {'name': 'low_interference_non_critical', 'params': '--nWifi=20 --tidCount=24 --criticalTids=0 --emergencyTids=0 --simtime=30 --interference=true --interferenceIntensity=0.3', 'description': 'Low interference - Non Critical SLA (All Non-Critical TIDs)'},
                    {'name': 'low_interference_mixed_tid', 'params': '--nWifi=20 --tidCount=24 --criticalTids=8 --emergencyTids=8 --simtime=30 --interference=true --interferenceIntensity=0.3', 'description': 'Low interference - Mixed TID (Emergency + Critical + Non-Critical)'},
                    {'name': 'medium_interference_critical_high', 'params': '--nWifi=20 --tidCount=24 --criticalTids=0 --emergencyTids=24 --simtime=30 --interference=true --interferenceIntensity=0.6', 'description': 'Medium interference - Critical High SLA (All Emergency TIDs)'},
                    {'name': 'medium_interference_critical_basic', 'params': '--nWifi=20 --tidCount=24 --criticalTids=24 --emergencyTids=0 --simtime=30 --interference=true --interferenceIntensity=0.6', 'description': 'Medium interference - Critical Basic SLA (All Critical TIDs)'},
                    {'name': 'medium_interference_non_critical', 'params': '--nWifi=20 --tidCount=24 --criticalTids=0 --emergencyTids=0 --simtime=30 --interference=true --interferenceIntensity=0.6', 'description': 'Medium interference - Non Critical SLA (All Non-Critical TIDs)'},
                    {'name': 'medium_interference_mixed_tid', 'params': '--nWifi=20 --tidCount=24 --criticalTids=8 --emergencyTids=8 --simtime=30 --interference=true --interferenceIntensity=0.6', 'description': 'Medium interference - Mixed TID (Emergency + Critical + Non-Critical)'},
                    {'name': 'high_interference_critical_high', 'params': '--nWifi=20 --tidCount=24 --criticalTids=0 --emergencyTids=24 --simtime=30 --interference=true --interferenceIntensity=0.9', 'description': 'High interference - Critical High SLA (All Emergency TIDs)'},
                    {'name': 'high_interference_critical_basic', 'params': '--nWifi=20 --tidCount=24 --criticalTids=24 --emergencyTids=0 --simtime=30 --interference=true --interferenceIntensity=0.9', 'description': 'High interference - Critical Basic SLA (All Critical TIDs)'},
                    {'name': 'high_interference_non_critical', 'params': '--nWifi=20 --tidCount=24 --criticalTids=0 --emergencyTids=0 --simtime=30 --interference=true --interferenceIntensity=0.9', 'description': 'High interference - Non Critical SLA (All Non-Critical TIDs)'},
                    {'name': 'high_interference_mixed_tid', 'params': '--nWifi=20 --tidCount=24 --criticalTids=8 --emergencyTids=8 --simtime=30 --interference=true --interferenceIntensity=0.9', 'description': 'High interference - Mixed TID (Emergency + Critical + Non-Critical)'},
                    {'name': 'burst_interference_critical_high', 'params': '--nWifi=20 --tidCount=24 --criticalTids=0 --emergencyTids=24 --simtime=35 --interferencePattern=burst_2.4ghz --interferenceIntensity=0.8', 'description': 'Burst interference - Critical High SLA (All Emergency TIDs)'},
                    {'name': 'burst_interference_critical_basic', 'params': '--nWifi=20 --tidCount=24 --criticalTids=24 --emergencyTids=0 --simtime=35 --interferencePattern=burst_2.4ghz --interferenceIntensity=0.8', 'description': 'Burst interference - Critical Basic SLA (All Critical TIDs)'},
                    {'name': 'burst_interference_non_critical', 'params': '--nWifi=20 --tidCount=24 --criticalTids=0 --emergencyTids=0 --simtime=35 --interferencePattern=burst_2.4ghz --interferenceIntensity=0.8', 'description': 'Burst interference - Non Critical SLA (All Non-Critical TIDs)'},
                    {'name': 'burst_interference_mixed_tid', 'params': '--nWifi=20 --tidCount=24 --criticalTids=8 --emergencyTids=8 --simtime=35 --interferencePattern=burst_2.4ghz --interferenceIntensity=0.8', 'description': 'Burst interference - Mixed TID (Emergency + Critical + Non-Critical)'},
                    {'name': 'burst_5ghz_interference_critical_high', 'params': '--nWifi=20 --tidCount=24 --criticalTids=0 --emergencyTids=24 --simtime=35 --interferencePattern=burst_5ghz --interferenceIntensity=0.7 --burstDuration=2000 --burstInterval=5000', 'description': 'Burst 5GHz interference - Critical High SLA (2s bursts every 5s)'},
                    {'name': 'multiband_burst_interference_mixed', 'params': '--nWifi=25 --tidCount=30 --criticalTids=10 --emergencyTids=10 --simtime=40 --interferencePattern=burst_all --interferenceIntensity=0.9 --burstDuration=1500 --burstInterval=4000', 'description': 'Multi-band burst interference - Mixed TID (All bands affected)'},
                    {'name': 'low_freq_interference_critical_basic', 'params': '--nWifi=20 --tidCount=24 --criticalTids=24 --emergencyTids=0 --simtime=30 --interference=true --interferenceFrequency=low --interferenceIntensity=0.4', 'description': 'Low frequency interference - Critical Basic SLA (Sporadic interference)'},
                    {'name': 'high_freq_interference_emergency', 'params': '--nWifi=15 --tidCount=18 --criticalTids=0 --emergencyTids=18 --simtime=30 --interference=true --interferenceFrequency=high --interferenceIntensity=0.6 --maxInterference=0.9', 'description': 'High frequency interference - Emergency Focus (Frequent high-intensity interference)'},
                    {'name': 'variable_interference_mixed', 'params': '--nWifi=20 --tidCount=24 --criticalTids=8 --emergencyTids=8 --simtime=35 --interferencePattern=gradual --interferenceIntensity=0.3 --maxInterference=0.8', 'description': 'Variable interference intensity - Mixed TID (Gradual intensity changes)'}
                ]
            },
            # LINK FAILURE SCENARIOS
            'failure_recovery': {
                'description': 'Link failure and recovery performance',
                'tests': [
                    {'name': 'random_failures_mixed_tid', 'params': '--nWifi=20 --tidCount=24 --criticalTids=8 --emergencyTids=8 --simtime=40 --linkFailureRate=0.1 --enableFailureRecovery=true', 'description': 'Random failures (10% chance per second) - Mixed TID'},
                    {'name': 'burst_failures_emulated_mixed_tid', 'params': '--nWifi=20 --tidCount=24 --criticalTids=8 --emergencyTids=8 --simtime=40 --linkFailureRate=0.3 --enableFailureRecovery=true', 'description': 'Burst failures emulated with high random failure rate (30%) - Mixed TID'},
                    {'name': 'periodic_failures_emulated_mixed_tid', 'params': '--nWifi=20 --tidCount=24 --criticalTids=8 --emergencyTids=8 --simtime=45 --interferencePattern=burst_all --burstDuration=2000 --burstInterval=10000 --interferenceIntensity=0.95', 'description': 'Periodic failures emulated via interference bursts (2s burst every 10s) - Mixed TID'},
                    {'name': 'long_duration_failures_emergency', 'params': '--nWifi=15 --tidCount=18 --criticalTids=0 --emergencyTids=18 --simtime=60 --linkFailureRate=0.1 --failureDuration=5000 --enableFailureRecovery=true', 'description': 'Long duration failures (5s) - Emergency Focus'}
                ]
            },
            # MOBILITY SCENARIOS
            'mobility': {
                'description': 'Performance under mobility conditions',
                'tests': [
                    {'name': 'static_nodes_critical_high', 'params': '--nWifi=20 --tidCount=24 --criticalTids=0 --emergencyTids=24 --simtime=30 --mobility=false', 'description': 'Static topology - Critical High SLA (All Emergency TIDs)'},
                    {'name': 'static_nodes_critical_basic', 'params': '--nWifi=20 --tidCount=24 --criticalTids=24 --emergencyTids=0 --simtime=30 --mobility=false', 'description': 'Static topology - Critical Basic SLA (All Critical TIDs)'},
                    {'name': 'static_nodes_non_critical', 'params': '--nWifi=20 --tidCount=24 --criticalTids=0 --emergencyTids=0 --simtime=30 --mobility=false', 'description': 'Static topology - Non Critical SLA (All Non-Critical TIDs)'},
                    {'name': 'static_nodes_mixed_tid', 'params': '--nWifi=20 --tidCount=24 --criticalTids=8 --emergencyTids=8 --simtime=30 --mobility=false', 'description': 'Static topology - Mixed TID (Emergency + Critical + Non-Critical)'},
                    {'name': 'low_mobility_critical_high', 'params': '--nWifi=20 --tidCount=24 --criticalTids=0 --emergencyTids=24 --simtime=35 --mobility=true --mobilitySpeed=2.0', 'description': 'Low mobility - Critical High SLA (All Emergency TIDs)'},
                    {'name': 'low_mobility_critical_basic', 'params': '--nWifi=20 --tidCount=24 --criticalTids=24 --emergencyTids=0 --simtime=35 --mobility=true --mobilitySpeed=2.0', 'description': 'Low mobility - Critical Basic SLA (All Critical TIDs)'},
                    {'name': 'low_mobility_non_critical', 'params': '--nWifi=20 --tidCount=24 --criticalTids=0 --emergencyTids=0 --simtime=35 --mobility=true --mobilitySpeed=2.0', 'description': 'Low mobility - Non Critical SLA (All Non-Critical TIDs)'},
                    {'name': 'low_mobility_mixed_tid', 'params': '--nWifi=20 --tidCount=24 --criticalTids=8 --emergencyTids=8 --simtime=35 --mobility=true --mobilitySpeed=2.0', 'description': 'Low mobility - Mixed TID (Emergency + Critical + Non-Critical)'},
                    {'name': 'medium_mobility_critical_high', 'params': '--nWifi=20 --tidCount=24 --criticalTids=0 --emergencyTids=24 --simtime=35 --mobility=true --mobilitySpeed=5.0', 'description': 'Medium mobility - Critical High SLA (All Emergency TIDs)'},
                    {'name': 'medium_mobility_critical_basic', 'params': '--nWifi=20 --tidCount=24 --criticalTids=24 --emergencyTids=0 --simtime=35 --mobility=true --mobilitySpeed=5.0', 'description': 'Medium mobility - Critical Basic SLA (All Critical TIDs)'},
                    {'name': 'medium_mobility_non_critical', 'params': '--nWifi=20 --tidCount=24 --criticalTids=0 --emergencyTids=0 --simtime=35 --mobility=true --mobilitySpeed=5.0', 'description': 'Medium mobility - Non Critical SLA (All Non-Critical TIDs)'},
                    {'name': 'medium_mobility_mixed_tid', 'params': '--nWifi=20 --tidCount=24 --criticalTids=8 --emergencyTids=8 --simtime=35 --mobility=true --mobilitySpeed=5.0', 'description': 'Medium mobility - Mixed TID (Emergency + Critical + Non-Critical)'},
                    {'name': 'high_mobility_critical_high', 'params': '--nWifi=20 --tidCount=24 --criticalTids=0 --emergencyTids=24 --simtime=35 --mobility=true --mobilitySpeed=10.0', 'description': 'High mobility - Critical High SLA (All Emergency TIDs)'},
                    {'name': 'high_mobility_critical_basic', 'params': '--nWifi=20 --tidCount=24 --criticalTids=24 --emergencyTids=0 --simtime=35 --mobility=true --mobilitySpeed=10.0', 'description': 'High mobility - Critical Basic SLA (All Critical TIDs)'},
                    {'name': 'high_mobility_non_critical', 'params': '--nWifi=20 --tidCount=24 --criticalTids=0 --emergencyTids=0 --simtime=35 --mobility=true --mobilitySpeed=10.0', 'description': 'High mobility - Non Critical SLA (All Non-Critical TIDs)'},
                    {'name': 'high_mobility_mixed_tid', 'params': '--nWifi=20 --tidCount=24 --criticalTids=8 --emergencyTids=8 --simtime=35 --mobility=true --mobilitySpeed=10.0', 'description': 'High mobility - Mixed TID (Emergency + Critical + Non-Critical)'},
                    {'name': 'gradual_mobility_critical_high', 'params': '--nWifi=20 --tidCount=24 --criticalTids=0 --emergencyTids=24 --simtime=40 --mobility=true --mobilityPattern=gradual --mobilitySpeed=3.0', 'description': 'Gradual mobility pattern - Critical High SLA (Predictable movement)'},
                    {'name': 'random_mobility_mixed_tid', 'params': '--nWifi=25 --tidCount=30 --criticalTids=10 --emergencyTids=10 --simtime=40 --mobility=true --mobilityPattern=random --mobilitySpeed=7.0', 'description': 'Random mobility pattern - Mixed TID (Unpredictable movement)'},
                    {'name': 'very_high_speed_emergency', 'params': '--nWifi=15 --tidCount=18 --criticalTids=0 --emergencyTids=18 --simtime=30 --mobility=true --mobilitySpeed=15.0', 'description': 'Very high speed mobility - Emergency Only (15 m/s speed)'}
                ]
            },
            # SCALABILITY SCENARIOS
            'scalability': {
                'description': 'Scalability performance tests',
                'tests': [
                    {'name': 'small_network_critical_high', 'params': '--nWifi=10 --tidCount=12 --criticalTids=0 --emergencyTids=12 --simtime=25 --dataRate=50Mbps', 'description': 'Small network - Critical High SLA (All Emergency TIDs)'},
                    {'name': 'small_network_critical_basic', 'params': '--nWifi=10 --tidCount=12 --criticalTids=12 --emergencyTids=0 --simtime=25 --dataRate=50Mbps', 'description': 'Small network - Critical Basic SLA (All Critical TIDs)'},
                    {'name': 'small_network_non_critical', 'params': '--nWifi=10 --tidCount=12 --criticalTids=0 --emergencyTids=0 --simtime=25 --dataRate=50Mbps', 'description': 'Small network - Non Critical SLA (All Non-Critical TIDs)'},
                    {'name': 'small_network_mixed_tid', 'params': '--nWifi=10 --tidCount=12 --criticalTids=4 --emergencyTids=4 --simtime=25 --dataRate=50Mbps', 'description': 'Small network - Mixed TID (Emergency + Critical + Non-Critical)'},
                    {'name': 'medium_network_critical_high', 'params': '--nWifi=30 --tidCount=36 --criticalTids=0 --emergencyTids=36 --simtime=30 --dataRate=100Mbps', 'description': 'Medium network - Critical High SLA (All Emergency TIDs)'},
                    {'name': 'medium_network_critical_basic', 'params': '--nWifi=30 --tidCount=36 --criticalTids=36 --emergencyTids=0 --simtime=30 --dataRate=100Mbps', 'description': 'Medium network - Critical Basic SLA (All Critical TIDs)'},
                    {'name': 'medium_network_non_critical', 'params': '--nWifi=30 --tidCount=36 --criticalTids=0 --emergencyTids=0 --simtime=30 --dataRate=100Mbps', 'description': 'Medium network - Non Critical SLA (All Non-Critical TIDs)'},
                    {'name': 'medium_network_mixed_tid', 'params': '--nWifi=30 --tidCount=36 --criticalTids=12 --emergencyTids=12 --simtime=30 --dataRate=100Mbps', 'description': 'Medium network - Mixed TID (Emergency + Critical + Non-Critical)'},
                    {'name': 'large_network_critical_high', 'params': '--nWifi=50 --tidCount=60 --criticalTids=0 --emergencyTids=60 --simtime=35 --dataRate=150Mbps', 'description': 'Large network - Critical High SLA (All Emergency TIDs)'},
                    {'name': 'large_network_critical_basic', 'params': '--nWifi=50 --tidCount=60 --criticalTids=60 --emergencyTids=0 --simtime=35 --dataRate=150Mbps', 'description': 'Large network - Critical Basic SLA (All Critical TIDs)'},
                    {'name': 'large_network_non_critical', 'params': '--nWifi=50 --tidCount=60 --criticalTids=0 --emergencyTids=0 --simtime=35 --dataRate=150Mbps', 'description': 'Large network - Non Critical SLA (All Non-Critical TIDs)'},
                    {'name': 'large_network_mixed_tid', 'params': '--nWifi=50 --tidCount=60 --criticalTids=20 --emergencyTids=20 --simtime=35 --dataRate=150Mbps', 'description': 'Large network - Mixed TID (Emergency + Critical + Non-Critical)'},
                    {'name': 'very_large_network_critical_high', 'params': '--nWifi=80 --tidCount=96 --criticalTids=0 --emergencyTids=96 --simtime=40 --dataRate=200Mbps', 'description': 'Very large network - Critical High SLA (All Emergency TIDs)'},
                    {'name': 'very_large_network_critical_basic', 'params': '--nWifi=80 --tidCount=96 --criticalTids=96 --emergencyTids=0 --simtime=40 --dataRate=200Mbps', 'description': 'Very large network - Critical Basic SLA (All Critical TIDs)'},
                    {'name': 'very_large_network_non_critical', 'params': '--nWifi=80 --tidCount=96 --criticalTids=0 --emergencyTids=0 --simtime=40 --dataRate=200Mbps', 'description': 'Very large network - Non Critical SLA (All Non-Critical TIDs)'},
                    {'name': 'very_large_network_mixed_tid', 'params': '--nWifi=80 --tidCount=96 --criticalTids=32 --emergencyTids=32 --simtime=40 --dataRate=200Mbps', 'description': 'Very large network - Mixed TID (Emergency + Critical + Non-Critical)'}
                ]
            },
            # CRITICAL TRAFFIC SCENARIOS
            'critical_performance': {
                'description': 'Critical traffic handling capabilities',
                'tests': [
                    {'name': 'low_critical_ratio_critical_high', 'params': '--nWifi=30 --tidCount=40 --criticalTids=0 --emergencyTids=8 --simtime=30 --dataRate=120Mbps', 'description': 'Low critical ratio - Critical High SLA (20% Emergency TIDs)'},
                    {'name': 'low_critical_ratio_critical_basic', 'params': '--nWifi=30 --tidCount=40 --criticalTids=8 --emergencyTids=0 --simtime=30 --dataRate=120Mbps', 'description': 'Low critical ratio - Critical Basic SLA (20% Critical TIDs)'},
                    {'name': 'low_critical_ratio_non_critical', 'params': '--nWifi=30 --tidCount=40 --criticalTids=0 --emergencyTids=0 --simtime=30 --dataRate=120Mbps', 'description': 'Low critical ratio - Non Critical SLA (All Non-Critical TIDs)'},
                    {'name': 'low_critical_ratio_mixed_tid', 'params': '--nWifi=30 --tidCount=40 --criticalTids=4 --emergencyTids=4 --simtime=30 --dataRate=120Mbps', 'description': 'Low critical ratio - Mixed TID (Emergency + Critical + Non-Critical)'},
                    {'name': 'medium_critical_ratio_critical_high', 'params': '--nWifi=30 --tidCount=40 --criticalTids=0 --emergencyTids=16 --simtime=30 --dataRate=120Mbps', 'description': 'Medium critical ratio - Critical High SLA (40% Emergency TIDs)'},
                    {'name': 'medium_critical_ratio_critical_basic', 'params': '--nWifi=30 --tidCount=40 --criticalTids=16 --emergencyTids=0 --simtime=30 --dataRate=120Mbps', 'description': 'Medium critical ratio - Critical Basic SLA (40% Critical TIDs)'},
                    {'name': 'medium_critical_ratio_non_critical', 'params': '--nWifi=30 --tidCount=40 --criticalTids=0 --emergencyTids=0 --simtime=30 --dataRate=120Mbps', 'description': 'Medium critical ratio - Non Critical SLA (All Non-Critical TIDs)'},
                    {'name': 'medium_critical_ratio_mixed_tid', 'params': '--nWifi=30 --tidCount=40 --criticalTids=8 --emergencyTids=8 --simtime=30 --dataRate=120Mbps', 'description': 'Medium critical ratio - Mixed TID (Emergency + Critical + Non-Critical)'},
                    {'name': 'high_critical_ratio_critical_high', 'params': '--nWifi=30 --tidCount=40 --criticalTids=0 --emergencyTids=28 --simtime=30 --dataRate=120Mbps', 'description': 'High critical ratio - Critical High SLA (70% Emergency TIDs)'},
                    {'name': 'high_critical_ratio_critical_basic', 'params': '--nWifi=30 --tidCount=40 --criticalTids=28 --emergencyTids=0 --simtime=30 --dataRate=120Mbps', 'description': 'High critical ratio - Critical Basic SLA (70% Critical TIDs)'},
                    {'name': 'high_critical_ratio_non_critical', 'params': '--nWifi=30 --tidCount=40 --criticalTids=0 --emergencyTids=0 --simtime=30 --dataRate=120Mbps', 'description': 'High critical ratio - Non Critical SLA (All Non-Critical TIDs)'},
                    {'name': 'high_critical_ratio_mixed_tid', 'params': '--nWifi=30 --tidCount=40 --criticalTids=14 --emergencyTids=14 --simtime=30 --dataRate=120Mbps', 'description': 'High critical ratio - Mixed TID (Emergency + Critical + Non-Critical)'},
                    {'name': 'emergency_with_critical_critical_high', 'params': '--nWifi=25 --tidCount=32 --criticalTids=0 --emergencyTids=24 --simtime=35 --dataRate=150Mbps', 'description': 'Emergency with critical - Critical High SLA (75% Emergency TIDs)'},
                    {'name': 'emergency_with_critical_critical_basic', 'params': '--nWifi=25 --tidCount=32 --criticalTids=24 --emergencyTids=0 --simtime=35 --dataRate=150Mbps', 'description': 'Emergency with critical - Critical Basic SLA (75% Critical TIDs)'},
                    {'name': 'emergency_with_critical_non_critical', 'params': '--nWifi=25 --tidCount=32 --criticalTids=0 --emergencyTids=0 --simtime=35 --dataRate=150Mbps', 'description': 'Emergency with critical - Non Critical SLA (All Non-Critical TIDs)'},
                    {'name': 'emergency_with_critical_mixed_tid', 'params': '--nWifi=25 --tidCount=32 --criticalTids=11 --emergencyTids=11 --simtime=35 --dataRate=150Mbps', 'description': 'Emergency with Critical - Mixed TID (Emergency + Critical + Non-Critical)'}
                ]
            },
            # ADVANCED NETWORK CONFIGURATION SCENARIOS
            'network_configuration': {
                'description': 'Advanced network configuration parameter testing',
                'tests': [
                    {'name': 'high_mcs_ul_ofdma_critical_high', 'params': '--nWifi=20 --tidCount=24 --criticalTids=0 --emergencyTids=24 --simtime=30 --mcs=11 --enableUlOfdma=true --channelWidth=160', 'description': 'High MCS with UL OFDMA - Critical High SLA (MCS 11, 160MHz channel)'},
                    {'name': 'bsrp_enabled_mixed_tid', 'params': '--nWifi=25 --tidCount=30 --criticalTids=10 --emergencyTids=10 --simtime=30 --enableBsrp=true --enableUlOfdma=true', 'description': 'BSRP enabled with UL OFDMA - Mixed TID (Buffer status reporting)'},
                    {'name': 'wide_channel_high_congestion_emergency', 'params': '--nWifi=30 --tidCount=36 --criticalTids=0 --emergencyTids=36 --simtime=35 --channelWidth=160 --congestionLevel=high --dataRate=200Mbps', 'description': 'Wide channel with high congestion - Emergency Focus (160MHz under stress)'},
                    {'name': 'low_mcs_high_load_critical_basic', 'params': '--nWifi=40 --tidCount=48 --criticalTids=48 --emergencyTids=0 --simtime=30 --mcs=2 --dataRate=50Mbps', 'description': 'Low MCS with high load - Critical Basic SLA (MCS 2 under load)'},
                    {'name': 'medium_congestion_mixed_protocol', 'params': '--nWifi=25 --tidCount=30 --criticalTids=10 --emergencyTids=10 --simtime=30 --congestionLevel=medium --dataRate=100Mbps', 'description': 'Medium congestion level - Mixed TID (Moderate network stress)'}
                ]
            },
            # PROTOCOL VARIATION SCENARIOS
            'protocol_variations': {
                'description': 'Mixed protocol and advanced protocol configuration testing',
                'tests': [
                    {'name': 'tcp_large_segments_critical_high', 'params': '--nWifi=20 --tidCount=24 --criticalTids=0 --emergencyTids=24 --simtime=30 --tcpSegmentSize=2000 --dataRate=100Mbps', 'description': 'TCP large segments - Critical High SLA (2KB segments)'},
                    {'name': 'tcp_small_segments_high_load_mixed', 'params': '--nWifi=30 --tidCount=36 --criticalTids=12 --emergencyTids=12 --simtime=30 --tcpSegmentSize=500 --dataRate=150Mbps', 'description': 'TCP small segments with high load - Mixed TID (500B segments)'},
                    {'name': 'mixed_protocol_mobility_emergency', 'params': '--nWifi=20 --tidCount=24 --criticalTids=0 --emergencyTids=24 --simtime=35 --mobility=true --mobilitySpeed=5.0', 'description': 'Mixed protocol with mobility - Emergency Focus (UDP/TCP alternating)'},
                    {'name': 'mixed_protocol_interference_critical_basic', 'params': '--nWifi=25 --tidCount=30 --criticalTids=30 --emergencyTids=0 --simtime=30 --interference=true --interferenceIntensity=0.5', 'description': 'Mixed protocol with interference - Critical Basic SLA (Protocol diversity under interference)'}
                ]
            },
            # NEW: COMBINED MOBILITY + INTERFERENCE EXTREME SCENARIOS
            'mobility_interference_extreme': {
                'description': 'Extreme scenarios combining mobility and interference stress testing',
                'tests': [
                    # Low mobility + Various interference levels
                    {'name': 'low_mobility_medium_interference_emergency', 'params': '--nWifi=20 --tidCount=24 --criticalTids=0 --emergencyTids=24 --simtime=40 --mobility=true --mobilitySpeed=2.0 --interference=true --interferenceIntensity=0.6', 'description': 'Low mobility + Medium interference - Emergency SLA (2m/s + 60% interference)'},
                    {'name': 'low_mobility_high_interference_mixed', 'params': '--nWifi=25 --tidCount=30 --criticalTids=10 --emergencyTids=10 --simtime=40 --mobility=true --mobilitySpeed=2.0 --interference=true --interferenceIntensity=0.9', 'description': 'Low mobility + High interference - Mixed TID (2m/s + 90% interference)'},
                    {'name': 'low_mobility_burst_interference_critical', 'params': '--nWifi=20 --tidCount=24 --criticalTids=24 --emergencyTids=0 --simtime=45 --mobility=true --mobilitySpeed=2.0 --interferencePattern=burst_2.4ghz --interferenceIntensity=0.8', 'description': 'Low mobility + Burst interference - Critical Basic (2m/s + 2.4GHz bursts)'},
                    
                    # Medium mobility + Various interference levels
                    {'name': 'medium_mobility_high_interference_emergency', 'params': '--nWifi=25 --tidCount=30 --criticalTids=0 --emergencyTids=30 --simtime=40 --mobility=true --mobilitySpeed=5.0 --interference=true --interferenceIntensity=0.9', 'description': 'Medium mobility + High interference - Emergency SLA (5m/s + 90% interference)'},
                    {'name': 'medium_mobility_burst_all_bands_mixed', 'params': '--nWifi=30 --tidCount=36 --criticalTids=12 --emergencyTids=12 --simtime=45 --mobility=true --mobilitySpeed=5.0 --interferencePattern=burst_all --interferenceIntensity=0.8 --burstDuration=1500 --burstInterval=4000', 'description': 'Medium mobility + Multi-band burst interference - Mixed TID (5m/s + all-band bursts)'},
                    {'name': 'medium_mobility_variable_interference_critical', 'params': '--nWifi=25 --tidCount=30 --criticalTids=30 --emergencyTids=0 --simtime=45 --mobility=true --mobilitySpeed=5.0 --interferencePattern=gradual --interferenceIntensity=0.4 --maxInterference=0.9', 'description': 'Medium mobility + Variable interference - Critical Basic (5m/s + gradual interference changes)'},
                    
                    # High mobility + Various interference levels  
                    {'name': 'high_mobility_high_interference_emergency', 'params': '--nWifi=25 --tidCount=30 --criticalTids=0 --emergencyTids=30 --simtime=40 --mobility=true --mobilitySpeed=10.0 --interference=true --interferenceIntensity=0.9', 'description': 'High mobility + High interference - Emergency SLA (10m/s + 90% interference)'},
                    {'name': 'high_mobility_burst_5ghz_mixed', 'params': '--nWifi=20 --tidCount=24 --criticalTids=8 --emergencyTids=8 --simtime=45 --mobility=true --mobilitySpeed=10.0 --interferencePattern=burst_5ghz --interferenceIntensity=0.8 --burstDuration=2000 --burstInterval=5000', 'description': 'High mobility + 5GHz burst interference - Mixed TID (10m/s + 5GHz bursts)'},
                    {'name': 'high_mobility_high_freq_interference_critical', 'params': '--nWifi=20 --tidCount=24 --criticalTids=24 --emergencyTids=0 --simtime=40 --mobility=true --mobilitySpeed=10.0 --interference=true --interferenceFrequency=high --interferenceIntensity=0.7 --maxInterference=0.9', 'description': 'High mobility + High freq interference - Critical Basic (10m/s + frequent interference)'},
                    
                    # Random mobility patterns + Interference
                    {'name': 'random_mobility_multiband_burst_emergency', 'params': '--nWifi=30 --tidCount=36 --criticalTids=0 --emergencyTids=36 --simtime=50 --mobility=true --mobilityPattern=random --mobilitySpeed=7.0 --interferencePattern=burst_all --interferenceIntensity=0.9 --burstDuration=1000 --burstInterval=3000', 'description': 'Random mobility + Multi-band bursts - Emergency SLA (7m/s random + all-band interference)'},
                    {'name': 'random_mobility_variable_interference_mixed', 'params': '--nWifi=25 --tidCount=30 --criticalTids=10 --emergencyTids=10 --simtime=50 --mobility=true --mobilityPattern=random --mobilitySpeed=8.0 --interferencePattern=gradual --interferenceIntensity=0.3 --maxInterference=0.95', 'description': 'Random mobility + Variable interference - Mixed TID (8m/s random + gradual interference)'},
                    
                    # Very high speed + Extreme interference
                    {'name': 'very_high_speed_extreme_interference_emergency', 'params': '--nWifi=20 --tidCount=24 --criticalTids=0 --emergencyTids=24 --simtime=40 --mobility=true --mobilitySpeed=15.0 --interference=true --interferenceIntensity=0.95', 'description': 'Very high speed + Extreme interference - Emergency Only (15m/s + 95% interference)'},
                    {'name': 'very_high_speed_periodic_failures_mixed', 'params': '--nWifi=25 --tidCount=30 --criticalTids=10 --emergencyTids=10 --simtime=50 --mobility=true --mobilitySpeed=15.0 --interferencePattern=burst_all --burstDuration=3000 --burstInterval=8000 --interferenceIntensity=0.98', 'description': 'Very high speed + Periodic failures - Mixed TID (15m/s + 98% interference bursts)'},
                    
                    # Large networks + Mobility + Interference
                    {'name': 'large_network_mobility_interference_stress', 'params': '--nWifi=50 --tidCount=60 --criticalTids=20 --emergencyTids=20 --simtime=45 --mobility=true --mobilitySpeed=8.0 --interference=true --interferenceIntensity=0.8', 'description': 'Large network mobility + interference stress test (50 nodes, 8m/s + 80% interference)'},
                    {'name': 'very_large_network_extreme_mobility_interference', 'params': '--nWifi=80 --tidCount=96 --criticalTids=32 --emergencyTids=32 --simtime=50 --mobility=true --mobilityPattern=random --mobilitySpeed=12.0 --interferencePattern=burst_all --interferenceIntensity=0.9 --burstDuration=2000 --burstInterval=5000', 'description': 'Very large network extreme test (80 nodes, 12m/s random + multi-band bursts)'},
                    
                    # Protocol variations + Mobility + Interference
                    {'name': 'tcp_mobility_interference_stress', 'params': '--nWifi=25 --tidCount=30 --criticalTids=15 --emergencyTids=15 --simtime=45 --mobility=true --mobilitySpeed=8.0 --interference=true --interferenceIntensity=0.8 --tcpSegmentSize=1500', 'description': 'TCP under mobility + interference stress (8m/s + 80% interference)'},
                    {'name': 'mixed_protocol_extreme_mobility_interference', 'params': '--nWifi=30 --tidCount=36 --criticalTids=12 --emergencyTids=12 --simtime=50 --mobility=true --mobilityPattern=random --mobilitySpeed=10.0 --interferencePattern=burst_all --interferenceIntensity=0.85', 'description': 'Mixed protocols under extreme conditions (10m/s random + burst interference)'}
                ]
            }
        }
        return scenarios
    
    def generate_test_command(self, scenario_category, test_config, strategy, protocol, seed):
        """Generates the full ns-3 command for a specific test run."""
        csv_file_path = self.output_csv_path

        command = (f"./ns3 run 'mlo_simulator "
                   f"--strategy={strategy} "
                   f"--protocol={protocol} "
                   f"--seed={seed} "
                   f"--csvFile={csv_file_path} "
                   f"--scenario={scenario_category}_{test_config['name']} "
                   f"--runNumber={seed} "
                   f"--verbose=1 ")
        
        command += test_config['params']
        command += "'"
        
        test_name = f"{scenario_category}_{test_config['name']}_{strategy}_{protocol}_{seed}"
        return command, test_name, test_config['description']

    def run_single_test(self, test_info):
        """Executes a single simulation run and creates a detailed log file."""
        command, test_name, description = test_info
        
        log_file_path = self.log_dir / f"{test_name}_{self.timestamp}.log"
        
        print(f"🚀 Starting extreme test: {test_name}")
        
        try:
            start_time = time.time()
            result = subprocess.run(
                command, shell=True, cwd=self.ns3_dir,
                capture_output=True, text=True
            )
            execution_time = time.time() - start_time
            
            with open(log_file_path, 'w') as f:
                f.write(f"--- EXTREME TEST DETAILS ---\n")
                f.write(f"Test Name: {test_name}\n")
                f.write(f"Description: {description}\n")
                f.write(f"Timestamp: {datetime.now().isoformat()}\n")
                f.write(f"Execution Time: {execution_time:.2f} seconds\n")
                f.write(f"Return Code: {result.returncode}\n\n")
                f.write(f"--- COMMAND ---\n{command}\n\n")
                f.write("--- STDOUT ---\n")
                f.write(result.stdout)
                f.write("\n--- STDERR ---\n")
                f.write(result.stderr)

            if result.returncode == 0:
                print(f"✅ SUCCESS: {test_name} (took {execution_time:.1f}s)")
                return True
            else:
                print(f"❌ FAILED: {test_name}. See log for details: {log_file_path}")
                return False

        except Exception as e:
            print(f"❌ CRITICAL ERROR: {test_name}. See log for details: {log_file_path}")
            with open(log_file_path, 'a') as f:
                f.write(f"\n--- PYTHON EXCEPTION ---\n{str(e)}")
            return False

    def run_all_tests(self, max_workers=None, categories=None, strategies=None, protocols=None):
        """Runs all specified test scenarios in parallel."""
        if max_workers is None:
            max_workers = multiprocessing.cpu_count()
            
        print("\n" + "="*80)
        print("🎯 MLO EXTREME SIMULATION RUNNER INITIALIZING 🎯")
        print("="*80)
        
        categories_to_run = categories or list(self.scenarios.keys())
        strategies_to_test = strategies or self.strategies
        protocols_to_test = protocols or self.protocols
        
        print(f"🧪 Extreme Test Categories: {', '.join(categories_to_run)}")
        print(f"📊 Strategies: {', '.join(strategies_to_test)}")
        print(f"🌐 Protocols: {', '.join(protocols_to_test)}")
        print(f"⚙️ Parallel Workers: {max_workers}")
        
        all_tests_to_run = []
        test_id_counter = int(self.timestamp.split('_')[1])
        
        for category_name in categories_to_run:
            if category_name not in self.scenarios:
                print(f"⚠️ Warning: Scenario category '{category_name}' not found. Skipping.")
                continue
            
            category = self.scenarios[category_name]
            for test_config in category['tests']:
                for strategy in strategies_to_test:
                    for protocol in protocols_to_test:
                        test_id_counter += 1
                        command, test_name, description = self.generate_test_command(
                            category_name, test_config, strategy, protocol, test_id_counter
                        )
                        all_tests_to_run.append((command, test_name, description))

        print(f"\nTotal extreme simulation runs to execute: {len(all_tests_to_run)}\n")
        overall_start_time = time.time()
        
        successful_runs = 0
        failed_runs = 0

        with ProcessPoolExecutor(max_workers=max_workers) as executor:
            future_to_test = {executor.submit(self.run_single_test, test): test for test in all_tests_to_run}
            
            for future in as_completed(future_to_test):
                try:
                    success = future.result()
                    if success:
                        successful_runs += 1
                    else:
                        failed_runs += 1
                except Exception as exc:
                    print(f"A test generated an exception: {exc}")
                    failed_runs += 1

        total_execution_time = time.time() - overall_start_time
        
        print("\n" + "="*80)
        print("🏁 MLO EXTREME SIMULATION RUN COMPLETE 🏁")
        print("="*80)
        print(f"⏱️  Total execution time: {total_execution_time:.2f} seconds")
        print(f"✅ Successful runs: {successful_runs}")
        print(f"❌ Failed runs: {failed_runs}")
        print(f"📄 All results appended to: {self.output_csv_path}")
        print(f"📁 Detailed logs are in: {self.log_dir}")


if __name__ == "__main__":
    import argparse
    
    parser = argparse.ArgumentParser(description='MLO Extreme Simulation Test Runner')
    parser.add_argument('--categories', nargs='+', 
                       choices=['baseline', 'interference', 'failure_recovery', 
                                'mobility', 'scalability', 'critical_performance',
                                'network_configuration', 'protocol_variations', 
                                'mobility_interference_extreme'],
                       help='Test categories to run (default: all)')
    parser.add_argument('--strategies', nargs='+',
                       choices=['RoundRobin', 'Greedy', 'Reliability', 'SLA-MLO'],
                       help='Strategies to test (default: all)')
    parser.add_argument('--protocols', nargs='+',
                       choices=['UDP', 'TCP', 'Mixed'],
                       help='Protocols to test (default: all)')
    parser.add_argument('--workers', type=int, default=None,
                       help=f'Number of parallel workers (default: all available CPU cores)')
    parser.add_argument('--list-scenarios', action='store_true',
                       help='List all available test scenarios and exit')
    
    args = parser.parse_args()
    
    runner = MLOExtremeSimulationRunner()
    
    if args.list_scenarios:
        print("📋 Available Extreme Test Scenarios:")
        print("=" * 60)
        for category, config in runner.scenarios.items():
            print(f"\n🎯 CATEGORY: {category.upper()}")
            print(f"   Description: {config['description']}")
            print(f"   Tests ({len(config['tests'])}):\n")
            for test in config['tests']:
                print(f"     • {test['name']}: {test['description']}")
    else:
        runner.run_all_tests(
            max_workers=args.workers,
            categories=args.categories,
            strategies=args.strategies,
            protocols=args.protocols
        )