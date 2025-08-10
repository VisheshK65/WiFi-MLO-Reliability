#!/usr/bin/env python3
"""
MLO Simulation Basic Test Runner
===============================
This script contains basic and moderate test scenarios for the ns-3 mlo_simulator.
For extreme test cases including mobility+interference combinations, use testing_script_extreme.py
"""

import subprocess
import sys
import os
import time
import multiprocessing
from datetime import datetime
from concurrent.futures import ProcessPoolExecutor, as_completed
from pathlib import Path

class MLOSimulationRunner:
    """
    A framework to run predefined MLO simulation scenarios.
    """
    
    def __init__(self, ns3_dir="."):
        """Initializes the test runner, setting up paths and scenarios."""
        self.ns3_dir = Path(ns3_dir)
        self.output_dir = self.ns3_dir / "scratch" / "output_files_csv"
        self.log_dir = self.ns3_dir / "scratch" / "logs"
        self.output_csv_path = self.output_dir / "mlo_unified_results.csv"
        self.timestamp = datetime.now().strftime("%Y%m%d_%H%M%S")
        
        self.setup_directory_structure()
        
        # Basic and moderate test scenarios (extreme cases moved to testing_script_extreme.py)
        self.scenarios = self.define_test_scenarios()
        self.strategies = ['RoundRobin', 'Greedy', 'Reliability', 'SLA-MLO']
        self.protocols = ['UDP', 'TCP', 'Mixed']
        
    def setup_directory_structure(self):
        """Creates the necessary directories for the output CSV and log files."""
        self.output_dir.mkdir(parents=True, exist_ok=True)
        self.log_dir.mkdir(parents=True, exist_ok=True)
        print(f"📁 Basic Test Output CSV will be saved to: {self.output_csv_path}")
        print(f"📁 Log files will be saved in: {self.log_dir}")

    def define_test_scenarios(self):
        """
        Defines basic and moderate test scenarios for systematic comparison.
        NOTE: Extreme cases have been moved to testing_script_extreme.py
        """
        scenarios = {
            # BASIC PERFORMANCE SCENARIOS
            'baseline': {
                'description': 'Basic performance under normal conditions',
                'tests': [
                    {'name': 'normal_load_critical_high', 'params': '--nWifi=20 --tidCount=24 --criticalTids=0 --emergencyTids=24 --simtime=30 --dataRate=100Mbps', 'description': 'Normal load - Emergency TIDs with Ultra-Critical SLA (1ms threshold)'},
                    {'name': 'normal_load_critical_basic', 'params': '--nWifi=20 --tidCount=24 --criticalTids=24 --emergencyTids=0 --simtime=30 --dataRate=100Mbps', 'description': 'Normal load - Critical Basic SLA (All Critical TIDs)'},
                    {'name': 'normal_load_non_critical', 'params': '--nWifi=20 --tidCount=24 --criticalTids=0 --emergencyTids=0 --simtime=30 --dataRate=100Mbps', 'description': 'Normal load - Non Critical SLA (All Non-Critical TIDs)'},
                    {'name': 'normal_load_mixed_tid', 'params': '--nWifi=20 --tidCount=24 --criticalTids=8 --emergencyTids=8 --simtime=30 --dataRate=100Mbps', 'description': 'Normal load - Mixed TID (Emergency + Critical + Non-Critical)'},
                    {'name': 'high_load_critical_high', 'params': '--nWifi=30 --tidCount=36 --criticalTids=0 --emergencyTids=36 --simtime=30 --dataRate=150Mbps', 'description': 'High load - Emergency TIDs with Ultra-Critical SLA (1ms threshold)'},
                    {'name': 'high_load_critical_basic', 'params': '--nWifi=30 --tidCount=36 --criticalTids=36 --emergencyTids=0 --simtime=30 --dataRate=150Mbps', 'description': 'High load - Critical Basic SLA (All Critical TIDs)'},
                    {'name': 'high_load_non_critical', 'params': '--nWifi=30 --tidCount=36 --criticalTids=0 --emergencyTids=0 --simtime=30 --dataRate=150Mbps', 'description': 'High load - Non Critical SLA (All Non-Critical TIDs)'},
                    {'name': 'high_load_mixed_tid', 'params': '--nWifi=30 --tidCount=36 --criticalTids=12 --emergencyTids=12 --simtime=30 --dataRate=150Mbps', 'description': 'High load - Mixed TID (Emergency + Critical + Non-Critical)'},
                    {'name': 'mixed_traffic_critical_high', 'params': '--nWifi=25 --tidCount=32 --criticalTids=0 --emergencyTids=32 --simtime=30 --dataRate=120Mbps', 'description': 'Mixed traffic - Emergency TIDs with Ultra-Critical SLA (1ms threshold)'},
                    {'name': 'mixed_traffic_critical_basic', 'params': '--nWifi=25 --tidCount=32 --criticalTids=32 --emergencyTids=0 --simtime=30 --dataRate=120Mbps', 'description': 'Mixed traffic - Critical Basic SLA (All Critical TIDs)'},
                    {'name': 'mixed_traffic_non_critical', 'params': '--nWifi=25 --tidCount=32 --criticalTids=0 --emergencyTids=0 --simtime=30 --dataRate=120Mbps', 'description': 'Mixed traffic - Non Critical SLA (All Non-Critical TIDs)'},
                    {'name': 'mixed_traffic_mixed_tid', 'params': '--nWifi=25 --tidCount=32 --criticalTids=11 --emergencyTids=11 --simtime=30 --dataRate=120Mbps', 'description': 'Mixed traffic - Mixed TID (Emergency + Critical + Non-Critical)'}
                ]
            },
            # INTERFERENCE SCENARIOS
            'interference': {
                'description': 'Performance under different interference patterns',
                'tests': [
                    {'name': 'low_interference_critical_high', 'params': '--nWifi=20 --tidCount=24 --criticalTids=0 --emergencyTids=24 --simtime=30 --interference=true --interferenceIntensity=0.3', 'description': 'Low interference - Emergency TIDs with Ultra-Critical SLA (1ms threshold)'},
                    {'name': 'low_interference_critical_basic', 'params': '--nWifi=20 --tidCount=24 --criticalTids=24 --emergencyTids=0 --simtime=30 --interference=true --interferenceIntensity=0.3', 'description': 'Low interference - Critical Basic SLA (All Critical TIDs)'},
                    {'name': 'low_interference_non_critical', 'params': '--nWifi=20 --tidCount=24 --criticalTids=0 --emergencyTids=0 --simtime=30 --interference=true --interferenceIntensity=0.3', 'description': 'Low interference - Non Critical SLA (All Non-Critical TIDs)'},
                    {'name': 'low_interference_mixed_tid', 'params': '--nWifi=20 --tidCount=24 --criticalTids=8 --emergencyTids=8 --simtime=30 --interference=true --interferenceIntensity=0.3', 'description': 'Low interference - Mixed TID (Emergency + Critical + Non-Critical)'},
                    {'name': 'medium_interference_critical_high', 'params': '--nWifi=20 --tidCount=24 --criticalTids=0 --emergencyTids=24 --simtime=30 --interference=true --interferenceIntensity=0.6', 'description': 'Medium interference - Emergency TIDs with Ultra-Critical SLA (1ms threshold)'},
                    {'name': 'medium_interference_critical_basic', 'params': '--nWifi=20 --tidCount=24 --criticalTids=24 --emergencyTids=0 --simtime=30 --interference=true --interferenceIntensity=0.6', 'description': 'Medium interference - Critical Basic SLA (All Critical TIDs)'},
                    {'name': 'medium_interference_non_critical', 'params': '--nWifi=20 --tidCount=24 --criticalTids=0 --emergencyTids=0 --simtime=30 --interference=true --interferenceIntensity=0.6', 'description': 'Medium interference - Non Critical SLA (All Non-Critical TIDs)'},
                    {'name': 'medium_interference_mixed_tid', 'params': '--nWifi=20 --tidCount=24 --criticalTids=8 --emergencyTids=8 --simtime=30 --interference=true --interferenceIntensity=0.6', 'description': 'Medium interference - Mixed TID (Emergency + Critical + Non-Critical)'},
                ]
            },
            # BASIC FAILURE SCENARIOS
            'failure_recovery': {
                'description': 'Basic link failure and recovery performance',
                'tests': [
                    {'name': 'random_failures_mixed_tid', 'params': '--nWifi=20 --tidCount=24 --criticalTids=8 --emergencyTids=8 --simtime=40 --linkFailureRate=0.1 --enableFailureRecovery=true', 'description': 'Random failures (10% chance per second) - Mixed TID'}
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
                ]
            },
            # SCALABILITY SCENARIOS
            'scalability': {
                'description': 'Scalability performance tests with node progression: 10, 20, 25, 30, 40, 50, 60',
                'tests': [
                    # 10 nodes
                    {'name': '10_nodes_mixed_tid', 'params': '--nWifi=10 --tidCount=12 --criticalTids=4 --emergencyTids=4 --simtime=25 --dataRate=50Mbps', 'description': '10 nodes - Mixed TID (Emergency + Critical + Non-Critical)'},
                    # 20 nodes  
                    {'name': '20_nodes_critical_high', 'params': '--nWifi=20 --tidCount=24 --criticalTids=0 --emergencyTids=24 --simtime=30 --dataRate=75Mbps', 'description': '20 nodes - Critical High SLA (All Emergency TIDs)'},
                    {'name': '20_nodes_mixed_tid', 'params': '--nWifi=20 --tidCount=24 --criticalTids=8 --emergencyTids=8 --simtime=30 --dataRate=75Mbps', 'description': '20 nodes - Mixed TID (Emergency + Critical + Non-Critical)'},
                    # 25 nodes
                    {'name': '25_nodes_critical_basic', 'params': '--nWifi=25 --tidCount=30 --criticalTids=30 --emergencyTids=0 --simtime=30 --dataRate=90Mbps', 'description': '25 nodes - Critical Basic SLA (All Critical TIDs)'},
                    {'name': '25_nodes_mixed_tid', 'params': '--nWifi=25 --tidCount=30 --criticalTids=10 --emergencyTids=10 --simtime=30 --dataRate=90Mbps', 'description': '25 nodes - Mixed TID (Emergency + Critical + Non-Critical)'},
                    # 30 nodes
                    {'name': '30_nodes_critical_high', 'params': '--nWifi=30 --tidCount=36 --criticalTids=0 --emergencyTids=36 --simtime=30 --dataRate=100Mbps', 'description': '30 nodes - Critical High SLA (All Emergency TIDs)'},
                    {'name': '30_nodes_mixed_tid', 'params': '--nWifi=30 --tidCount=36 --criticalTids=12 --emergencyTids=12 --simtime=30 --dataRate=100Mbps', 'description': '30 nodes - Mixed TID (Emergency + Critical + Non-Critical)'},
                    # 40 nodes
                    {'name': '40_nodes_critical_basic', 'params': '--nWifi=40 --tidCount=48 --criticalTids=48 --emergencyTids=0 --simtime=35 --dataRate=120Mbps', 'description': '40 nodes - Critical Basic SLA (All Critical TIDs)'},
                    {'name': '40_nodes_mixed_tid', 'params': '--nWifi=40 --tidCount=48 --criticalTids=16 --emergencyTids=16 --simtime=35 --dataRate=120Mbps', 'description': '40 nodes - Mixed TID (Emergency + Critical + Non-Critical)'},
                    # 50 nodes
                    {'name': '50_nodes_critical_high', 'params': '--nWifi=50 --tidCount=60 --criticalTids=0 --emergencyTids=60 --simtime=35 --dataRate=150Mbps', 'description': '50 nodes - Critical High SLA (All Emergency TIDs)'},
                    {'name': '50_nodes_mixed_tid', 'params': '--nWifi=50 --tidCount=60 --criticalTids=20 --emergencyTids=20 --simtime=35 --dataRate=150Mbps', 'description': '50 nodes - Mixed TID (Emergency + Critical + Non-Critical)'},
                    # 60 nodes (maximum for basic testing)
                    {'name': '60_nodes_critical_basic', 'params': '--nWifi=60 --tidCount=72 --criticalTids=72 --emergencyTids=0 --simtime=40 --dataRate=180Mbps', 'description': '60 nodes - Critical Basic SLA (All Critical TIDs)'},
                    {'name': '60_nodes_mixed_tid', 'params': '--nWifi=60 --tidCount=72 --criticalTids=24 --emergencyTids=24 --simtime=40 --dataRate=180Mbps', 'description': '60 nodes - Mixed TID (Emergency + Critical + Non-Critical)'},
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
                ]
            },
            # BASIC NETWORK CONFIGURATION SCENARIOS
            'network_configuration': {
                'description': 'Basic network configuration parameter testing',
                'tests': [
                    {'name': 'basic_ul_ofdma_mixed_tid', 'params': '--nWifi=20 --tidCount=24 --criticalTids=8 --emergencyTids=8 --simtime=30 --enableUlOfdma=true', 'description': 'Basic UL OFDMA - Mixed TID'},
                    {'name': 'bsrp_enabled_mixed_tid', 'params': '--nWifi=25 --tidCount=30 --criticalTids=10 --emergencyTids=10 --simtime=30 --enableBsrp=true --enableUlOfdma=true', 'description': 'BSRP enabled with UL OFDMA - Mixed TID (Buffer status reporting)'},
                    {'name': 'medium_congestion_mixed_protocol', 'params': '--nWifi=25 --tidCount=30 --criticalTids=10 --emergencyTids=10 --simtime=30 --congestionLevel=medium --dataRate=100Mbps', 'description': 'Medium congestion level - Mixed TID (Moderate network stress)'}
                ]
            },
            # BASIC PROTOCOL VARIATION SCENARIOS
            'protocol_variations': {
                'description': 'Basic protocol configuration testing',
                'tests': [
                    {'name': 'tcp_basic_segments_mixed', 'params': '--nWifi=20 --tidCount=24 --criticalTids=8 --emergencyTids=8 --simtime=30 --tcpSegmentSize=1500 --dataRate=100Mbps', 'description': 'TCP standard segments - Mixed TID (1.5KB segments)'},
                    {'name': 'mixed_protocol_basic_mobility', 'params': '--nWifi=20 --tidCount=24 --criticalTids=8 --emergencyTids=8 --simtime=35 --mobility=true --mobilitySpeed=3.0', 'description': 'Mixed protocol with basic mobility - Mixed TID (UDP/TCP alternating)'},
                    {'name': 'mixed_protocol_basic_interference', 'params': '--nWifi=20 --tidCount=24 --criticalTids=8 --emergencyTids=8 --simtime=30 --interference=true --interferenceIntensity=0.4', 'description': 'Mixed protocol with basic interference - Mixed TID'}
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
        
        print(f"🚀 Starting test: {test_name}")
        
        try:
            start_time = time.time()
            # Execute the ns-3 simulation command without a timeout
            result = subprocess.run(
                command, shell=True, cwd=self.ns3_dir,
                capture_output=True, text=True
            )
            execution_time = time.time() - start_time
            
            with open(log_file_path, 'w') as f:
                f.write(f"--- TEST DETAILS ---\n")
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
            
        print("\n" + "="*70)
        print("🎯 MLO BASIC SIMULATION RUNNER INITIALIZING 🎯")
        print("="*70)
        
        categories_to_run = categories or list(self.scenarios.keys())
        strategies_to_test = strategies or self.strategies
        protocols_to_test = protocols or self.protocols
        
        print(f"🧪 Basic Test Categories: {', '.join(categories_to_run)}")
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

        print(f"\nTotal simulation runs to execute: {len(all_tests_to_run)}\n")
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
        
        print("\n" + "="*70)
        print("🏁 MLO BASIC SIMULATION RUN COMPLETE 🏁")
        print("="*70)
        print(f"⏱️  Total execution time: {total_execution_time:.2f} seconds")
        print(f"✅ Successful runs: {successful_runs}")
        print(f"❌ Failed runs: {failed_runs}")
        print(f"📄 All results appended to: {self.output_csv_path}")
        print(f"📁 Detailed logs are in: {self.log_dir}")


if __name__ == "__main__":
    import argparse
    
    parser = argparse.ArgumentParser(description='MLO Basic Simulation Test Runner')
    parser.add_argument('--categories', nargs='+', 
                       choices=['baseline', 'interference', 'failure_recovery', 
                                'mobility', 'scalability', 'critical_performance',
                                'network_configuration', 'protocol_variations'],
                       help='Test categories to run (default: basic categories only)')
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
    
    runner = MLOSimulationRunner()
    
    if args.list_scenarios:
        print("📋 Available Basic Test Scenarios:")
        print("=" * 50)
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
