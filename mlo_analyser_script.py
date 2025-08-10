#!/usr/bin/env python3

import pandas as pd
import numpy as np
import matplotlib.pyplot as plt
import seaborn as sns
from scipy import stats
from scipy.stats import mannwhitneyu, ttest_ind, kruskal
import warnings
warnings.filterwarnings('ignore')

from datetime import datetime
import os
from pathlib import Path
import itertools
from collections import defaultdict

# Professional matplotlib configuration for PDF output
plt.style.use('default')
plt.rcParams.update({
    'figure.dpi': 300,
    'savefig.dpi': 300,
    'font.size': 12,
    'axes.titlesize': 14,
    'axes.labelsize': 12,
    'xtick.labelsize': 10,
    'ytick.labelsize': 10,
    'legend.fontsize': 11,
    'figure.titlesize': 16,
    'font.family': 'DejaVu Sans',
    'axes.grid': True,
    'grid.alpha': 0.3,
    'axes.spines.top': False,
    'axes.spines.right': False,
    'axes.axisbelow': True,
    'savefig.bbox': 'tight',
    'savefig.pad_inches': 0.1,
    'savefig.format': 'pdf'
})

# Define strategy ordering (VERY IMPORTANT: Always maintain this order)
STRATEGY_ORDER = ['RoundRobin', 'Greedy', 'SLA-MLO', 'Reliability']

# Define strategy colors for consistency
STRATEGY_COLORS = {
    'RoundRobin': '#FF6B6B',    # Red
    'Greedy': '#4ECDC4',        # Teal  
    'SLA-MLO': '#45B7D1',       # Blue
    'Reliability': '#96CEB4'    # Green
}

# SLA Tier definitions
SLA_TIERS = ['Combined', 'Critical High', 'Critical Basic', 'Non Critical']

# Node count categories for detailed analysis
NODE_CATEGORIES = {
    'Small': (1, 20),      # 1-20 nodes
    'Medium': (21, 40),    # 21-40 nodes  
    'Large': (41, 60)      # 41-60 nodes
}

class MLOEnhancedAnalyzer:
    """Enhanced MLO Analysis with SLA-tier specific analysis and individual PDF plots"""
    
    def __init__(self, base_dir='scratch/output_files_csv', output_dir='MLO_Output_Plots'):
        self.base_dir = Path(base_dir)
        self.output_dir = Path(output_dir)
        self.timestamp = datetime.now().strftime("%Y%m%d_%H%M%S")
        
        # Data storage
        self.unified_data = None
        self.sla_tier_data = {}
        
        # Create output structure
        self.create_output_structure()
        
    def create_output_structure(self):
        """Create directory structure for SLA-tier specific analysis"""
        # Main directories for each SLA tier
        self.tier_dirs = {}
        for tier in SLA_TIERS:
            tier_dir = self.output_dir / tier.replace(' ', '_')
            tier_dir.mkdir(parents=True, exist_ok=True)
            self.tier_dirs[tier] = tier_dir
            
            # Subdirectories for different analysis types
            (tier_dir / 'individual_metrics').mkdir(exist_ok=True)
            (tier_dir / 'network_analysis').mkdir(exist_ok=True)
            (tier_dir / 'node_analysis').mkdir(exist_ok=True)
            (tier_dir / 'link_utilization').mkdir(exist_ok=True)
            
        print(f"✅ Created output structure in: {self.output_dir}")
        
    def load_data(self):
        """Load and prepare data with SLA tier filtering"""
        # Load unified dataset
        unified_path = self.base_dir / 'mlo_unified_results.csv'
        if not unified_path.exists():
            raise FileNotFoundError(f"Unified dataset not found: {unified_path}")
            
        self.unified_data = pd.read_csv(unified_path)
        print(f"📊 Loaded {len(self.unified_data)} records from unified dataset")
        
        # Filter data for each SLA tier
        self.sla_tier_data['Combined'] = self.unified_data.copy()
        self.sla_tier_data['Critical High'] = self.unified_data[
            self.unified_data['SLATier'] == 'Critical High'
        ].copy()
        self.sla_tier_data['Critical Basic'] = self.unified_data[
            self.unified_data['SLATier'] == 'Critical Basic'
        ].copy()
        self.sla_tier_data['Non Critical'] = self.unified_data[
            self.unified_data['SLATier'] == 'Non Critical'
        ].copy()
        
        # Print data distribution
        for tier in SLA_TIERS:
            count = len(self.sla_tier_data[tier])
            print(f"  📈 {tier}: {count} records")
            
    def get_ordered_strategies(self, data):
        """Get strategies in the correct order, filtered by what's available in data"""
        available_strategies = data['Strategy'].unique()
        return [strategy for strategy in STRATEGY_ORDER if strategy in available_strategies]
    
    def create_individual_metric_plots(self):
        """Create individual plots for each metric for each SLA tier"""
        metrics = [
            ('PDR', 'Packet Delivery Ratio (%)', 'PDR'),
            ('AvgDelay', 'Average Delay (ms)', 'Average_Delay'),
            ('Throughput', 'Throughput (Mbps)', 'Throughput'),
            ('CriticalPDR', 'Critical PDR (%)', 'Critical_PDR'),
            ('NonCriticalPDR', 'Non-Critical PDR (%)', 'Non_Critical_PDR'),
            ('ReliabilityScore', 'Reliability Score', 'Reliability_Score')
        ]
        
        for tier in SLA_TIERS:
            data = self.sla_tier_data[tier]
            if data.empty:
                continue
                
            strategies = self.get_ordered_strategies(data)
            
            for metric, ylabel, filename in metrics:
                if metric not in data.columns:
                    continue
                    
                # Create individual plot
                fig, ax = plt.subplots(figsize=(10, 6))
                
                # Prepare data for boxplot in correct order
                plot_data = []
                plot_labels = []
                colors = []
                
                for strategy in strategies:
                    strategy_data = data[data['Strategy'] == strategy][metric].values
                    if len(strategy_data) > 0:
                        plot_data.append(strategy_data)
                        plot_labels.append(strategy)
                        colors.append(STRATEGY_COLORS.get(strategy, '#999999'))
                
                # Create boxplot with proper error handling
                if plot_data and all(len(data) > 0 for data in plot_data):
                    try:
                        box_plot = ax.boxplot(plot_data, labels=plot_labels, patch_artist=True, 
                                              showfliers=True, whis=1.5)
                        
                        # Color the boxes
                        for patch, color in zip(box_plot['boxes'], colors):
                            patch.set_facecolor(color)
                            patch.set_alpha(0.7)
                    except Exception as e:
                        print(f"Warning: Could not create boxplot for {metric} in {tier}: {e}")
                        # Fallback to bar plot if boxplot fails
                        means = [np.mean(data) for data in plot_data]
                        stds = [np.std(data) for data in plot_data]
                        bars = ax.bar(range(len(plot_labels)), means, 
                                      color=colors, alpha=0.7, yerr=stds, capsize=5)
                        ax.set_xticks(range(len(plot_labels)))
                        ax.set_xticklabels(plot_labels)
                
                # Add SLA threshold lines for Average Delay charts
                if metric == 'AvgDelay' and tier != 'Combined':
                    threshold_map = {
                        'Critical High': (1.0, '1ms threshold'),
                        'Critical Basic': (50.0, '50ms threshold'), 
                        'Non Critical': (100.0, '100ms threshold')
                    }
                    
                    if tier in threshold_map:
                        threshold_value, threshold_label = threshold_map[tier]
                        ax.axhline(y=threshold_value, color='red', linestyle='--', linewidth=2, 
                                   label=threshold_label, alpha=0.8)
                        ax.legend(loc='upper right')
                
                ax.set_ylabel(ylabel)
                ax.set_xlabel('Strategy')
                ax.set_title(f'{ylabel} by Strategy - {tier}')
                ax.grid(True, alpha=0.3)
                
                # Save as PDF
                output_path = self.tier_dirs[tier] / 'individual_metrics' / f'{filename}.pdf'
                plt.savefig(output_path, format='pdf', bbox_inches='tight')
                plt.close()
                
        print("✅ Created individual metric plots for all SLA tiers")
        
    def create_network_resilience_analysis(self):
        """Create individual network resilience analysis plots for all SLA tiers"""
        for tier in SLA_TIERS:
            data = self.sla_tier_data[tier]
            if data.empty:
                continue
                
            strategies = self.get_ordered_strategies(data)
            
            # Network resilience metrics - now create individual plots
            metrics = [
                ('RecoveryTimeMs', 'Recovery Time (ms)', 'recovery_time'),
                ('AvgJitterMs', 'Jitter (ms)', 'jitter'),
                ('TailLatencyMs', 'Tail Latency (ms)', 'tail_latency'),
                ('LoadBalancingEfficiency', 'Load Balancing Efficiency', 'load_balancing_efficiency')
            ]
            
            for metric, ylabel, filename in metrics:
                if metric not in data.columns:
                    continue
                    
                # Create individual plot for each metric
                fig, ax = plt.subplots(figsize=(10, 6))
                
                plot_data = []
                plot_labels = []
                colors = []
                
                for strategy in strategies:
                    strategy_data = data[data['Strategy'] == strategy][metric].values
                    if len(strategy_data) > 0:
                        plot_data.append(strategy_data)
                        plot_labels.append(strategy)
                        colors.append(STRATEGY_COLORS.get(strategy, '#999999'))
                
                if plot_data and all(len(data) > 0 for data in plot_data):
                    try:
                        box_plot = ax.boxplot(plot_data, labels=plot_labels, patch_artist=True,
                                              showfliers=True, whis=1.5)
                        for patch, color in zip(box_plot['boxes'], colors):
                            patch.set_facecolor(color)
                            patch.set_alpha(0.7)
                    except Exception as e:
                        print(f"Warning: Could not create boxplot for {metric} in {tier}: {e}")
                        # Fallback to bar plot if boxplot fails
                        means = [np.mean(data) for data in plot_data]
                        stds = [np.std(data) for data in plot_data]
                        bars = ax.bar(range(len(plot_labels)), means, 
                                      color=colors, alpha=0.7, yerr=stds, capsize=5)
                        ax.set_xticks(range(len(plot_labels)))
                        ax.set_xticklabels(plot_labels)
                
                ax.set_ylabel(ylabel)
                ax.set_xlabel('Strategy')
                ax.set_title(f'{ylabel} by Strategy - {tier}')
                ax.grid(True, alpha=0.3)
                
                # Save as individual PDF
                output_path = self.tier_dirs[tier] / 'network_analysis' / f'{filename}.pdf'
                plt.savefig(output_path, format='pdf', bbox_inches='tight')
                plt.close()
            
        print("✅ Created individual network resilience analysis plots")
        
    def create_reliability_trend_analysis(self):
        """Create reliability trend analysis plots for all SLA tiers"""
        for tier in SLA_TIERS:
            data = self.sla_tier_data[tier]
            if data.empty:
                continue
                
            strategies = self.get_ordered_strategies(data)
            
            # Group by scenario type for trend analysis
            scenario_types = data['ScenarioType'].unique()
            
            fig, ax = plt.subplots(figsize=(12, 8))
            
            for strategy in strategies:
                strategy_data = data[data['Strategy'] == strategy]
                means = []
                scenario_labels = []
                
                for scenario_type in sorted(scenario_types):
                    scenario_data = strategy_data[strategy_data['ScenarioType'] == scenario_type]
                    if not scenario_data.empty:
                        means.append(scenario_data['ReliabilityScore'].mean())
                        scenario_labels.append(scenario_type)
                
                if means:
                    ax.plot(scenario_labels, means, marker='o', linewidth=2, markersize=6,
                            label=strategy, color=STRATEGY_COLORS.get(strategy, '#999999'))
            
            # Set proper y-axis range to show full reliability score range
            ax.set_ylim(0, 100)
            ax.set_ylabel('Reliability Score')
            ax.set_xlabel('Scenario Type')
            ax.set_title(f'Reliability Trend Analysis - {tier}')
            ax.legend()
            ax.grid(True, alpha=0.3)
            plt.xticks(rotation=45)
            
            plt.tight_layout()
            output_path = self.tier_dirs[tier] / 'network_analysis' / 'reliability_trend.pdf'
            plt.savefig(output_path, format='pdf', bbox_inches='tight')
            plt.close()
            
        print("✅ Created reliability trend analysis plots")
        
    def create_sla_deviation_analysis(self):
        """Create tier-specific SLA deviation analysis plots - only relevant metric per tier"""
        # Skip Combined tier for SLA deviation analysis
        for tier in ['Critical High', 'Critical Basic', 'Non Critical']:
            data = self.sla_tier_data[tier]
            if data.empty:
                continue
                
            strategies = self.get_ordered_strategies(data)
            
            # Map each tier to its relevant SLA deviation metric
            tier_metric_map = {
                'Critical High': ('CriticalHighSLADeviation', 'Critical High SLA Deviation', '1ms threshold'),
                'Critical Basic': ('CriticalBasicSLADeviation', 'Critical Basic SLA Deviation', '50ms threshold'),
                'Non Critical': ('NonCriticalSLADeviation', 'Non-Critical SLA Deviation', '100ms threshold')
            }
            
            if tier not in tier_metric_map:
                continue
                
            metric, title, threshold_desc = tier_metric_map[tier]
            
            if metric not in data.columns:
                continue
                
            # Create single plot for the relevant metric
            fig, ax = plt.subplots(figsize=(10, 6))
            fig.suptitle(f'{title} - {tier} Tier', fontsize=16)
            
            plot_data = []
            plot_labels = []
            colors = []
            
            for strategy in strategies:
                strategy_data = data[data['Strategy'] == strategy][metric].values
                if len(strategy_data) > 0:
                    plot_data.append(strategy_data)
                    plot_labels.append(strategy)
                    colors.append(STRATEGY_COLORS.get(strategy, '#999999'))
            
            if plot_data and all(len(data) > 0 for data in plot_data):
                try:
                    box_plot = ax.boxplot(plot_data, labels=plot_labels, patch_artist=True,
                                          showfliers=True, whis=1.5)
                    for patch, color in zip(box_plot['boxes'], colors):
                        patch.set_facecolor(color)
                        patch.set_alpha(0.7)
                except Exception as e:
                    print(f"Warning: Could not create boxplot for {metric} in {tier}: {e}")
                    # Fallback to bar plot if boxplot fails
                    means = [np.mean(data) for data in plot_data]
                    stds = [np.std(data) for data in plot_data]
                    bars = ax.bar(range(len(plot_labels)), means, 
                                  color=colors, alpha=0.7, yerr=stds, capsize=5)
                    ax.set_xticks(range(len(plot_labels)))
                    ax.set_xticklabels(plot_labels)
            
            ax.set_ylabel('SLA Deviation')
            ax.set_xlabel('Strategy')
            ax.set_title(f'{title}\n({threshold_desc})')
            ax.grid(True, alpha=0.3)
            
            plt.tight_layout()
            output_path = self.tier_dirs[tier] / 'network_analysis' / 'sla_deviation.pdf'
            plt.savefig(output_path, format='pdf', bbox_inches='tight')
            plt.close()
            
        print("✅ Created tier-specific SLA deviation analysis plots")
        
    def create_detailed_node_count_analysis(self):
        """Create detailed node count analysis with proper boxplot categorization"""
        for tier in SLA_TIERS:
            data = self.sla_tier_data[tier]
            if data.empty:
                continue
                
            strategies = self.get_ordered_strategies(data)
            
            # Categorize nodes based on the NODE_CATEGORIES dictionary
            def categorize_nodes(node_count):
                if 1 <= node_count <= 20:
                    return 'Small'
                elif 21 <= node_count <= 40:
                    return 'Medium'
                elif 41 <= node_count <= 60:
                    return 'Large'
                return 'Other' # Handle cases outside defined ranges
            
            data['NodeCategory'] = data['NodeCount'].apply(categorize_nodes)
            
            # Define metrics with their proper labels for plotting
            metrics = {
                'PDR': 'PDR (%)',
                'AvgDelay': 'Average Delay (ms)',
                'Throughput': 'Throughput (Mbps)'
            }

            # --- CORRECTED SECTION ---
            # Define descriptive titles for each node category
            category_titles = {
                'Small': 'Small Networks (1-20 Nodes)',
                'Medium': 'Medium Networks (21-40 Nodes)',
                'Large': 'Large Networks (41-60 Nodes)'
            }

            for metric, ylabel in metrics.items():
                if metric not in data.columns:
                    continue
                    
                fig, axes = plt.subplots(1, 3, figsize=(18, 6))
                fig.suptitle(f'{ylabel} by Node Category - {tier}', fontsize=16)
                
                # Use the keys from the titles dictionary to ensure order
                categories = category_titles.keys()
                
                for cat_idx, category in enumerate(categories):
                    ax = axes[cat_idx]
                    cat_data = data[data['NodeCategory'] == category]
                    
                    if cat_data.empty:
                        ax.text(0.5, 0.5, f'No data for {category}', 
                                ha='center', va='center', transform=ax.transAxes)
                        # Set title even if there's no data
                        ax.set_title(category_titles[category])
                        continue
                    
                    plot_data = []
                    plot_labels = []
                    colors = []
                    
                    for strategy in strategies:
                        strategy_data = cat_data[cat_data['Strategy'] == strategy][metric].values
                        if len(strategy_data) > 0:
                            plot_data.append(strategy_data)
                            plot_labels.append(strategy)
                            colors.append(STRATEGY_COLORS.get(strategy, '#999999'))
                    
                    # Create boxplot with error handling
                    if plot_data and all(len(data_arr) > 0 for data_arr in plot_data):
                        try:
                            box_plot = ax.boxplot(plot_data, labels=plot_labels, patch_artist=True,
                                                  showfliers=True, whis=1.5)
                            
                            # Color the boxes
                            for patch, color in zip(box_plot['boxes'], colors):
                                patch.set_facecolor(color)
                                patch.set_alpha(0.7)
                        except Exception as e:
                            print(f"Warning: Could not create boxplot for {metric} in {tier} {category}: {e}")
                            # Fallback to bar plot
                            means = [np.mean(data_arr) for data_arr in plot_data]
                            stds = [np.std(data_arr) for data_arr in plot_data]
                            bars = ax.bar(range(len(plot_labels)), means, 
                                          color=colors, alpha=0.7, yerr=stds, capsize=5)
                            ax.set_xticks(range(len(plot_labels)))
                            ax.set_xticklabels(plot_labels)
                    
                    ax.set_xlabel('Strategy')
                    ax.set_ylabel(ylabel)
                    # Use the descriptive title from the dictionary
                    ax.set_title(category_titles[category])
                    ax.grid(True, alpha=0.3)
                    plt.setp(ax.get_xticklabels(), rotation=45)
                
                plt.tight_layout()
                output_path = self.tier_dirs[tier] / 'node_analysis' / f'node_count_{metric}.pdf'
                plt.savefig(output_path, format='pdf', bbox_inches='tight')
                plt.close()
                
        print("✅ Created detailed node count analysis plots")
        
    def create_multi_link_utilization_plots(self):
        """Create multi-link utilization plots for all SLA tiers - usage only"""
        for tier in SLA_TIERS:
            data = self.sla_tier_data[tier]
            if data.empty:
                continue
                
            strategies = self.get_ordered_strategies(data)
            
            # Link utilization analysis - only usage, no throughput
            link_cols = ['Link0Usage', 'Link1Usage', 'Link2Usage']
            if not all(col in data.columns for col in link_cols):
                continue
                
            fig, ax = plt.subplots(figsize=(12, 8))
            fig.suptitle(f'Multi-Link Usage Analysis - {tier}', fontsize=16)
            
            # Link Usage Distribution
            x_pos = np.arange(len(strategies))
            width = 0.25
            
            link_names = ['2.4GHz', '5GHz', '6GHz']
            link_colors = ['#FF9999', '#99CCFF', '#99FF99']
            
            for link_idx, (link_col, link_name, link_color) in enumerate(zip(link_cols, link_names, link_colors)):
                means = []
                stds = []
                
                for strategy in strategies:
                    strategy_data = data[data['Strategy'] == strategy][link_col]
                    if not strategy_data.empty:
                        means.append(strategy_data.mean())
                        stds.append(strategy_data.std())
                    else:
                        means.append(0)
                        stds.append(0)
                
                ax.bar(x_pos + link_idx * width, means, width, 
                       label=link_name, color=link_color, alpha=0.7)
            
            ax.set_xlabel('Strategy')
            ax.set_ylabel('Link Usage (%)')
            ax.set_title('Average Link Usage by Strategy')
            ax.set_xticks(x_pos + width)
            ax.set_xticklabels(strategies)
            ax.legend()
            ax.grid(True, alpha=0.3)
            
            plt.tight_layout()
            output_path = self.tier_dirs[tier] / 'link_utilization' / 'multi_link_utilization.pdf'
            plt.savefig(output_path, format='pdf', bbox_inches='tight')
            plt.close()
            
        print("✅ Created multi-link utilization plots")
        
    def run_complete_analysis(self):
        """Run the complete enhanced MLO analysis"""
        print("🚀 Starting Enhanced MLO Analysis...")
        print(f"📊 Base directory: {self.base_dir}")
        print(f"📁 Output directory: {self.output_dir}")
        print(f"⏰ Timestamp: {self.timestamp}")
        
        try:
            # Load and prepare data
            self.load_data()
            
            # Create all analysis plots
            print("\n📈 Creating individual metric plots...")
            self.create_individual_metric_plots()
            
            print("\n🔧 Creating network resilience analysis...")
            self.create_network_resilience_analysis()
            
            print("\n📊 Creating reliability trend analysis...")
            self.create_reliability_trend_analysis()
            
            print("\n⚖️ Creating SLA deviation analysis...")
            self.create_sla_deviation_analysis()
            
            print("\n🏗️ Creating detailed node count analysis...")
            self.create_detailed_node_count_analysis()
            
            print("\n🔗 Creating multi-link utilization plots...")
            self.create_multi_link_utilization_plots()
            
            print(f"\n✅ Enhanced MLO Analysis Complete!")
            print(f"📁 All PDF plots saved in: {self.output_dir}")
            print(f"📊 Analysis covers {len(SLA_TIERS)} SLA tiers:")
            for tier in SLA_TIERS:
                count = len(self.sla_tier_data[tier])
                print(f"   • {tier}: {count} records")
                
        except Exception as e:
            print(f"❌ Analysis failed: {e}")
            import traceback
            traceback.print_exc()
            raise

def main():
    """Main execution function"""
    print("Enhanced MLO Complete Analysis Script")
    print("=" * 45)
    print("Comprehensive MLO performance analysis with SLA-tier specific plots")
    print("All plots saved as individual PDF files for LaTeX integration")
    print("")
    
    # Configuration
    base_dir = 'scratch/output_files_csv'
    output_dir = 'MLO_Output_Plots'
    
    try:
        analyzer = MLOEnhancedAnalyzer(base_dir=base_dir, output_dir=output_dir)
        analyzer.run_complete_analysis()
        
    except Exception as e:
        print(f"❌ Analysis failed: {e}")
        import traceback
        traceback.print_exc()

if __name__ == "__main__":
    main()
