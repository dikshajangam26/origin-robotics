#!/usr/bin/env python3
"""
Main Streamlit Dashboard Application
"""

import streamlit as st
import time
from data_processor import DataProcessor
from visualizations import *
from config import *


# Add auto-refresh every 2 seconds
if 'last_refresh' not in st.session_state:
    st.session_state.last_refresh = time.time()

# Auto-refresh every 2 seconds
if time.time() - st.session_state.last_refresh > 2:
    st.rerun()
    st.session_state.last_refresh = time.time()

# Page configuration
st.set_page_config(
    page_title=DASHBOARD_TITLE,
    page_icon=DASHBOARD_ICON,
    layout="wide",
    initial_sidebar_state="expanded"
)

# Custom CSS for better appearance
st.markdown("""
<style>
    .main {
        padding: 2rem;
    }
    .metric-card {
        background-color: #1f1f2e;
        padding: 1.5rem;
        border-radius: 10px;
        border-left: 4px solid #3498db;
        margin: 0.5rem 0;
    }
    h1 {
        color: #3498db;
        text-align: center;
        margin-bottom: 2rem;
    }
    h2 {
        color: #2ecc71;
        border-bottom: 2px solid #3498db;
        padding-bottom: 0.5rem;
    }
</style>
""", unsafe_allow_html=True)

# Title
st.markdown("# Live Navigation Dashboard")
st.markdown("---")

# Sidebar configuration
st.sidebar.markdown("## ⚙️ Settings")
refresh_rate = st.sidebar.slider(
    "Refresh Rate (seconds)",
    0.5, 5.0, 1.0, step=0.5
)
show_advanced = st.sidebar.checkbox("Show Advanced Metrics", value=False)

# Load data
data_processor = DataProcessor()

st.session_state.refresh_counter = st.session_state.get('refresh_counter', 0) + 1

if st.session_state.refresh_counter % 4 == 0:  # Refresh every 2 seconds
    data_processor.load_data()
    st.rerun()

# Auto-refresh
st.sidebar.markdown("---")
auto_refresh = st.sidebar.checkbox("🔄 Auto Refresh", value=True)

if auto_refresh:
    placeholder = st.empty()
    with placeholder.container():
        st.info("📊 Auto-refreshing every 2 seconds...")

# Main dashboard
if data_processor.data is None or len(data_processor.data) == 0:
    st.warning("⏳ Waiting for data... Please make sure your robot is running and CSV file exists.")
    st.info("Expected CSV file at: robot_data.csv")
else:
    # Get latest data
    current_x, current_y, current_theta = data_processor.get_latest_position()
    front_dist, left_dist, right_dist = data_processor.get_lidar_data()
    current_speed = data_processor.get_speed()
    stats = data_processor.get_statistics()
    x_traj, y_traj = data_processor.get_trajectory()
    
    # Top statistics row
    st.markdown("## 📊 Key Metrics")
    
    col1, col2, col3, col4, col5, col6 = st.columns(6)
    
    with col1:
        st.metric("⏱️ Total Time", f"{stats.get('total_time', 0):.1f}s")
    with col2:
        st.metric("📏 Distance", f"{stats.get('total_distance', 0):.1f}m")
    with col3:
        st.metric("🚗 Avg Speed", f"{stats.get('avg_speed', 0):.2f}m/s")
    with col4:
        st.metric("⚡ Current Speed", f"{current_speed:.2f}m/s")
    with col5:
        st.metric("👤 Workers Detected", f"{int(stats.get('worker_detections', 0))}")
    with col6:
        st.metric("🚨 Emergency Stops", f"{int(stats.get('emergency_stops', 0))}")
    
    st.markdown("---")
    
    # Main visualization row
    st.markdown("## 🗺️ Navigation Visualization")
    
    col1, col2 = st.columns([2, 1])
    
    with col1:
        # Robot trajectory map
        fig_map = create_robot_trajectory_map(x_traj, y_traj, current_x, current_y, current_theta)
        st.plotly_chart(fig_map, use_container_width=True)
    
    with col2:
        # LiDAR gauges
        fig_lidar = create_lidar_gauge(front_dist, left_dist, right_dist)
        st.plotly_chart(fig_lidar, use_container_width=True)
    
    st.markdown("---")
    
    # Time series data
    st.markdown("## 📈 Time Series Data")
    
    col1, col2 = st.columns(2)
    
    with col1:
        fig_pos = create_position_time_series(data_processor.data)
        st.plotly_chart(fig_pos, use_container_width=True)
    
    with col2:
        fig_lidar_ts = create_lidar_time_series(data_processor.data)
        st.plotly_chart(fig_lidar_ts, use_container_width=True)
    
    st.markdown("---")
    
    # State timeline
    st.markdown("## ⚙️ Robot State Changes")
    fig_state = create_state_timeline(data_processor.data)
    st.plotly_chart(fig_state, use_container_width=True)

    
    st.markdown("---")
    
    # Advanced metrics (optional)
    if show_advanced:
        st.markdown("## 🔬 Advanced Metrics")
        
        col1, col2, col3 = st.columns(3)
        
        st.markdown("---")
    st.markdown("## 📊 Trajectory Analysis Plots")

    col1, col2, col3 = st.columns(3)

    with col1:
        st.markdown("### Path Smoothing Output")
        fig_smooth = create_path_smoothing_demo()
        st.plotly_chart(fig_smooth, use_container_width=True)

    with col2:
        st.markdown("### Velocity Profile")
        fig_vel = create_velocity_profile_demo()
        st.plotly_chart(fig_vel, use_container_width=True)

    with col3:
        st.markdown("### Trajectory Data")
        # Trajectory visualization will show once more data is collected
        st.info("📊 Trajectory data will display once robot starts moving...")
        st.markdown("---")
    
    # Data table (expandable)
    with st.expander("📋 View Raw Data"):
        st.dataframe(
            data_processor.data,
            use_container_width=True,
            height=400
        )
        
        # Download button
        csv = data_processor.data.to_csv(index=False)
        st.download_button(
            label="⬇️ Download CSV",
            data=csv,
            file_name="robot_data_export.csv",
            mime="text/csv"
        )
    
    st.markdown("---")
    
    # Footer
    st.markdown("""
    <div style='text-align: center; color: #7f8c8d; font-size: 12px; margin-top: 3rem;'>
        <p>Origin Robotics Software Apprentice | Real-time Navigation Dashboard</p>
        <p>Last Updated: """ + time.strftime("%Y-%m-%d %H:%M:%S") + """</p>
    </div>
    """, unsafe_allow_html=True)