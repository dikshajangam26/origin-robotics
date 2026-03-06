#!/usr/bin/env python3
"""
Create beautiful visualizations
"""

import plotly.graph_objects as go
import plotly.express as px
import pandas as pd
import numpy as np
from config import *

def create_robot_trajectory_map(x_positions, y_positions, current_x, current_y, current_theta):
    """Create beautiful robot trajectory map"""
    
    fig = go.Figure()
    
    # Add trajectory path
    fig.add_trace(go.Scatter(
        x=x_positions,
        y=y_positions,
        mode='lines',
        name='Robot Path',
        line=dict(
            color=COLOR_PATH,
            width=3,
        ),
        hovertemplate='<b>Position</b><br>X: %{x:.2f}m<br>Y: %{y:.2f}m<extra></extra>'
    ))
    
    # Add current position
    fig.add_trace(go.Scatter(
        x=[current_x],
        y=[current_y],
        mode='markers+text',
        name='Robot Position',
        marker=dict(
            size=20,
            color=COLOR_ROBOT,
            symbol='circle'
        ),
        text=['ROBOT'],
        textposition='top center',
        hovertemplate='<b>Current Position</b><br>X: %{x:.2f}m<br>Y: %{y:.2f}m<extra></extra>'
    ))
    
    # Add robot heading arrow
    arrow_length = 2.0
    arrow_x = current_x + arrow_length * np.cos(current_theta)
    arrow_y = current_y + arrow_length * np.sin(current_theta)
    
    fig.add_annotation(
        x=arrow_x,
        y=arrow_y,
        ax=current_x,
        ay=current_y,
        xref='x',
        yref='y',
        axref='x',
        ayref='y',
        showarrow=True,
        arrowhead=2,
        arrowsize=2,
        arrowwidth=3,
        arrowcolor=COLOR_ROBOT,
        text='',
    )
    
    # Add start point
    if len(x_positions) > 0:
        fig.add_trace(go.Scatter(
            x=[x_positions[0]],
            y=[y_positions[0]],
            mode='markers+text',
            name='Start',
            marker=dict(size=12, color=COLOR_SAFE, symbol='diamond'),
            text=['START'],
            textposition='top center',
            showlegend=False,
            hovertemplate='<b>Start Position</b><extra></extra>'
        ))
    
    fig.update_layout(
        title='<b>🗺️ Robot Trajectory Map</b>',
        xaxis_title='X Position (meters)',
        yaxis_title='Y Position (meters)',
        height=600,
        template=CHART_TEMPLATE,
        hovermode='closest',
        showlegend=True,
        xaxis=dict(
            scaleanchor="y",
            scaleratio=1,
            range=[-5, MAP_SIZE_X]
        ),
        yaxis=dict(
            scaleanchor="x",
            scaleratio=1,
            range=[-5, MAP_SIZE_Y]
        ),
    )
    
    return fig

def create_lidar_gauge(front_distance, left_distance, right_distance):
    """Create LiDAR proximity gauges"""
    
    fig = go.Figure()
    
    # Determine colors based on distance
    def get_color(distance):
        if distance < OBSTACLE_THRESHOLD:
            return COLOR_DANGER
        elif distance < WARNING_THRESHOLD:
            return COLOR_WARNING
        else:
            return COLOR_SAFE
    
    # Create three gauges
    gauges_data = [
        {'label': 'Front', 'value': front_distance, 'gauge_min': 0, 'gauge_max': 20},
        {'label': 'Left', 'value': left_distance, 'gauge_min': 0, 'gauge_max': 20},
        {'label': 'Right', 'value': right_distance, 'gauge_min': 0, 'gauge_max': 20},
    ]
    
    colors = [COLOR_SAFE, COLOR_SAFE, COLOR_SAFE]
    
    fig = go.Figure(data=[
        go.Indicator(
            domain={'x': [0, 0.3], 'y': [0, 1]},
            value=front_distance,
            title={'text': "Front (m)", 'font': {'size': 16}},
            gauge={
                'axis': {'range': [0, 20]},
                'bar': {'color': get_color(front_distance)},
                'steps': [
                    {'range': [0, OBSTACLE_THRESHOLD], 'color': COLOR_DANGER},
                    {'range': [OBSTACLE_THRESHOLD, WARNING_THRESHOLD], 'color': COLOR_WARNING},
                    {'range': [WARNING_THRESHOLD, 20], 'color': COLOR_SAFE}
                ],
                'threshold': {
                    'line': {'color': "red", 'width': 4},
                    'thickness': 0.75,
                    'value': OBSTACLE_THRESHOLD
                }
            }
        ),
        go.Indicator(
            domain={'x': [0.35, 0.65], 'y': [0, 1]},
            value=left_distance,
            title={'text': "Left (m)", 'font': {'size': 16}},
            gauge={
                'axis': {'range': [0, 20]},
                'bar': {'color': get_color(left_distance)},
                'steps': [
                    {'range': [0, OBSTACLE_THRESHOLD], 'color': COLOR_DANGER},
                    {'range': [OBSTACLE_THRESHOLD, WARNING_THRESHOLD], 'color': COLOR_WARNING},
                    {'range': [WARNING_THRESHOLD, 20], 'color': COLOR_SAFE}
                ],
                'threshold': {
                    'line': {'color': "red", 'width': 4},
                    'thickness': 0.75,
                    'value': OBSTACLE_THRESHOLD
                }
            }
        ),
        go.Indicator(
            domain={'x': [0.7, 1], 'y': [0, 1]},
            value=right_distance,
            title={'text': "Right (m)", 'font': {'size': 16}},
            gauge={
                'axis': {'range': [0, 20]},
                'bar': {'color': get_color(right_distance)},
                'steps': [
                    {'range': [0, OBSTACLE_THRESHOLD], 'color': COLOR_DANGER},
                    {'range': [OBSTACLE_THRESHOLD, WARNING_THRESHOLD], 'color': COLOR_WARNING},
                    {'range': [WARNING_THRESHOLD, 20], 'color': COLOR_SAFE}
                ],
                'threshold': {
                    'line': {'color': "red", 'width': 4},
                    'thickness': 0.75,
                    'value': OBSTACLE_THRESHOLD
                }
            }
        )
    ])
    
    fig.update_layout(
        title='<b>📡 LiDAR Obstacle Detection</b>',
        height=350,
        template=CHART_TEMPLATE,
    )
    
    return fig

def create_position_time_series(data):
    """Create position over time graph"""
    
    fig = go.Figure()
    
    # X position
    fig.add_trace(go.Scatter(
        x=data['Time_s'],
        y=data['Pos_X'],
        name='X Position',
        line=dict(color='#3498db', width=2),
        mode='lines',
        hovertemplate='Time: %{x:.2f}s<br>X: %{y:.2f}m<extra></extra>'
    ))
    
    # Y position
    fig.add_trace(go.Scatter(
        x=data['Time_s'],
        y=data['Pos_Y'],
        name='Y Position',
        line=dict(color='#e74c3c', width=2),
        mode='lines',
        hovertemplate='Time: %{x:.2f}s<br>Y: %{y:.2f}m<extra></extra>'
    ))
    
    fig.update_layout(
        title='<b>📍 Position Over Time</b>',
        xaxis_title='Time (seconds)',
        yaxis_title='Position (meters)',
        height=350,
        template=CHART_TEMPLATE,
        hovermode='x unified',
    )
    
    return fig

def create_lidar_time_series(data):
    """Create LiDAR readings over time"""
    
    fig = go.Figure()
    
    fig.add_trace(go.Scatter(
        x=data['Time_s'],
        y=data['Lidar_Front'],
        name='Front',
        line=dict(color='#3498db', width=2),
        fill='tozeroy',
    ))
    
    fig.add_trace(go.Scatter(
        x=data['Time_s'],
        y=data['Lidar_Left'],
        name='Left',
        line=dict(color='#2ecc71', width=2),
        fill='tozeroy',
    ))
    
    fig.add_trace(go.Scatter(
        x=data['Time_s'],
        y=data['Lidar_Right'],
        name='Right',
        line=dict(color='#f39c12', width=2),
        fill='tozeroy',
    ))
    
    # Add danger zone
    fig.add_hline(
        y=OBSTACLE_THRESHOLD,
        line_dash='dash',
        line_color='red',
        annotation_text='⚠️ Danger Zone',
        annotation_position='right'
    )
    
    fig.update_layout(
        title='<b>🛡️ LiDAR Distance Over Time</b>',
        xaxis_title='Time (seconds)',
        yaxis_title='Distance (meters)',
        height=350,
        template=CHART_TEMPLATE,
        hovermode='x unified',
    )
    
    return fig

def create_state_timeline(data):
    """Create state changes timeline"""
    
    # Get unique states and times
    states = data[['Time_s', 'State']]
    state_changes = states[states['State'].shift() != states['State']].copy()
    
    if len(state_changes) == 0:
        state_changes = states.iloc[[0]]
    
    # Map states to colors
    state_colors = {
        'FORWARD': '#2ecc71',
        'WORKER_WAIT': '#f39c12',
        'EMERGENCY': '#e74c3c',
        'U_TURN': '#9b59b6'
    }
    
    fig = go.Figure()
    
    for i, (idx, row) in enumerate(state_changes.iterrows()):
        if i < len(state_changes) - 1:
            end_time = state_changes.iloc[i+1]['Time_s']
        else:
            end_time = data['Time_s'].max()
        
        fig.add_trace(go.Bar(
            x=[end_time - row['Time_s']],
            y=[row['State']],
            orientation='h',
            name=row['State'],
            marker=dict(color=state_colors.get(row['State'], '#95a5a6')),
            showlegend=False,
            hovertemplate=f"{row['State']}<br>Duration: %{{x:.2f}}s<extra></extra>"
        ))
    
    fig.update_layout(
        title='<b>⚙️ Robot State Timeline</b>',
        xaxis_title='Duration (seconds)',
        yaxis_title='State',
        height=250,
        template=CHART_TEMPLATE,
        barmode='relative',
    )
    
    return fig

def create_statistics_cards(stats):
    """Return formatted statistics"""
    
    return {
        '⏱️ Total Time': f"{stats.get('total_time', 0):.2f}s",
        '📏 Distance Traveled': f"{stats.get('total_distance', 0):.2f}m",
        '🚗 Average Speed': f"{stats.get('avg_speed', 0):.2f}m/s",
        '⚠️ Min Obstacle Distance': f"{stats.get('min_obstacle_distance', 0):.2f}m",
        '👤 Worker Detections': f"{int(stats.get('worker_detections', 0))}",
        '🚨 Emergency Stops': f"{int(stats.get('emergency_stops', 0))}"
    }

def create_path_smoothing_demo():
    """Show path smoothing example (from ROS2 node)"""
    
    # Create sample waypoints
    waypoints_x = [0, 2, 5, 8, 10]
    waypoints_y = [0, 3, 2, 4, 2]
    
    # Generate smooth path using Catmull-Rom
    smooth_x = []
    smooth_y = []
    
    for i in range(len(waypoints_x) - 1):
        p0_x, p0_y = waypoints_x[max(0, i-1)], waypoints_y[max(0, i-1)]
        p1_x, p1_y = waypoints_x[i], waypoints_y[i]
        p2_x, p2_y = waypoints_x[i+1], waypoints_y[i+1]
        p3_x, p3_y = waypoints_x[min(len(waypoints_x)-1, i+2)], waypoints_y[min(len(waypoints_y)-1, i+2)]
        
        for t in np.linspace(0, 1, 50):
            t2, t3 = t*t, t*t*t
            
            # Catmull-Rom formula
            x = 0.5 * (2*p1_x + (-p0_x + p2_x)*t + (2*p0_x - 5*p1_x + 4*p2_x - p3_x)*t2 + (-p0_x + 3*p1_x - 3*p2_x + p3_x)*t3)
            y = 0.5 * (2*p1_y + (-p0_y + p2_y)*t + (2*p0_y - 5*p1_y + 4*p2_y - p3_y)*t2 + (-p0_y + 3*p1_y - 3*p2_y + p3_y)*t3)
            
            smooth_x.append(x)
            smooth_y.append(y)
    
    fig = go.Figure()
    
    # Add waypoints
    fig.add_trace(go.Scatter(
        x=waypoints_x,
        y=waypoints_y,
        mode='markers+text',
        name='Waypoints',
        marker=dict(size=12, color=COLOR_DANGER, symbol='diamond'),
        text=[f'W{i}' for i in range(len(waypoints_x))],
        textposition='top center'
    ))
    
    # Add smooth path
    fig.add_trace(go.Scatter(
        x=smooth_x,
        y=smooth_y,
        mode='lines',
        name='Smooth Path',
        line=dict(color=COLOR_PATH, width=3)
    ))
    
    fig.update_layout(
        title='<b>✨ Path Smoothing Output (Catmull-Rom Spline)</b>',
        xaxis_title='X (meters)',
        yaxis_title='Y (meters)',
        height=400,
        template=CHART_TEMPLATE,
        hovermode='closest'
    )
    
    return fig


def create_velocity_profile_demo():
    """Show ideal trapezoidal velocity profile"""
    
    # Generate trapezoidal profile
    max_vel = 0.5  # m/s
    max_accel = 0.2  # m/s^2
    
    # Calculate phases
    accel_time = max_vel / max_accel  # Time to reach max velocity
    total_distance = 30  # meters
    
    # Acceleration distance
    accel_dist = 0.5 * max_accel * accel_time**2
    
    # Deceleration is same as acceleration (symmetric)
    decel_dist = accel_dist
    
    # Cruise distance
    cruise_dist = total_distance - accel_dist - decel_dist
    cruise_time = cruise_dist / max_vel if cruise_dist > 0 else 0
    
    # Total time
    total_time = accel_time + cruise_time + accel_time
    
    # Generate velocity profile
    times = []
    velocities = []
    
    # Acceleration phase
    for t in np.linspace(0, accel_time, 50):
        times.append(t)
        velocities.append(max_accel * t)
    
    # Cruise phase
    for t in np.linspace(accel_time, accel_time + cruise_time, 50):
        times.append(t)
        velocities.append(max_vel)
    
    # Deceleration phase
    for t in np.linspace(accel_time + cruise_time, total_time, 50):
        times.append(t)
        velocities.append(max_vel - max_accel * (t - (accel_time + cruise_time)))
    
    fig = go.Figure()
    
    fig.add_trace(go.Scatter(
        x=times,
        y=velocities,
        mode='lines',
        name='Velocity',
        line=dict(color=COLOR_ROBOT, width=4),
        fill='tozeroy'
    ))
    
    # Add annotations for phases
    fig.add_vline(x=accel_time, line_dash='dash', line_color='gray', annotation_text='Accel End')
    fig.add_vline(x=accel_time + cruise_time, line_dash='dash', line_color='gray', annotation_text='Decel Start')
    
    fig.update_layout(
        title='<b>📈 Ideal Trapezoidal Velocity Profile</b>',
        xaxis_title='Time (seconds)',
        yaxis_title='Velocity (m/s)',
        height=400,
        template=CHART_TEMPLATE,
        hovermode='x unified'
    )
    
    return fig


def create_trajectory_data_visualization(data):
    """Show actual trajectory data from robot"""
    
    if data is None or len(data) == 0:
        return None
    
    # Create 3D-like visualization of trajectory with time
    times = data['Time_s'].values
    x_pos = data['Pos_X'].values
    y_pos = data['Pos_Y'].values
    
    # Calculate distance along trajectory
    distances = np.zeros(len(data))
    for i in range(1, len(data)):
        dx = x_pos[i] - x_pos[i-1]
        dy = y_pos[i] - y_pos[i-1]
        distances[i] = distances[i-1] + np.sqrt(dx**2 + dy**2)
    
    fig = go.Figure()
    
    # Main trajectory with color gradient showing time progression
    fig.add_trace(go.Scatter(
        x=x_pos,
        y=y_pos,
        mode='lines+markers',
        name='Actual Trajectory',
        line=dict(
            color=times.tolist(),
            colorscale='Viridis',
            width=3,
            colorbar=dict(title="Time (s)")
        ),
        marker=dict(size=5),
        hovertemplate='<b>Position</b><br>X: %{x:.2f}m<br>Y: %{y:.2f}m<extra></extra>'
    ))
    
    # Start point
    fig.add_trace(go.Scatter(
        x=[x_pos[0]],
        y=[y_pos[0]],
        mode='markers+text',
        name='Start',
        marker=dict(size=15, color=COLOR_SAFE, symbol='diamond'),
        text=['START'],
        textposition='top center',
        showlegend=False
    ))
    
    # End point
    fig.add_trace(go.Scatter(
        x=[x_pos[-1]],
        y=[y_pos[-1]],
        mode='markers+text',
        name='End',
        marker=dict(size=15, color=COLOR_DANGER, symbol='star'),
        text=['END'],
        textposition='top center',
        showlegend=False
    ))
    
    fig.update_layout(
        title='<b>🎯 Actual Robot Trajectory Data (Color = Time)</b>',
        xaxis_title='X Position (meters)',
        yaxis_title='Y Position (meters)',
        height=400,
        template=CHART_TEMPLATE,
        hovermode='closest'
    )
    
    return fig

def create_worker_detection_status(detection_level, data):
    """Show worker detection history and current status"""
    
    fig = go.Figure()
    
    # Current status indicator
    status_colors = {
        0: '#2ecc71',  # Green - Safe
        1: '#3498db',  # Blue - Detected far
        2: '#f39c12',  # Orange - Caution
        3: '#e74c3c'   # Red - Critical
    }
    
    status_text = {
        0: 'SAFE - No Workers',
        1: 'DETECTED - Far (>3m)',
        2: 'CAUTION - Medium (1-3m)',
        3: 'CRITICAL - Close (<1m)'
    }
    
    # Add large indicator
    fig.add_trace(go.Indicator(
        mode="gauge+number+delta",
        value=detection_level,
        domain={'x': [0, 1], 'y': [0, 1]},
        title={'text': "Worker Detection Level"},
        gauge={
            'axis': {'range': [0, 3]},
            'bar': {'color': status_colors.get(detection_level, '#95a5a6')},
            'steps': [
                {'range': [0, 1], 'color': '#2ecc71'},
                {'range': [1, 2], 'color': '#3498db'},
                {'range': [2, 3], 'color': '#f39c12'},
                {'range': [3, 3], 'color': '#e74c3c'}
            ],
            'threshold': {
                'line': {'color': 'red', 'width': 4},
                'thickness': 0.75,
                'value': 2.5
            }
        },
        number={'suffix': f" - {status_text.get(detection_level, 'UNKNOWN')}"},
    ))
    
    # Add history
    if data is not None and 'Worker_Level' in data.columns:
        fig.add_trace(go.Scatter(
            x=data['Time_s'],
            y=data['Worker_Level'],
            mode='lines+markers',
            name='Detection Level',
            line=dict(color='#9b59b6', width=2),
            marker=dict(size=4),
            yaxis='y2'
        ))
    
    fig.update_layout(
        title='<b>👤 Yellow Worker Detection Status</b>',
        height=350,
        template=CHART_TEMPLATE,
    )
    
    return fig