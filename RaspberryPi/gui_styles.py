#!/usr/bin/env python3
"""
Golf Cart GUI Styles
Custom styles and themes for the golf cart control interface

Author: Sistema UWB Carrito de Golf
Date: July 2025
"""

import tkinter as tk
from tkinter import ttk

class GolfCartStyles:
    """Custom styles for the Golf Cart GUI"""
    
    def __init__(self, root):
        self.root = root
        self.setup_styles()
    
    def setup_styles(self):
        """Setup custom styles for ttk widgets"""
        style = ttk.Style()
        
        # Configure overall theme
        style.theme_use('clam')
        
        # Custom progressbar styles for battery indicator
        style.configure(
            'green.Horizontal.TProgressbar',
            background='#27ae60',
            borderwidth=1,
            lightcolor='#2ecc71',
            darkcolor='#229954'
        )
        
        style.configure(
            'yellow.Horizontal.TProgressbar',
            background='#f1c40f',
            borderwidth=1,
            lightcolor='#f4d03f',
            darkcolor='#d4ac0d'
        )
        
        style.configure(
            'red.Horizontal.TProgressbar',
            background='#e74c3c',
            borderwidth=1,
            lightcolor='#ec7063',
            darkcolor='#c0392b'
        )
        
        # Custom frame styles
        style.configure(
            'Card.TLabelFrame',
            background='#ecf0f1',
            borderwidth=2,
            relief='raised'
        )
        
        style.configure(
            'Card.TLabelFrame.Label',
            background='#ecf0f1',
            font=('Arial', 10, 'bold')
        )
        
        # Custom button styles would go here if using ttk buttons
        # But we're using tk buttons for better color control

def create_custom_button(parent, text, command, bg_color, width=10, height=2):
    """Create a custom styled button"""
    return tk.Button(
        parent,
        text=text,
        command=command,
        bg=bg_color,
        fg='white',
        font=('Arial', 10, 'bold'),
        width=width,
        height=height,
        relief='raised',
        borderwidth=2,
        activebackground=_darken_color(bg_color),
        activeforeground='white'
    )

def _darken_color(hex_color):
    """Darken a hex color by 20% for active state"""
    # Simple darkening - remove # and convert to RGB
    hex_color = hex_color.lstrip('#')
    rgb = tuple(int(hex_color[i:i+2], 16) for i in (0, 2, 4))
    # Darken each component
    darkened = tuple(max(0, int(c * 0.8)) for c in rgb)
    # Convert back to hex
    return f"#{darkened[0]:02x}{darkened[1]:02x}{darkened[2]:02x}"

# Color palette for the golf cart interface
COLORS = {
    'primary': '#3498db',      # Blue
    'secondary': '#2ecc71',    # Green
    'danger': '#e74c3c',       # Red
    'warning': '#f39c12',      # Orange
    'info': '#9b59b6',         # Purple
    'dark': '#2c3e50',         # Dark blue
    'light': '#ecf0f1',        # Light gray
    'success': '#27ae60'       # Dark green
}
