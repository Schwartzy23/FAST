import pandas as pd
import matplotlib.pyplot as plt
import numpy as np
from pathlib import Path

# Constants
DIELECTRIC_CONSTANT = 8.854e-12 # Dielectric constant of air (F/m)
POLYMIDE_DIELECTRIC_CONSTANT = 3.2 # Dielectric constant of polymide (F/m)
CU_SENSOR_WIDTH = 0.0111 # Sensor Width (m)
CU_SENSOR_LENGTH = 0.0156 # Sensor Length (m)
YOUNG_MODULUS = 200e9  # Young's modulus for steel (Pa)
INITIAL_CAPACITANCE = 1.92e-7  # Initial capacitance (F)
CU_SENSOR_AREA = CU_SENSOR_WIDTH * CU_SENSOR_LENGTH  # Sensor area (m²)


# Capacitor Sensor
CAP_SENSOR_THICKNESS =   DIELECTRIC_CONSTANT * CU_SENSOR_AREA * POLYMIDE_DIELECTRIC_CONSTANT / INITIAL_CAPACITANCE # The original thickness of the dielectric 

print(CAP_SENSOR_THICKNESS)

def calculate_strain(capacitance):
    """
    Calculate strain from capacitance values
    Strain = (C - C₀) / C₀
    where C is current capacitance and C₀ is initial capacitance
    """
    return (capacitance - INITIAL_CAPACITANCE) / INITIAL_CAPACITANCE

def calculate_force(strain):
    """
    Calculate force using Young's modulus
    F = E * A * ε
    where:
    E = Young's modulus
    A = Cross-sectional area
    ε = Strain
    """
    return YOUNG_MODULUS * CU_SENSOR_AREA * strain

def load_capacitance_data(file_path):
    """
    Load capacitance data from a CSV file.
    Expected CSV format: timestamp,capacitance_value
    """
    try:
        df = pd.read_csv(file_path)
        # Calculate strain and force
        df['strain'] = calculate_strain(df['capacitance_value'])
        df['force'] = calculate_force(df['strain'])
        return df
    except Exception as e:
        print(f"Error loading file: {e}")
        return None



print(calculate_force)