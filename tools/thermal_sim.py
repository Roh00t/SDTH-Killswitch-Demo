import argparse
import logging
from pathlib import Path
import matplotlib.pyplot as plt
import numpy as np
from dataclasses import dataclass

logging.basicConfig(level=logging.INFO, format="%(levelname)s: %(message)s")

@dataclass(frozen=True)
class PhysicsParams:
    # Laser & Engagement Parameters
    laser_power_w: float = 5000.0          # 5 kW source power
    engagement_range_m: float = 350.0       # 350 m tactical engagement distance
    atm_attenuation_coeff: float = 0.0008  # Beer-Lambert gamma at 1.55 um (1/m)
    boresight_jitter_deg: float = 0.282     # Boresight alignment error (deg)
    optical_absorption: float = 0.85       # Polycarbonate absorption at 1.55 um

    # Material Properties (Polycarbonate / Nylon 6,6 Rotor Junction)
    density_kg_m3: float = 1200.0           # kg/m^3
    specific_heat_j_kg_k: float = 1200.0    # J/(kg*K)
    thermal_cond_w_m_k: float = 0.2         # W/(m*K)
    temp_init_c: float = 22.0              # Initial ambient temp (°C)
    temp_melt_c: float = 260.0             # Melting threshold (°C)
    latent_heat_fusion_j_kg: float = 130e3  # Enthalpy of fusion (J/kg)
    melt_depth_m: float = 0.001            # 1 mm critical load thickness

def secure_path_resolve(output_arg: str, base_dir: Path) -> Path:
    """Security Posture: Prevent Path Traversal (CWE-22)"""
    target_path = Path(output_arg).resolve()
    if not str(target_path).startswith(str(base_dir.resolve())):
        raise PermissionError(f"Security exception: Cannot write outside repository path: {target_path}")
    return target_path

def run_thermal_simulation(output_plot_path: str = "docs/thermal_kill_chain.png") -> None:
    repo_root = Path(__file__).resolve().parent.parent
    
    try:
        safe_out_path = secure_path_resolve(output_plot_path, repo_root)
    except PermissionError as e:
        logging.error(e)
        return

    p = PhysicsParams()

    # 1. Beer-Lambert Atmospheric Attenuation
    p_target_transmitted = p.laser_power_w * np.exp(-p.atm_attenuation_coeff * p.engagement_range_m)
    
    # 2. Boresight Jitter Beam Spread (Effective spot radius on target at range)
    jitter_rad = np.radians(p.boresight_jitter_deg)
    spot_radius_m = max(0.005, p.engagement_range_m * np.sin(jitter_rad / 2.0) * 0.16)
    
    # 3. Peak Surface Heat Flux (Gaussian Peak)
    area = np.pi * (spot_radius_m ** 2)
    p_absorbed = p_target_transmitted * p.optical_absorption
    q_surface = (2.0 * p_absorbed) / area  # W/m^2

    # 4. Thermal Diffusivity alpha = k / (rho * cp)
    alpha = p.thermal_cond_w_m_k / (p.density_kg_m3 * p.specific_heat_j_kg_k)

    # 5. Time Discretization
    dt = 0.002
    t_final = 2.0
    t = np.arange(0, t_final, dt)
    temp = np.zeros_like(t)
    temp[0] = p.temp_init_c

    melt_onset_time = None
    latent_heat_energy = p.density_kg_m3 * p.melt_depth_m * p.latent_heat_fusion_j_kg
    absorbed_latent_energy = 0.0

    # 6. 1D Heat Conduction Transient with Phase Change Enthalpy
    for i in range(1, len(t)):
        time_curr = t[i]
        
        if melt_onset_time is None:
            # Semi-infinite 1D thermal diffusion surface equation
            t_rise = p.temp_init_c + ((2.0 * q_surface / p.thermal_cond_w_m_k) * np.sqrt((alpha * time_curr) / np.pi))
            
            if t_rise >= p.temp_melt_c:
                temp[i] = p.temp_melt_c
                melt_onset_time = time_curr
            else:
                temp[i] = t_rise
        else:
            # Phase change enthalpy plateau at Tmelt = 260°C
            if absorbed_latent_energy < latent_heat_energy:
                absorbed_latent_energy += q_surface * dt
                temp[i] = p.temp_melt_c
            else:
                # Post-melt superheating
                time_post = time_curr - (melt_onset_time + (latent_heat_energy / q_surface))
                temp[i] = p.temp_melt_c + ((2.0 * q_surface / p.thermal_cond_w_m_k) * np.sqrt((alpha * max(0.001, time_post)) / np.pi))

    logging.info(f"Effective Spot Radius at {p.engagement_range_m}m: {spot_radius_m*100:.2f} cm")
    logging.info(f"Surface Heat Flux: {q_surface/1e4:.2f} W/cm^2")
    if melt_onset_time:
        logging.info(f"Time to Onset of Melting (Tmelt = {p.temp_melt_c}°C): {melt_onset_time:.3f} s")

    safe_out_path.parent.mkdir(parents=True, exist_ok=True)

    # 7. Render Pitch Deck Plot
    plt.style.use('seaborn-v0_8-darkgrid' if 'seaborn-v0_8-darkgrid' in plt.style.available else 'default')
    fig, ax = plt.subplots(figsize=(9, 5), dpi=300)

    ax.plot(t, temp, color='#D32F2F', linewidth=2.5, label='Spot Center Temp (°C)')
    ax.axhline(y=p.temp_melt_c, color='#333333', linestyle='--', linewidth=1.5, label=f'Melt Threshold ({p.temp_melt_c}°C)')
    
    if melt_onset_time:
        ax.axvline(x=melt_onset_time, color='#FF9800', linestyle=':', linewidth=1.8, label=f'Melt Onset ({melt_onset_time:.2f}s)')
        ax.scatter([melt_onset_time], [p.temp_melt_c], color='#D32F2F', s=90, zorder=5)

    ax.set_title("C-UAS Target Kill Chain: 5 kW Laser Thermal Transient on Rotor Junction", fontsize=12, fontweight='bold')
    ax.set_xlabel("Dwell Time (seconds)", fontsize=10, fontweight='bold')
    ax.set_ylabel("Material Temperature (°C)", fontsize=10, fontweight='bold')
    ax.set_xlim(0, t_final)
    ax.set_ylim(0, 500)
    ax.legend(loc='upper left', frameon=True, facecolor='white', framealpha=0.9)
    plt.tight_layout()

    fig.savefig(str(safe_out_path))
    logging.info(f"Plot safely written to {safe_out_path}")

if __name__ == "__main__":
    parser = argparse.ArgumentParser(description="Killswitch Thermal Simulator")
    parser.add_argument("--output", type=str, default="docs/thermal_kill_chain.png", help="Path to save output plot")
    args = parser.parse_args()
    run_thermal_simulation(args.output)