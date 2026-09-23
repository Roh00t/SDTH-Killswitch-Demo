import argparse
import logging
from pathlib import Path
import matplotlib.pyplot as plt
import numpy as np
from dataclasses import dataclass

# 1. AI Security Posture: Strict Logging over Print Statements
logging.basicConfig(level=logging.INFO, format="%(levelname)s: %(message)s")

# 2. AI Security Posture: Immutable Physical Constants
@dataclass(frozen=True)
class PhysicsParams:
    laser_power_w: float = 5000.0
    spot_radius_m: float = 0.005
    absorption_coeff: float = 0.85
    density_kg_m3: float = 1200.0
    specific_heat_j_kg_k: float = 1200.0
    thermal_cond_w_m_k: float = 0.2
    temp_init_c: float = 22.0
    temp_melt_c: float = 260.0
    latent_heat_fusion_j_kg: float = 130e3
    critical_depth_m: float = 0.001

def secure_path_resolve(output_arg: str, base_dir: Path) -> Path:
    """Security Posture: Prevent Path Traversal (CWE-22)"""
    target_path = Path(output_arg).resolve()
    if not str(target_path).startswith(str(base_dir.resolve())):
        raise PermissionError(f"Security exception: Cannot write outside repository. Prevented traversal to {target_path}")
    return target_path

def run_thermal_simulation(output_plot_path: str = "docs/thermal_kill_chain.png") -> None:
    repo_root = Path(__file__).resolve().parent.parent
    
    try:
        safe_out_path = secure_path_resolve(output_plot_path, repo_root)
    except PermissionError as e:
        logging.error(e)
        return

    p = PhysicsParams()
    
    area = np.pi * (p.spot_radius_m ** 2)
    p_absorbed = p.laser_power_w * p.absorption_coeff
    mass = p.density_kg_m3 * area * p.critical_depth_m

    q_to_melt = mass * p.specific_heat_j_kg_k * (p.temp_melt_c - p.temp_init_c)
    q_phase_change = mass * p.latent_heat_fusion_j_kg
    q_total_required = q_to_melt + q_phase_change

    # 3. Security Posture: Bound Resource Allocation (CWE-400)
    dt = 0.001
    t_final = 2.0
    if (t_final / dt) > 100_000:
        raise ValueError("Simulation resolution too high; risk of Memory OOM.")

    t = np.arange(0, t_final, dt)
    temp = np.zeros_like(t)
    temp[0] = p.temp_init_c

    energy_accumulated = 0.0
    melt_time = None

    for i in range(1, len(t)):
        energy_accumulated += p_absorbed * dt
        if energy_accumulated < q_to_melt:
            temp[i] = p.temp_init_c + (energy_accumulated / (mass * p.specific_heat_j_kg_k))
        elif energy_accumulated < q_total_required:
            temp[i] = p.temp_melt_c 
            if melt_time is None:
                melt_time = t[i]
        else:
            temp[i] = p.temp_melt_c + ((energy_accumulated - q_total_required) / (mass * p.specific_heat_j_kg_k))

    logging.info(f"Absorbed Power: {p_absorbed:.1f} W")
    logging.info(f"Energy to Reach Tmelt: {q_to_melt:.2f} J")
    logging.info(f"Phase Change Energy: {q_phase_change:.2f} J")
    if melt_time:
        logging.info(f"Time to Initiate Melting: {melt_time:.3f} s")
    
    safe_out_path.parent.mkdir(parents=True, exist_ok=True)
    
    # Render Plot
    plt.style.use('seaborn-v0_8-darkgrid' if 'seaborn-v0_8-darkgrid' in plt.style.available else 'default')
    fig, ax = plt.subplots(figsize=(8, 4.5), dpi=300)
    ax.plot(t, temp, color='#D32F2F', linewidth=2.5, label='Spot Center Temp (°C)')
    ax.axhline(y=p.temp_melt_c, color='#333333', linestyle='--', linewidth=1.5, label=f'Melt Threshold ({p.temp_melt_c}°C)')
    
    if melt_time:
        ax.axvline(x=melt_time, color='#FF9800', linestyle=':', linewidth=1.5, label=f'Melt Onset ({melt_time:.2f}s)')
        ax.scatter([melt_time], [p.temp_melt_c], color='#D32F2F', s=80, zorder=5)

    ax.set_title("C-UAS Target Kill Chain: 5 kW Laser Thermal Transient on Rotor Junction", fontsize=12, fontweight='bold')
    ax.set_xlabel("Dwell Time (seconds)", fontsize=10)
    ax.set_ylabel("Material Temperature (°C)", fontsize=10)
    ax.set_xlim(0, t_final)
    ax.set_ylim(0, 500)
    ax.legend(loc='upper left', frameon=True)
    plt.tight_layout()

    fig.savefig(str(safe_out_path))
    logging.info(f"Plot safely written to {safe_out_path}")

if __name__ == "__main__":
    parser = argparse.ArgumentParser(description="Killswitch Thermal Simulator")
    parser.add_argument("--output", type=str, default="docs/thermal_kill_chain.png", help="Path to save output plot")
    args = parser.parse_args()
    run_thermal_simulation(args.output)