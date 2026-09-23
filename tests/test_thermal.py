import pytest
from pathlib import Path
from tools.thermal_sim import secure_path_resolve, PhysicsParams

def test_path_traversal_blocked():
    """Verify CWE-22 protection."""
    base_dir = Path(__file__).resolve().parent.parent
    malicious_path = "../../../etc/passwd"
    
    with pytest.raises(PermissionError):
        secure_path_resolve(malicious_path, base_dir)

def test_safe_path_allowed():
    """Verify valid outputs are permitted."""
    base_dir = Path(__file__).resolve().parent.parent
    valid_path = "docs/test_plot.png"
    
    resolved = secure_path_resolve(valid_path, base_dir)
    assert str(resolved).startswith(str(base_dir))

def test_physics_immutability():
    """Verify parameters cannot be accidentally mutated."""
    p = PhysicsParams()
    with pytest.raises(AttributeError): # Frozen dataclasses raise AttributeError on reassignment
        p.laser_power_w = 9000.0