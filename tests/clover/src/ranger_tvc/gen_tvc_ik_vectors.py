"""Generate tvc_ik_vectors.h, the inverse-kinematics test vectors for the Ranger TVC unit tests.

Re-run from the arty directory whenever the CSVs in data/ or the gimbal geometry change:

    python3 tests/clover/src/ranger_tvc/gen_tvc_ik_vectors.py

Sources:
  - Pitch: data/TVCpitchnew.csv, exported by data/inversekinematicsactuatorsnewgimbal.m.
  - Yaw: computed here from the same MATLAB formula with psi0 = 113.696 deg, in double precision. The supplied
    data/TVCyawnew.csv is NOT used: it is identical to the pitch CSV, i.e. it was generated with 110.155 deg.
    If psi0 turns out to be 110.155, change YAW_THETA0_DEG here and TVC_PSI0_DEG / TVC_YAW.l_center_in in
    clover/src/ranger/tvc/tvc_config.h, then re-run.
"""

import csv
import math
from pathlib import Path

HERE = Path(__file__).resolve().parent

# Geometry from inversekinematicsactuatorsnewgimbal.m
X_IN = 11.2120
TE_IN = 8.3981
E_IN = 6.6941
ER_IN = 4.349
YAW_THETA0_DEG = 113.696


def matlab_length_in(angle_deg: float, theta0_deg: float) -> float:
    lts = math.sqrt(X_IN**2 + TE_IN**2)
    lre = math.sqrt(E_IN**2 + ER_IN**2)
    return math.sqrt(lts**2 + lre**2 - 2 * lts * lre * math.cos(math.radians(theta0_deg + angle_deg)))


def read_csv(path: Path) -> list[tuple[float, float]]:
    with path.open(newline="") as f:
        reader = csv.reader(f)
        next(reader)  # header
        return [(float(a), float(length)) for a, length in reader]


def float_literal(value: float) -> str:
    text = f"{value:.15g}"
    if "." not in text and "e" not in text:
        text += ".0"
    return text + "f"


def emit_array(name: str, rows: list[tuple[float, float]]) -> str:
    body = ",\n".join(f"    {{{float_literal(a)}, {float_literal(length)}}}" for a, length in rows)
    return f"constexpr TvcIkVector {name}[] = {{\n{body},\n}};\n"


def main() -> None:
    pitch = read_csv(HERE / "data" / "TVCpitchnew.csv")
    yaw = [(a, matlab_length_in(a, YAW_THETA0_DEG)) for a, _ in pitch]

    out = [
        "#pragma once\n",
        "// >>> GENERATED FILE <<< by tests/clover/src/ranger_tvc/gen_tvc_ik_vectors.py -- do not edit by hand.\n",
        "// Pitch: data/TVCpitchnew.csv. Yaw: MATLAB formula with psi0 = %.3f deg (see generator docstring).\n\n"
        % YAW_THETA0_DEG,
        "struct TvcIkVector {\n    float angle_deg;\n    float length_in;\n};\n\n",
        emit_array("TVC_PITCH_IK_VECTORS", pitch),
        "\n",
        emit_array("TVC_YAW_IK_VECTORS", yaw),
    ]
    (HERE / "tvc_ik_vectors.h").write_text("".join(out))


if __name__ == "__main__":
    main()
