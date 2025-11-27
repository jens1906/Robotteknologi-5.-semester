import os
import sys
from typing import Dict, Tuple, Iterable

DIMENSION_KEYS = [
    "mental_demand",
    "physical_demand",
    "temporal_demand",
    "performance",
    "effort",
    "frustration",
]

LABEL_ALIASES = {
    # Map human labels to standard keys
    "mental demand": "mental_demand",
    "mental demands": "mental_demand",
    "physical demand": "physical_demand",
    "physical demands": "physical_demand",
    "temporal demand": "temporal_demand",
    "temporal demands": "temporal_demand",
    "performance": "performance",
    "effort": "effort",
    "frustration": "frustration",
}

# Precompute lowercase canonical keys set for quick membership tests
CANONICAL_KEYS = set(DIMENSION_KEYS)


def _to_number(token: str) -> float | None:
    """Convert leading token to float; returns None if not possible."""
    try:
        return float(token)
    except ValueError:
        parts = token.split()
        if parts:
            try:
                return float(parts[0])
            except ValueError:
                return None
        return None


def parse_kv_file(path: str) -> Dict[str, float]:
    """Parse lines like 'Label: number' into a dict of floats.
    - Robust to trailing annotations (e.g., '5 (33%)') by taking the leading number.
    - Normalizes labels via aliases and underscores, all lowercase.
    """
    if not os.path.exists(path):
        raise FileNotFoundError(path)

    result: Dict[str, float] = {}
    with open(path, "r", encoding="utf-8") as f:
        for raw in f:
            line = raw.strip()
            if not line or ":" not in line:
                continue
            label, value = line.split(":", 1)
            key = LABEL_ALIASES.get(label.strip().lower(), label.strip().lower().replace(" ", "_"))
            num = _to_number(value.strip())
            if num is None:
                continue
            result[key] = num
    return result


def compute_overall_workload(weights: Dict[str, float], ratings: Dict[str, float]) -> Tuple[float, Dict[str, float]]:
    """Compute sum(weights*ratings)/sum(weights) using canonical TLX keys."""
    # Fast path using generator expressions
    products: Dict[str, float] = {}
    total_w = 0.0
    total_wr = 0.0
    for key in DIMENSION_KEYS:
        w = weights.get(key, 0.0)
        r = ratings.get(key, 0.0)
        p = w * r
        products[key] = p
        total_w += w
        total_wr += p
    if total_w == 0.0:
        raise ZeroDivisionError("Sum of weights is zero; cannot compute workload.")
    return total_wr / total_w, products


def main():
    # Default paths relative to this script directory
    base_dir = os.path.dirname(__file__)
    weights_path = os.path.join(base_dir, "tlx_weights.txt")
    ratings_path = os.path.join(base_dir, "tlx_ratings.txt")

    # Allow optional CLI args: weights_path ratings_path
    if len(sys.argv) >= 2:
        weights_path = sys.argv[1]
    if len(sys.argv) >= 3:
        ratings_path = sys.argv[2]

    weights = parse_kv_file(weights_path)
    ratings = parse_kv_file(ratings_path)

    overall, products = compute_overall_workload(weights, ratings)

    # Print a concise report
    print("TLX Overall Workload")
    print(f"weights: {weights_path}")
    print(f"ratings: {ratings_path}")
    print("")
    to_label = lambda k: k.replace("_", " ").title()
    for key in DIMENSION_KEYS:
        print(f"{to_label(key)}: weight={weights.get(key, 0.0)} rating={ratings.get(key, 0.0)} product={products.get(key, 0.0)}")
    print("")
    print(f"Overall_Workload = sum(weights*ratings)/sum(weights) = {overall}")


if __name__ == "__main__":
    main()
