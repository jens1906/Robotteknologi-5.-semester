import os
import sys
from typing import Dict, Tuple

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
        # Use direct lookups to raise KeyError when a key is missing
        w = weights[key]
        r = ratings[key]
        p = w * r
        products[key] = p
        total_w += w
        total_wr += p
    if total_w != 15.0:
        raise ZeroDivisionError("Sum of weights is not 15; cannot compute workload.")
    return total_wr / total_w, products


def export_csv(csv_path: str, weights_path: str, ratings_path: str, weights: Dict[str, float], ratings: Dict[str, float], products: Dict[str, float], overall: float) -> None:
    """Write results to a CSV file with headers: Dimension,Weight,Rating,Product.
    Includes a final summary row with Overall_Workload.
    """
    import csv

    with open(csv_path, "w", newline="", encoding="utf-8") as f:
        writer = csv.writer(f)
        # metadata rows
        writer.writerow(["weights", weights_path])
        writer.writerow(["ratings", ratings_path])
        writer.writerow([])
        # header
        writer.writerow(["Dimension", "Weight", "Rating", "Product"])
        for key in DIMENSION_KEYS:
            label = key.replace("_", " ").title()
            writer.writerow([label, weights[key], ratings[key], products[key]])
        writer.writerow([])
        writer.writerow(["Overall_Workload", overall])


def export_csv_combined(csv_path: str, session: str, weights: Dict[str, float], ratings: Dict[str, float], products: Dict[str, float], overall: float, write_header: bool = False) -> None:
    """Append results to a single CSV with a Session column.
    Header: Session,Dimension,Weight,Rating,Product,Overall_Workload
    """
    import csv

    with open(csv_path, "a", newline="", encoding="utf-8") as f:
        writer = csv.writer(f)
        if write_header:
            writer.writerow(["Session", "Dimension", "Weight", "Rating", "Product", "Overall_Workload"])
        for key in DIMENSION_KEYS:
            label = key.replace("_", " ").title()
            writer.writerow([session, label, weights[key], ratings[key], products[key], ""])
        # Final summary row for the session
        writer.writerow([session, "Overall_Workload", "", "", "", overall])


def main(w: str, r: str):
    # Default paths relative to this script directory
    base_dir = os.path.dirname(__file__)
    weights_path = os.path.join(base_dir, w)
    ratings_path = os.path.join(base_dir, r)

    # Paths are provided by caller; CLI is handled in __main__

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
        print(f"{to_label(key)}: weight={weights[key]} rating={ratings[key]} product={products[key]}")
    print("")
    print(f"Overall_Workload = sum(weights*ratings)/sum(weights) = {overall}")
    # Always export a CSV next to the weights/ratings files.
    # Derive session suffix from weights filename if possible.
    weights_name = os.path.basename(weights_path)
    session = None
    if weights_name.startswith("tlx_weights_") and weights_name.endswith(".txt"):
        session = weights_name[len("tlx_weights_"):-4]
    csv_name = f"tlx_results_{session}.csv" if session else "tlx_results.csv"
    csv_path = os.path.join(os.path.dirname(weights_path), csv_name)
    export_csv(csv_path, weights_path, ratings_path, weights, ratings, products, overall)
    print(f"CSV exported: {csv_path}")
    # Optional CSV export via environment variable or CLI flag handled in __main__


if __name__ == "__main__":
    base_dir = os.path.dirname(__file__)
    args = sys.argv[1:]

    # Optional combined CSV export: always generates a single file when multiple sessions run
    combined_csv = None
    if "--csv" in args:
        i = args.index("--csv")
        if i + 1 < len(args):
            combined_csv = args[i + 1]
            args = args[:i] + args[i+2:]

    def run_and_export(weights_path: str, ratings_path: str, session_label: str | None = None):
        weights = parse_kv_file(weights_path)
        ratings = parse_kv_file(ratings_path)
        overall, products = compute_overall_workload(weights, ratings)
        # Print report
        print("TLX Overall Workload")
        print(f"weights: {weights_path}")
        print(f"ratings: {ratings_path}")
        print("")
        to_label = lambda k: k.replace("_", " ").title()
        for key in DIMENSION_KEYS:
            print(f"{to_label(key)}: weight={weights[key]} rating={ratings[key]} product={products[key]}")
        print("")
        print(f"Overall_Workload = sum(weights*ratings)/sum(weights) = {overall}")

        # CSV behavior:
        # - If combined_csv is provided and a session_label is set, append to the combined file
        # - Else export a per-run CSV next to input files
        if combined_csv and session_label:
            write_header = not os.path.exists(combined_csv)
            export_csv_combined(combined_csv, session_label, weights, ratings, products, overall, write_header)
            print(f"Appended to combined CSV: {combined_csv} (session {session_label})")
        else:
            weights_name = os.path.basename(weights_path)
            session = None
            if weights_name.startswith("tlx_weights_") and weights_name.endswith(".txt"):
                session = weights_name[len("tlx_weights_"):-4]
            csv_name = f"tlx_results_{session}.csv" if session else "tlx_results.csv"
            csv_path = os.path.join(os.path.dirname(weights_path), csv_name)
            export_csv(csv_path, weights_path, ratings_path, weights, ratings, products, overall)
            print(f"CSV exported: {csv_path}")

    # Mode 1: explicit file paths
    if len(args) >= 2 and not args[0].startswith("--"):
        run_and_export(args[0], args[1])
        sys.exit(0)

    # Mode 2: one or more sessions provided
    if args and args[0] in {"--session", "--sessions"}:
        sessions = args[1:]
        if not sessions:
            print("No sessions provided after --session/--sessions", file=sys.stderr)
            sys.exit(2)
        for s in sessions:
            run_and_export(os.path.join(base_dir, f"tlx_weights_{s}.txt"), os.path.join(base_dir, f"tlx_ratings_{s}.txt"), s)
        sys.exit(0)

    # Mode 3: autodiscover any matching s* pairs in this folder
    names = set(os.listdir(base_dir))
    weights_sessions = {name[len("tlx_weights_"):-4] for name in names if name.startswith("tlx_weights_") and name.endswith(".txt")}
    ratings_sessions = {name[len("tlx_ratings_"):-4] for name in names if name.startswith("tlx_ratings_") and name.endswith(".txt")}
    sessions = sorted(weights_sessions & ratings_sessions)
    if not sessions:
        sessions = ["s1", "s2", "s3", "s4"]
    # If multiple sessions, default to combined CSV in the same folder
    if len(sessions) > 1 and combined_csv is None:
        combined_csv = os.path.join(base_dir, "tlx_results_all.csv")
    for s in sessions:
        run_and_export(os.path.join(base_dir, f"tlx_weights_{s}.txt"), os.path.join(base_dir, f"tlx_ratings_{s}.txt"), s)