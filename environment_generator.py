import argparse
import random
import math
from pathlib import Path

# ---------- tiny parser helpers ----------
def _decomment(s: str) -> str:
    return s.split("#", 1)[0].strip()

def parse_env(path: Path):
    cfg = {
        "START_X": 0.0, "START_Y": 0.0, "START_HEADING_DEG": 0.0,
        "GOAL_X": 500.0, "GOAL_Y": 0.0, "GOAL_RADIUS_CM": 10.0,
        "ROBOT_SHAPE": "rect",
        "ROBOT_WIDTH_CM": 20.0, "ROBOT_HEIGHT_CM": 20.0,
        "ROBOT_RADIUS_CM": 0.0,
        "ROBOT_POLY_FACES": 0, "ROBOT_POLY_RADIUS_CM": 0.0,
        "CLEARANCE_CM": 5.0,
        "OBSTACLES": []
    }
    if not path.exists():
        return cfg
    for raw in path.read_text(encoding="utf-8").splitlines():
        s = _decomment(raw)
        if not s:
            continue
        if s.upper().startswith("OBSTACLE="):
            # ignored here; we regenerate them
            continue
        if "=" not in s:
            continue
        k, v = s.split("=", 1)
        k = k.strip().upper()
        v = v.strip()
        try:
            if k in {"START_X","START_Y","START_HEADING_DEG","GOAL_X","GOAL_Y","GOAL_RADIUS_CM",
                     "ROBOT_WIDTH_CM","ROBOT_HEIGHT_CM","ROBOT_RADIUS_CM","ROBOT_POLY_RADIUS_CM",
                     "CLEARANCE_CM"}:
                cfg[k] = float(v)
            elif k in {"ROBOT_POLY_FACES"}:
                cfg[k] = int(v)
            elif k in {"ROBOT_SHAPE"}:
                cfg[k] = v
        except:
            pass
    return cfg

def _get(cfg, key, default):
    return cfg[key] if key in cfg else default

def robot_inflation_radius(cfg):
    shape = _get(cfg, "ROBOT_SHAPE", "rect").lower()
    clearance = float(_get(cfg, "CLEARANCE_CM", 5.0))
    if shape == "circle":
        rr = float(_get(cfg, "ROBOT_RADIUS_CM", 0.0))
        return rr + clearance
    elif shape == "rect":
        w = float(_get(cfg, "ROBOT_WIDTH_CM", 20.0))
        h = float(_get(cfg, "ROBOT_HEIGHT_CM", 20.0))
        return 0.5 * math.hypot(w, h) + clearance
    elif shape == "polygon":
        faces = int(_get(cfg, "ROBOT_POLY_FACES", 0))
        pr = float(_get(cfg, "ROBOT_POLY_RADIUS_CM", 0.0))
        if faces >= 3 and pr > 0:
            return pr + clearance
        w = float(_get(cfg, "ROBOT_WIDTH_CM", 20.0))
        h = float(_get(cfg, "ROBOT_HEIGHT_CM", 20.0))
        return 0.5 * math.hypot(w, h) + clearance
    else:
        w = float(_get(cfg, "ROBOT_WIDTH_CM", 20.0))
        h = float(_get(cfg, "ROBOT_HEIGHT_CM", 20.0))
        return 0.5 * math.hypot(w, h) + clearance

# ---------- geometry ----------
def dist(a, b):
    return math.hypot(a[0]-b[0], a[1]-b[1])

def circle_bbox(cx, cy, r):
    return (cx-r, cy-r, cx+r, cy+r)

def intersects_start_goal(cx, cy, r, start, goal, keepout):
    # forbid centers whose inflated disc (r) touches the keepout discs at start/goal (radius = keepout)
    return (dist((cx,cy), start) <= r + keepout) or (dist((cx,cy), goal) <= r + keepout)

def overlaps_existing(cx, cy, r, placed, pad=0.0):
    for (px, py, pr) in placed:
        if dist((cx,cy), (px,py)) < (r + pr + pad):
            return True
    return False

# ---------- IO ----------
def replace_obstacles(env_path: Path, obstacles_lines, output_path: Path|None):
    src_lines = env_path.read_text(encoding="utf-8").splitlines(keepends=False) if env_path.exists() else []
    def is_obstacle_line(raw: str) -> bool:
        s = _decomment(raw)
        return s.upper().startswith("OBSTACLE=")

    kept = [line for line in src_lines if not is_obstacle_line(line)]
    first_idx = next((i for i, line in enumerate(src_lines) if is_obstacle_line(line)), None)

    new_block = []
    if obstacles_lines:
        new_block.append("")
        new_block.append("# --- Obstacles (auto-generated) ---")
        new_block.extend(obstacles_lines)

    if first_idx is None:
        out = kept + new_block
    else:
        kept_indices = [i for i, line in enumerate(src_lines) if not is_obstacle_line(line)]
        insertion_pos = 0
        for pos, orig_idx in enumerate(kept_indices):
            if orig_idx >= first_idx:
                insertion_pos = pos
                break
        else:
            insertion_pos = len(kept)
        out = kept[:insertion_pos] + new_block + kept[insertion_pos:]

    out_text = "\n".join(out).rstrip() + "\n"

    if output_path is None:
        bak = env_path.with_suffix(env_path.suffix + ".bak")
        bak.write_text("\n".join(src_lines) + ("\n" if (src_lines and not src_lines[-1].endswith("\n")) else ""), encoding="utf-8")
        env_path.write_text(out_text, encoding="utf-8")
        return env_path, bak
    else:
        output_path.write_text(out_text, encoding="utf-8")
        return output_path, None

# ---------- main randomizer ----------
def main():
    ap = argparse.ArgumentParser(description="Randomize obstacles in environment.txt")
    ap.add_argument("--env", default="environment.txt")
    ap.add_argument("--out", default=None, help="Write to another env file; default: in-place (with .bak)")
    ap.add_argument("--seed", type=int, default=None)
    ap.add_argument("--count-min", type=int, default=1)
    ap.add_argument("--count-max", type=int, default=5)
    ap.add_argument("--circle-prob", type=float, default=0.5, help="Probability an obstacle is a circle (else polygon)")
    ap.add_argument("--faces-min", type=int, default=3)
    ap.add_argument("--faces-max", type=int, default=8)
    ap.add_argument("--radius-min", type=float, default=30.0)
    ap.add_argument("--radius-max", type=float, default=120.0)
    ap.add_argument("--rotation", action="store_true", help="Allow random rotation for polygons")
    ap.add_argument("--nonoverlap", action="store_true", help="Try to avoid overlaps between obstacles")
    ap.add_argument("--tries-per-obstacle", type=int, default=200)
    ap.add_argument("--pad", type=float, default=50.0, help="Pad (cm) around the start/goal bounding box for placement")
    ap.add_argument("--keepout-scale", type=float, default=1.5, help="Start/goal keepout multiplier on robot inflation")
    args = ap.parse_args()

    if args.seed is not None:
        random.seed(args.seed)

    env_path = Path(args.env)
    out_path = Path(args.out) if args.out else None
    cfg = parse_env(env_path)

    # Placement area: bounding box around start & goal, padded
    sx, sy = float(cfg["START_X"]), float(cfg["START_Y"])
    gx, gy = float(cfg["GOAL_X"]), float(cfg["GOAL_Y"])
    minx = min(sx, gx) - args.pad
    maxx = max(sx, gx) + args.pad
    miny = min(sy, gy) - args.pad
    maxy = max(sy, gy) + args.pad

    # Robot inflation impacts keepout around start/goal
    inflate = robot_inflation_radius(cfg)
    keepout = inflate * args.keepout_scale

    # How many obstacles
    n_obs = random.randint(args.count_min, args.count_max)

    placed_for_overlap = []  # (cx,cy,eff_radius_for_overlap)

    obstacles_lines = []
    for _ in range(n_obs):
        is_circle = (random.random() < args.circle_prob)

        faces = 0
        rot = 0.0
        r = random.uniform(args.radius_min, args.radius_max)
        if not is_circle:
            faces = random.randint(max(3, args.faces_min), max(args.faces_min, args.faces_max))
            rot = random.uniform(0.0, 360.0) if args.rotation else 0.0

        # Effective radius to use for overlap tests and keepout:
        # For polygons we use their circumradius r. For circles it's r.
        eff_r = r

        ok = False
        for _try in range(args.tries_per_obstacle):
            cx = random.uniform(minx, maxx)
            cy = random.uniform(miny, maxy)

            if intersects_start_goal(cx, cy, eff_r, (sx, sy), (gx, gy), keepout):
                continue
            if args.nonoverlap and overlaps_existing(cx, cy, eff_r, placed_for_overlap, pad=inflate):
                continue

            # Accept
            if is_circle:
                obstacles_lines.append(f"OBSTACLE=circle:{cx:.2f},{cy:.2f},{r:.2f}")
            else:
                obstacles_lines.append(f"OBSTACLE=polygon:{cx:.2f},{cy:.2f},{r:.2f},{faces},{rot:.2f}")
            placed_for_overlap.append((cx, cy, eff_r))
            ok = True
            break

        if not ok:
            # Could not place this obstacle; skip silently
            pass

    out_file, bak_file = replace_obstacles(env_path, obstacles_lines, out_path)
    if bak_file:
        print(f"Updated: {out_file} (backup at {bak_file})")
    else:
        print(f"Wrote: {out_file}")
    print(f"Placed {len(obstacles_lines)} obstacles.")

if __name__ == "__main__":
    main()
