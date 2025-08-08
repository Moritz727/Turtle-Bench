import turtle
import math
import argparse
import json
from pathlib import Path

# -------------------------
# Parsing helpers (robust, allow inline comments)
# -------------------------
def _decomment(line: str) -> str:
    return line.split('#', 1)[0].strip()

def parse_env(path):
    """
    Parse the new environment.txt format:
      - Strips inline '#' comments
      - Supports SCREEN_WIDTH/HEIGHT, START_*, GOAL_*, TURN_CONVENTION
      - Supports robot fields (RECT/CIRCLE/POLY) with *_CM naming
      - Parses OBSTACLE lines:
          OBSTACLE=circle:cx,cy,r
          OBSTACLE=polygon:cx,cy,R,faces[,rot_deg]
    Produces cfg with OBSTACLES=[('circle',(cx,cy,r)), ('polygon',(cx,cy,R,faces,rot))]
    """
    from pathlib import Path

    def decomment(s): return s.split('#', 1)[0].strip()

    cfg = {}
    obstacles = []
    for i, raw in enumerate(Path(path).read_text(encoding="utf-8").splitlines(), start=1):
        line = decomment(raw)
        if not line:
            continue
        if "=" not in line:
            raise ValueError(f"Expected key=value on line {i}: {raw}")

        k, v = [x.strip() for x in line.split("=", 1)]

        if k == "OBSTACLE":
            try:
                typ, rest = v.split(":", 1)
                typ = typ.strip().lower()
                nums = [float(x) for x in rest.split(",")]
                if typ == "circle":
                    if len(nums) < 3:
                        raise ValueError("circle expects cx,cy,r")
                    cx, cy, r = nums[:3]
                    obstacles.append(("circle", (cx, cy, r)))
                elif typ == "polygon":
                    if len(nums) not in (4, 5):
                        raise ValueError("polygon expects cx,cy,R,faces[,rot_deg]")
                    cx, cy, R, faces = nums[:4]
                    rot = nums[4] if len(nums) == 5 else 0.0
                    obstacles.append(("polygon", (cx, cy, R, int(faces), rot)))
                else:
                    raise ValueError(f"unknown obstacle type '{typ}'")
            except Exception as e:
                raise ValueError(f"Failed to parse OBSTACLE on line {i}: {raw}\n{e}") from e
            continue

        # Floats
        if k in {
            "START_X", "START_Y", "GOAL_X", "GOAL_Y", "GOAL_RADIUS_CM",
            "ROBOT_WIDTH_CM", "ROBOT_HEIGHT_CM", "ROBOT_RADIUS_CM",
            "ROBOT_POLY_RADIUS_CM", "CLEARANCE_CM", "START_HEADING_DEG"
        }:
            try:
                cfg[k] = float(v)
            except Exception as e:
                raise ValueError(f"Failed to parse float for {k} on line {i}: {raw}\n{e}") from e
            continue

        # Ints
        if k in {"SCREEN_WIDTH", "SCREEN_HEIGHT", "ROBOT_POLY_FACES"}:
            try:
                cfg[k] = int(float(v))  # tolerate "600" or "600.0"
            except Exception as e:
                raise ValueError(f"Failed to parse int for {k} on line {i}: {raw}\n{e}") from e
            continue

        # Enums / strings
        if k in {"ROBOT_SHAPE", "TURN_CONVENTION"}:
            cfg[k] = v.strip().lower()
            continue

        # Unknown -> warn but don’t crash
        print(f"[parse_env] Warning: unknown key '{k}' on line {i} – ignoring.")

    cfg["OBSTACLES"] = obstacles

    # Defaults
    cfg.setdefault("SCREEN_WIDTH", 800)
    cfg.setdefault("SCREEN_HEIGHT", 600)
    cfg.setdefault("START_X", 0.0)
    cfg.setdefault("START_Y", 0.0)
    cfg.setdefault("START_HEADING_DEG", 0.0)
    cfg.setdefault("GOAL_X", 0.0)
    cfg.setdefault("GOAL_Y", 0.0)
    cfg.setdefault("GOAL_RADIUS_CM", 10.0)
    cfg.setdefault("TURN_CONVENTION", "right_positive")

    cfg.setdefault("ROBOT_SHAPE", "rect")
    cfg.setdefault("ROBOT_WIDTH_CM", 20.0)
    cfg.setdefault("ROBOT_HEIGHT_CM", 20.0)
    cfg.setdefault("ROBOT_RADIUS_CM", 0.0)
    cfg.setdefault("ROBOT_POLY_FACES", 0)
    cfg.setdefault("ROBOT_POLY_RADIUS_CM", 0.0)
    cfg.setdefault("CLEARANCE_CM", 5.0)

    return cfg



def parse_instructions(path):
    ops = []
    lines = Path(path).read_text(encoding="utf-8").splitlines()
    for i, raw in enumerate(lines, start=1):
        s = _decomment(raw)
        if not s:
            continue
        low = s.lower().replace(" ", "")
        try:
            if low.startswith("move(") and low.endswith(")"):
                val_str = s[s.find("(")+1:s.rfind(")")]
                d = float(_decomment(val_str).strip())
                ops.append(("move", d))
            elif low.startswith("turn(") and low.endswith(")"):
                val_str = s[s.find("(")+1:s.rfind(")")]
                a = float(_decomment(val_str).strip())
                ops.append(("turn", a))
            else:
                raise ValueError("Instruction must be move(<cm>) or turn(<deg>)")
        except Exception as e:
            raise ValueError(f"Invalid instruction on line {i}: {raw}\n{e}") from e
    return ops

# -------------------------
# Geometry / metrics
# -------------------------
def _get(cfg, key, default):
    return cfg[key] if key in cfg else default

def robot_inflation_radius(cfg):
    """
    Return the inflation amount to apply to *obstacles* so that the robot can be treated as a point.
    This is the robot's maximum distance from its center (circumradius) + CLEARANCE_CM.
    """
    shape = _get(cfg, "ROBOT_SHAPE", "rect").lower()
    clearance = float(_get(cfg, "CLEARANCE_CM", 5.0))

    if shape == "circle":
        rr = float(_get(cfg, "ROBOT_RADIUS_CM", 0.0))
        return rr + clearance

    elif shape == "rect":
        w = float(_get(cfg, "ROBOT_WIDTH_CM", 20.0))
        h = float(_get(cfg, "ROBOT_HEIGHT_CM", 20.0))
        circum = 0.5 * math.hypot(w, h)  # max distance from center (half-diagonal)
        return circum + clearance

    elif shape == "polygon":
        faces = int(_get(cfg, "ROBOT_POLY_FACES", 0))
        pr = float(_get(cfg, "ROBOT_POLY_RADIUS_CM", 0.0))  # circumradius
        if faces < 3 or pr <= 0:
            # Fallback safely to rectangle’s half-diagonal if misconfigured
            w = float(_get(cfg, "ROBOT_WIDTH_CM", 20.0))
            h = float(_get(cfg, "ROBOT_HEIGHT_CM", 20.0))
            circum = 0.5 * math.hypot(w, h)
            return circum + clearance
        return pr + clearance

    else:
        # Unknown shape => conservative fallback
        w = float(_get(cfg, "ROBOT_WIDTH_CM", 20.0))
        h = float(_get(cfg, "ROBOT_HEIGHT_CM", 20.0))
        circum = 0.5 * math.hypot(w, h)
        return circum + clearance


def regular_polygon_vertices(cx, cy, R, faces, rot_deg=0.0):
    """Vertices CCW starting at angle = rot_deg (0° along +x)."""
    verts = []
    rot = math.radians(rot_deg)
    for k in range(faces):
        ang = rot + 2*math.pi * k / faces
        verts.append((cx + R*math.cos(ang), cy + R*math.sin(ang)))
    return verts

def _dot(ax, ay, bx, by): return ax*bx + ay*by

def point_segment_distance(px, py, ax, ay, bx, by):
    """Distance from point P to segment AB."""
    abx, aby = (bx - ax), (by - ay)
    apx, apy = (px - ax), (py - ay)
    denom = abx*abx + aby*aby
    if denom == 0.0:
        return math.hypot(px - ax, py - ay)
    t = max(0.0, min(1.0, _dot(apx, apy, abx, aby) / denom))
    cx, cy = (ax + t*abx, ay + t*aby)
    return math.hypot(px - cx, py - cy)

def is_inside_convex_polygon(px, py, verts):
    """All cross products must have the same sign for convex CCW polygon."""
    inside = True
    n = len(verts)
    for i in range(n):
        x1, y1 = verts[i]
        x2, y2 = verts[(i+1) % n]
        cross = (x2 - x1)*(py - y1) - (y2 - y1)*(px - x1)
        if cross < 0:  # assuming verts CCW; if neg, point is to the right => outside
            inside = False
            break
    return inside

def signed_distance_to_regular_polygon(px, py, cx, cy, R, faces, rot_deg):
    """Signed distance to polygon boundary (convex, CCW).
       Negative if inside, positive if outside."""
    verts = regular_polygon_vertices(cx, cy, R, faces, rot_deg)
    # Distance to edges:
    d_edge = float("inf")
    n = len(verts)
    for i in range(n):
        ax, ay = verts[i]
        bx, by = verts[(i+1) % n]
        d_edge = min(d_edge, point_segment_distance(px, py, ax, ay, bx, by))
    inside = is_inside_convex_polygon(px, py, verts)
    return -d_edge if inside else d_edge

def min_clearance_to_obstacles(point, obstacles, inflate):
    """Return min distance from point to *inflated* obstacle boundary (negative => inside)."""
    px, py = point
    dmin = float("inf")
    for kind, data in obstacles:
        if kind == "circle":
            cx, cy, r = data
            R = r + inflate
            d = math.hypot(px - cx, py - cy) - R
        elif kind == "polygon":
            cx, cy, r, faces, rot = data
            # Use circumradius r; inflation handled by subtracting 'inflate'
            sd = signed_distance_to_regular_polygon(px, py, cx, cy, r, faces, rot)
            d = sd - inflate
        else:
            raise NotImplementedError
        dmin = min(dmin, d)
    return dmin

# -------------------------
# Bounds & view fitting
# -------------------------
def compute_world_bounds(cfg, inflate):
    xs = [cfg["START_X"], cfg["GOAL_X"]]
    ys = [cfg["START_Y"], cfg["GOAL_Y"]]
    for kind, data in cfg["OBSTACLES"]:
        if kind == "circle":
            cx, cy, r = data
            R = r + inflate
            xs += [cx - R, cx + R]
            ys += [cy - R, cy + R]
        elif kind == "polygon":
            cx, cy, r, faces, rot = data
            R = r + inflate
            xs += [cx - R, cx + R]
            ys += [cy - R, cy + R]
    minx, maxx = min(xs), max(xs)
    miny, maxy = min(ys), max(ys)
    width = maxx - minx
    height = maxy - miny
    # Add padding (10% + 20 cm)
    pad = max(20.0, 0.1 * max(width, height))
    return (minx - pad, miny - pad, maxx + pad, maxy + pad)

# -------------------------
# Drawing & execution
# -------------------------
def draw_circle(t, center, radius, color='black'):
    t.penup()
    t.color(color)
    t.goto(center[0], center[1] - radius)
    t.pendown()
    t.circle(radius)
    t.penup()

def draw_polygon(t, cx, cy, r, faces, rot_deg=0.0, color='black'):
    verts = regular_polygon_vertices(cx, cy, r, faces, rot_deg)
    t.penup()
    t.color(color)
    t.goto(verts[0])
    t.pendown()
    for v in verts[1:]:
        t.goto(v)
    t.goto(verts[0])
    t.penup()

def setup_screen_and_world(cfg, bounds):
    screen = turtle.Screen()
    screen.setup(width=cfg["SCREEN_WIDTH"], height=cfg["SCREEN_HEIGHT"])
    screen.title("Robot Path Benchmark")
    # Fit world coordinates so everything is visible
    left, bottom, right, top = bounds
    screen.setworldcoordinates(left, bottom, right, top)
    return screen

def setup_robot(cfg):
    bot = turtle.Turtle()
    bot.hideturtle()
    bot.speed(0)  # draw scene fast
    bot.width(2)
    bot.color("black")
    bot.penup()
    bot.goto(cfg["START_X"], cfg["START_Y"])
    bot.setheading(cfg["START_HEADING_DEG"])  # CCW from +x
    bot.pendown()
    bot.showturtle()
    bot.speed(6)  # runtime speed
    return bot

def draw_scene(pen, cfg, inflate):
    for kind, data in cfg["OBSTACLES"]:
        if kind == "circle":
            cx, cy, r = data
            draw_circle(pen, (cx, cy), r, color="gray")
            draw_circle(pen, (cx, cy), r + inflate, color="blue")
        elif kind == "polygon":
            cx, cy, r, faces, rot = data
            draw_polygon(pen, cx, cy, r, faces, rot, color="gray")
            draw_polygon(pen, cx, cy, r + inflate, faces, rot, color="blue")
    # Goal
    gx, gy = cfg["GOAL_X"], cfg["GOAL_Y"]
    pen.penup(); pen.goto(gx, gy); pen.dot(10, "green"); pen.pendown()

def execute(bot, cfg, ops, obstacles, inflate, sample_step_cm=2.0):
    total_len = 0.0
    min_clear = float("inf")
    collisions = 0

    def sample_and_draw_forward(dist):
        nonlocal total_len, min_clear, collisions
        steps = max(1, int(abs(dist) / sample_step_cm))
        step_len = dist / steps
        for _ in range(steps):
            bot.forward(step_len)
            total_len += abs(step_len)
            px, py = bot.position()
            d = min_clearance_to_obstacles((px, py), obstacles, inflate)
            min_clear = min(min_clear, d)
            if d < 0:
                collisions += 1

    for op, val in ops:
        if op == "move":
            sample_and_draw_forward(val)
        elif op == "turn":
            if cfg["TURN_CONVENTION"] == "right_positive":
                if val >= 0: bot.right(val)
                else:        bot.left(-val)
            else:
                if val >= 0: bot.left(val)
                else:        bot.right(-val)

    px, py = bot.position()
    gx, gy = cfg["GOAL_X"], cfg["GOAL_Y"]
    reached_goal = (math.hypot(px - gx, py - gy) <= cfg["GOAL_RADIUS_CM"])

    return {
        "total_path_length_cm": round(total_len, 3),
        "final_x": round(px, 3),
        "final_y": round(py, 3),
        "final_heading_deg": round(bot.heading(), 3),
        "reached_goal": bool(reached_goal),
        "min_clearance_cm": (None if min_clear == float("inf") else round(min_clear, 3)),
        "collision_samples": collisions,
        "turn_convention": cfg["TURN_CONVENTION"]
    }

def main():
    ap = argparse.ArgumentParser(description="Turtle benchmark runner")
    ap.add_argument("--env", default="environment.txt", help="Path to environment.txt")
    ap.add_argument("--instructions", default="instructions.txt", help="Path to instructions.txt")
    ap.add_argument("--speed", type=int, default=6, help="Turtle speed 1..10")
    ap.add_argument("--metrics_out", default="metrics.json", help="Where to save metrics JSON")
    args = ap.parse_args()

    cfg = parse_env(args.env)
    ops = parse_instructions(args.instructions)

    inflate = robot_inflation_radius(cfg)
    bounds = compute_world_bounds(cfg, inflate)
    screen = setup_screen_and_world(cfg, bounds)

    robot = setup_robot(cfg)
    robot.speed(args.speed)

    pen = turtle.Turtle()
    pen.hideturtle()
    pen.speed(0)
    draw_scene(pen, cfg, inflate)

    metrics = execute(robot, cfg, ops, cfg["OBSTACLES"], inflate)
    Path(args.metrics_out).write_text(json.dumps(metrics, indent=2))
    print(json.dumps(metrics, indent=2))
    turtle.done()

if __name__ == "__main__":
    main()
