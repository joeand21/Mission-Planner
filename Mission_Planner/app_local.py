from flask import Flask, jsonify, request, send_from_directory
from flask_cors import CORS
from pathlib import Path
import json
import datetime
import os

from shapely.geometry import Polygon, LineString, MultiLineString, Point
from shapely.ops import transform
from shapely.affinity import rotate
import pyproj
import numpy as np
import heapq

BASE_DIR = Path(__file__).resolve().parent
MISSIONS_DIR = BASE_DIR / "saved_missions"
MISSIONS_DIR.mkdir(exist_ok=True)

app = Flask(__name__, static_folder=str(BASE_DIR))
CORS(app)

# ── Serve frontend ────────────────────────────────────
@app.route('/')
@app.route('/mission')
def serve_map():
    return send_from_directory(str(BASE_DIR), 'index_local.html')

# ── Mock telemetry ────────────────────────────────────
@app.route('/nav/telemetry', methods=['GET'])
def nav_telemetry():
    return jsonify({
        'lat': 56.6744,
        'lng': 12.8578,
        'alt': 0.0,
        'heading': 0,
        'groundspeed': 0.0,
        'battery': 100,
        'mode': 0,
        'armed': False
    })

# ── Mock drone commands ───────────────────────────────
@app.route('/nav/ArmOn', methods=['POST'])
def arm_on():
    return jsonify({'status': 'Armed (simulated)'})

@app.route('/nav/Disarm', methods=['POST'])
def disarm():
    return jsonify({'status': 'Disarmed (simulated)'})

@app.route('/nav/AutoModeOn', methods=['POST'])
def auto_mode():
    return jsonify({'status': 'Auto mode (simulated)'})

@app.route('/nav/hold', methods=['POST'])
def hold():
    return jsonify({'status': 'Hold mode (simulated)'})

@app.route('/nav/manual', methods=['POST'])
def manual():
    return jsonify({'status': 'Manual mode (simulated)'})

@app.route('/nav/start', methods=['POST'])
def nav_start():
    return jsonify({'status': 'Mission started (simulated)'})

@app.route('/nav/stop', methods=['POST'])
def nav_stop():
    return jsonify({'status': 'Stopped (simulated)'})

@app.route('/nav/rtl', methods=['POST'])
def nav_rtl():
    return jsonify({'status': 'Returning home (simulated)'})

@app.route('/nav/pause', methods=['POST'])
def nav_pause():
    return jsonify({'status': 'Mission paused (simulated)'})

@app.route('/nav/resume', methods=['POST'])
def nav_resume():
    return jsonify({'status': 'Mission resumed (simulated)'})

@app.route('/nav/jump_to_waypoint', methods=['POST'])
def nav_jump():
    seq = request.json.get('seq', 0)
    return jsonify({'status': f'Jumped to waypoint {seq} (simulated)'})

@app.route('/nav/upload_mission', methods=['POST'])
def nav_upload():
    geojson = request.json
    coords = geojson['features'][0]['geometry']['coordinates']
    return jsonify({'status': 'success', 'waypoints': len(coords)})

@app.route('/nav/fetch_mission', methods=['GET'])
def nav_fetch():
    return jsonify({'waypoints': [], 'count': 0})

@app.route('/nav/clear_mission', methods=['POST'])
def nav_clear():
    return jsonify({'status': 'Mission cleared (simulated)'})

@app.route('/nav/set_param', methods=['POST'])
def nav_set_param():
    return jsonify({'status': 'Parameter set (simulated)'})

@app.route('/nav/mission_count', methods=['GET'])
def nav_mission_count():
    return jsonify({'count': 0, 'has_mission': False})

# ── Lawnmower generation ──────────────────────────────
def run_lawnmower(polygon_coords, spacing_meters, angle_deg=90, safety_meters=0):
    coords_deg = [(c[0], c[1]) for c in polygon_coords]
    if coords_deg[0] != coords_deg[-1]:
        coords_deg.append(coords_deg[0])
    poly_deg = Polygon(coords_deg)

    centroid = poly_deg.centroid
    utm_zone = int((centroid.x + 180) / 6) + 1
    hemi = 6 if centroid.y >= 0 else 7
    crs_proj = f"EPSG:32{hemi}{utm_zone:02d}"
    to_proj = pyproj.Transformer.from_crs("EPSG:4326", crs_proj, always_xy=True)
    to_deg  = pyproj.Transformer.from_crs(crs_proj, "EPSG:4326", always_xy=True)
    poly_proj = transform(to_proj.transform, poly_deg)

    if safety_meters > 0:
        poly_proj = poly_proj.buffer(-safety_meters)
        if poly_proj.is_empty:
            return []

    math_angle = (90.0 - angle_deg) % 360.0
    centroid_proj = poly_proj.centroid
    poly_rot = rotate(poly_proj, -math_angle, origin=centroid_proj)

    minx, miny, maxx, maxy = poly_rot.bounds
    width = maxx - minx
    y_values = np.arange(miny + spacing_meters / 2, maxy, spacing_meters)

    lines = []
    for i, y in enumerate(y_values):
        line = LineString([(minx - width, y), (maxx + width, y)])
        clipped = poly_rot.intersection(line)
        if clipped.is_empty:
            continue
        if isinstance(clipped, LineString):
            segments = [clipped]
        elif isinstance(clipped, MultiLineString):
            segments = list(clipped.geoms)
        else:
            continue
        segments.sort(key=lambda s: s.coords[0][0])
        if i % 2 == 0:
            for seg in segments:
                lines.extend([seg.coords[0], seg.coords[-1]])
        else:
            for seg in reversed(segments):
                lines.extend([seg.coords[-1], seg.coords[0]])

    if not lines:
        return []

    path_rot = LineString(lines)
    path_proj = rotate(path_rot, math_angle, origin=centroid_proj)
    path_deg = transform(to_deg.transform, path_proj)
    return [[c[0], c[1]] for c in path_deg.coords]


def run_lawnmower_with_obstacles(polygon_coords, spacing_meters, angle_deg=90, safety_meters=0, no_go_zones=None):
    if not no_go_zones:
        return run_lawnmower(polygon_coords, spacing_meters, angle_deg, safety_meters)

    coords_deg = [(c[0], c[1]) for c in polygon_coords]
    if coords_deg[0] != coords_deg[-1]:
        coords_deg.append(coords_deg[0])
    poly_deg = Polygon(coords_deg)

    centroid = poly_deg.centroid
    utm_zone = int((centroid.x + 180) / 6) + 1
    hemi = 6 if centroid.y >= 0 else 7
    crs_proj = f"EPSG:32{hemi}{utm_zone:02d}"
    to_proj = pyproj.Transformer.from_crs("EPSG:4326", crs_proj, always_xy=True)
    to_deg  = pyproj.Transformer.from_crs(crs_proj, "EPSG:4326", always_xy=True)
    poly_proj = transform(to_proj.transform, poly_deg)

    if safety_meters > 0:
        poly_proj = poly_proj.buffer(-safety_meters)
        if poly_proj.is_empty:
            return []

    no_go_proj = []
    for zone in no_go_zones:
        zone_deg = [(c[0], c[1]) for c in zone]
        if zone_deg[0] != zone_deg[-1]:
            zone_deg.append(zone_deg[0])
        try:
            no_go_proj.append(transform(to_proj.transform, Polygon(zone_deg)))
        except Exception:
            pass

    math_angle = (90.0 - angle_deg) % 360.0
    centroid_proj = poly_proj.centroid
    poly_rot = rotate(poly_proj, -math_angle, origin=centroid_proj)
    no_go_rot = [rotate(z, -math_angle, origin=centroid_proj) for z in no_go_proj]

    minx, miny, maxx, maxy = poly_rot.bounds
    width = maxx - minx
    y_values = np.arange(miny + spacing_meters / 2, maxy, spacing_meters)

    lines = []
    for i, y in enumerate(y_values):
        line = LineString([(minx - width, y), (maxx + width, y)])
        clipped = poly_rot.intersection(line)
        if clipped.is_empty:
            continue
        for nogo in no_go_rot:
            clipped = clipped.difference(nogo)
        if clipped.is_empty:
            continue
        if isinstance(clipped, LineString):
            segments = [clipped]
        elif isinstance(clipped, MultiLineString):
            segments = list(clipped.geoms)
        else:
            continue
        segments.sort(key=lambda s: s.coords[0][0])
        if i % 2 == 0:
            for seg in segments:
                lines.extend([seg.coords[0], seg.coords[-1]])
        else:
            for seg in reversed(segments):
                lines.extend([seg.coords[-1], seg.coords[0]])

    if not lines:
        return []

    path_rot = LineString(lines)
    path_proj = rotate(path_rot, math_angle, origin=centroid_proj)
    path_deg = transform(to_deg.transform, path_proj)
    return [[c[0], c[1]] for c in path_deg.coords]


def compute_return_path(polygon_coords, spacing_meters, safety_meters, last_waypoint, home_coord, no_go_zones=None):
    coords_deg = [(c[0], c[1]) for c in polygon_coords]
    if coords_deg[0] != coords_deg[-1]:
        coords_deg.append(coords_deg[0])
    poly_deg = Polygon(coords_deg)

    centroid = poly_deg.centroid
    utm_zone = int((centroid.x + 180) / 6) + 1
    hemi = 6 if centroid.y >= 0 else 7
    crs_proj = f"EPSG:32{hemi}{utm_zone:02d}"
    to_proj = pyproj.Transformer.from_crs("EPSG:4326", crs_proj, always_xy=True)
    to_deg  = pyproj.Transformer.from_crs(crs_proj, "EPSG:4326", always_xy=True)
    poly_proj = transform(to_proj.transform, poly_deg)

    if safety_meters > 0:
        poly_proj = poly_proj.buffer(-safety_meters)
        if poly_proj.is_empty:
            return []

    minx, miny, maxx, maxy = poly_proj.bounds
    step = spacing_meters
    cols = int((maxx - minx) / step) + 1
    rows = int((maxy - miny) / step) + 1

    grid = []
    for r in range(rows):
        row = []
        y = miny + r * step
        for c in range(cols):
            x = minx + c * step
            row.append(0 if poly_proj.contains(Point(x, y)) else 1)
        grid.append(row)

    if no_go_zones:
        for zone_coords in no_go_zones:
            zone_deg = [(c[0], c[1]) for c in zone_coords]
            if zone_deg[0] != zone_deg[-1]:
                zone_deg.append(zone_deg[0])
            try:
                zone_poly = transform(to_proj.transform, Polygon(zone_deg))
                for r in range(rows):
                    y = miny + r * step
                    for c in range(cols):
                        x = minx + c * step
                        if zone_poly.contains(Point(x, y)):
                            grid[r][c] = 1
            except Exception:
                pass

    def to_grid(lng, lat):
        x, y = to_proj.transform(lng, lat)
        c = int(round((x - minx) / step))
        r = int(round((y - miny) / step))
        return r, c

    def to_coord(r, c):
        x = minx + c * step
        y = miny + r * step
        lng, lat = to_deg.transform(x, y)
        return [lng, lat]

    def is_free(r, c):
        if r < 0 or r >= rows or c < 0 or c >= cols:
            return True
        return grid[r][c] == 0

    def nearest_free_inside(r, c):
        if 0 <= r < rows and 0 <= c < cols and grid[r][c] == 0:
            return r, c
        for dist in range(1, max(rows, cols)):
            for dr in range(-dist, dist+1):
                for dc in range(-dist, dist+1):
                    nr, nc = r+dr, c+dc
                    if 0 <= nr < rows and 0 <= nc < cols and grid[nr][nc] == 0:
                        return nr, nc
        return r, c

    def astar(start, goal):
        h = lambda a, b: abs(a[0]-b[0]) + abs(a[1]-b[1])
        open_set = [(h(start, goal), 0, start, None)]
        came_from = {}
        gscore = {start: 0}
        closed = set()
        while open_set:
            _, g, node, parent = heapq.heappop(open_set)
            if node in closed:
                continue
            came_from[node] = parent
            if node == goal:
                path, cur = [], node
                while cur is not None:
                    path.append(cur)
                    cur = came_from[cur]
                path.reverse()
                return path
            closed.add(node)
            for dr, dc in [(-1,0),(1,0),(0,-1),(0,1)]:
                nr, nc = node[0]+dr, node[1]+dc
                if is_free(nr, nc):
                    ng = g + 1
                    if (nr,nc) not in gscore or ng < gscore[(nr,nc)]:
                        gscore[(nr,nc)] = ng
                        heapq.heappush(open_set, (ng + h((nr,nc), goal), ng, (nr,nc), node))
        return None

    start_grid = to_grid(last_waypoint[0], last_waypoint[1])
    goal_grid  = to_grid(home_coord[0], home_coord[1])
    start_inside = nearest_free_inside(*start_grid)
    goal_inside  = nearest_free_inside(*goal_grid)
    path = astar(start_inside, goal_inside)
    if not path:
        return []

    full_path = [start_grid] + path + [goal_grid]
    deduped = [full_path[0]]
    for p in full_path[1:]:
        if p != deduped[-1]:
            deduped.append(p)
    return [to_coord(r, c) for r, c in deduped]


@app.route('/nav/generate_lawnmower', methods=['POST'])
def generate_lawnmower_route():
    try:
        data = request.json
        coords  = data['coordinates']
        spacing = float(data.get('spacing', 10))
        angle   = float(data.get('angle', 90))
        safety  = float(data.get('safety', 0))
        no_go   = data.get('no_go_zones', None)
        home    = data.get('home_coord', None)

        waypoints = run_lawnmower_with_obstacles(coords, spacing, angle, safety, no_go)

        return_path = []
        if home and waypoints:
            return_path = compute_return_path(
                coords, spacing, safety,
                last_waypoint=waypoints[-1],
                home_coord=home,
                no_go_zones=no_go
            )

        return jsonify({'status': 'success', 'waypoints': waypoints, 'return_path': return_path})
    except Exception as e:
        import traceback
        traceback.print_exc()
        return jsonify({'error': str(e)}), 500


# ── Mission save/load/delete/overwrite ────────────────
@app.route('/missions/save', methods=['POST'])
def save_mission():
    try:
        data = request.get_json()
        name = data.get('name', '').strip()
        geojson = data.get('geojson')
        if not name:
            return jsonify({'error': 'Mission name required'}), 400
        safe_name = "".join(c for c in name if c.isalnum() or c in (' ', '-', '_')).strip()
        timestamp = datetime.datetime.now().strftime("%Y%m%d_%H%M%S")
        filename = f"{timestamp}_{safe_name}.json"
        with open(MISSIONS_DIR / filename, 'w') as f:
            json.dump({'name': name, 'created': datetime.datetime.now().isoformat(), 'geojson': geojson}, f, indent=2)
        return jsonify({'status': 'saved', 'filename': filename})
    except Exception as e:
        return jsonify({'error': str(e)}), 500

@app.route('/missions/list', methods=['GET'])
def list_missions():
    try:
        missions = []
        for f in sorted(MISSIONS_DIR.glob('*.json'), reverse=True):
            with open(f) as file:
                data = json.load(file)
                missions.append({'filename': f.name, 'name': data.get('name', f.stem), 'created': data.get('created', '')})
        return jsonify(missions)
    except Exception as e:
        return jsonify({'error': str(e)}), 500

@app.route('/missions/load/<filename>', methods=['GET'])
def load_mission(filename):
    try:
        filepath = MISSIONS_DIR / Path(filename).name
        if not filepath.exists():
            return jsonify({'error': 'Mission not found'}), 404
        with open(filepath) as f:
            data = json.load(f)
        if not data.get('geojson') or 'features' not in data['geojson']:
            return jsonify({'error': 'Mission has invalid geojson'}), 400
        return jsonify(data)
    except Exception as e:
        return jsonify({'error': str(e)}), 500

@app.route('/missions/delete/<filename>', methods=['DELETE'])
def delete_mission(filename):
    try:
        filepath = MISSIONS_DIR / Path(filename).name
        if not filepath.exists():
            return jsonify({'error': 'Not found'}), 404
        filepath.unlink()
        return jsonify({'status': 'deleted'})
    except Exception as e:
        return jsonify({'error': str(e)}), 500

@app.route('/missions/overwrite/<filename>', methods=['PUT'])
def overwrite_mission(filename):
    try:
        filepath = MISSIONS_DIR / Path(filename).name
        if not filepath.exists():
            return jsonify({'error': 'Mission not found'}), 404
        with open(filepath) as f:
            existing = json.load(f)
        data = request.get_json()
        existing['geojson'] = data.get('geojson')
        filepath.unlink()
        with open(filepath, 'w') as f:
            json.dump(existing, f, indent=2)
        return jsonify({'status': 'overwritten', 'filename': filename})
    except Exception as e:
        return jsonify({'error': str(e)}), 500


if __name__ == '__main__':
    print("Starting local mission planner at http://localhost:8080/mission")
    app.run(debug=True, port=8080, host='0.0.0.0')
