# Mission Planner

A web-based mission planning interface for an autonomous water surface drone (Catfish), 
built as a degree project at Halmstad University in collaboration with an industry partner. 
Replaces ArduPilot's Mission Planner desktop application with an integrated web interface 
running on the drone's onboard Raspberry Pi 5 (BlueOS).

## Features

- Interactive map interface for mission planning (Leaflet.js)
- Lawnmower/boustrophedon coverage path generation
- No-go zone drawing and avoidance
- A* return path to home position
- Transit waypoints
- Mission save, load, edit and delete
- Per-waypoint loiter time configuration
- Live telemetry display
- MAVLink communication via mavlink2rest

## System Architecture

The system runs as a Flask application inside BlueOS on a Raspberry Pi 5 aboard the USV.
The frontend communicates with the Flask backend via REST API, which forwards MAVLink 
commands to the Pixhawk autopilot through mavlink2rest on port 6040.

## Running Locally (Simulation)

A local simulation version is included for development and testing without the drone.

### Requirements

pip install flask flask-cors shapely pyproj numpy

### Start

python app_local.py

### Open in browser

http://localhost:5000/mission

All drone commands return simulated responses. Route generation and mission 
save/load/edit/delete work fully.

## File Structure

app.py                  # Production Flask backend (runs on Raspberry Pi/BlueOS)
app_local.py            # Local simulation backend
index.html              # Frontend (production)
index_local.html        # Frontend (local simulation)
requirements_local.txt  # Python dependencies for local simulation
saved_missions/         # Mission JSON files stored here

## Tech Stack

- Frontend: Leaflet.js, leaflet-editable, Turf.js
- Backend: Flask (Python)
- Path planning: Shapely, pyproj, NumPy
- Autopilot communication: MAVLink via mavlink2rest
- Hardware: Raspberry Pi 5, Pixhawk autopilot, BlueOS
