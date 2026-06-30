#!/usr/bin/env python3

from pathlib import Path
import csv
import json
import math
import re
import sys

#in html we need to limit the number of drawn scan rays for performance, otherwise the browser may struggle with thousands of lines.
MAX_DRAWN_SCAN_RAYS = 400


#parses numbers from strings to float
def parse_number(value):
    text = str(value).strip()

    match = re.search(r"[-+]?\d*\.?\d+", text)
    if not match:
        raise ValueError(f"Could not parse number from: {value}")

    return float(match.group(0))

# reads simple key-value config files, supporting various separators and optional units, and returns a dictionary of parsed values.
def read_key_value_file(config_path):

    config_path = Path(config_path)

    values = {}

    if not config_path.exists():
        return values

    for raw_line in config_path.read_text(encoding="utf-8").splitlines():
        line = raw_line.split("#", 1)[0].strip()

        if not line:
            continue

        if "=" in line:
            key, value = line.split("=", 1)
        elif ":" in line:
            key, value = line.split(":", 1)
        else:
            parts = line.split(maxsplit=1)
            if len(parts) != 2:
                continue

            key, value = parts

        key = key.strip()
        value = value.strip()

        try:
            values[key] = parse_number(value)
        except ValueError:
            continue

    return values

# Reads drone_config.txt, supporting both named key-value format and positional format, and extracts lidar-related parameters into a dictionary.
def read_drone_config(config_path):

    config_path = Path(config_path)

    result = {
        "lidarMinRangeCm": None,
        "lidarMaxRangeCm": None,
        "lidarBeamSpacingCm": None,
        "lidarFovCircleCount": None,
    }

    if not config_path.exists():
        print(f"Warning: drone_config.txt was not found at: {config_path}")
        return result

    text = config_path.read_text(encoding="utf-8")

    # First try named/key-value format.
    values = read_key_value_file(config_path)

    if "lidarMinRangeCm" in values:
        result["lidarMinRangeCm"] = values["lidarMinRangeCm"]

    if "lidarMaxRangeCm" in values:
        result["lidarMaxRangeCm"] = values["lidarMaxRangeCm"]

    if "lidarBeamSpacingCm" in values:
        result["lidarBeamSpacingCm"] = values["lidarBeamSpacingCm"]

    if "lidarFovCircleCount" in values:
        result["lidarFovCircleCount"] = values["lidarFovCircleCount"]

    # If named format worked, return it.
    if result["lidarMaxRangeCm"] is not None:
        print("Read drone_config.txt as named config format.")
        return result

    # Otherwise parse positional format.
    numbers = []

    for match in re.finditer(r"[-+]?\d*\.?\d+", text):
        numbers.append(float(match.group(0)))

    if len(numbers) >= 9:
        # Positional format used by the current C++ project:
        # 0 maxAdvanceCm
        # 1 maxElevateCm
        # 2 maxRotateDeg
        # 3 lidarMinRangeCm
        # 4 lidarMaxRangeCm
        # 5 lidarBeamSpacingCm
        # 6 lidarFovCircleCount
        # 7 minPassWidthCm
        # 8 minPassHeightCm
        result["lidarMinRangeCm"] = numbers[3]
        result["lidarMaxRangeCm"] = numbers[4]
        result["lidarBeamSpacingCm"] = numbers[5]
        result["lidarFovCircleCount"] = numbers[6]

        print("Read drone_config.txt as positional config format.")
        return result


    print(
        "Warning: could not read lidar fields from drone_config.txt. "
        "Expected either named fields or at least 9 positional numbers."
    )

    return result

# Reads the map from map_input.txt, which starts with sizeX sizeY sizeZ followed by cell values, and returns the dimensions and a list of occupied cells.
def read_map(map_path):

    map_path = Path(map_path)
    tokens = map_path.read_text(encoding="utf-8").split()

    if len(tokens) < 3:
        raise ValueError("map_input.txt must start with: sizeX sizeY sizeZ")

    size_x = int(tokens[0])
    size_y = int(tokens[1])
    size_z = int(tokens[2])

    expected_cells = size_x * size_y * size_z
    cell_tokens = tokens[3:3 + expected_cells]

    occupied_cells = []

    index = 0

    for height in range(size_z):
        for y in range(size_y):
            for x in range(size_x):
                value = 0

                if index < len(cell_tokens):
                    try:
                        value = int(cell_tokens[index])
                    except ValueError:
                        value = 0

                if value == 1:
                    occupied_cells.append({
                        "x": x,
                        "y": y,
                        "height": height
                    })

                index += 1

    return size_x, size_y, size_z, occupied_cells

# Reads drone_path.csv, which contains the drone's movement log with columns step,event,x,y,height,angle, and returns a list of points with parsed values.
def read_drone_path(path_path):

    path_path = Path(path_path)

    if not path_path.exists():
        raise FileNotFoundError(
            "drone_path.csv was not found. Run the simulator first."
        )

    route = []

    with path_path.open("r", encoding="utf-8") as file:
        reader = csv.DictReader(file)

        for row in reader:
            route.append({
                "step": int(parse_number(row["step"])),
                "event": row["event"],
                "x": parse_number(row["x"]),
                "y": parse_number(row["y"]),
                "height": parse_number(row["height"]),
                "angle": parse_number(row["angle"])
            })

    return route

# Converts an angle in degrees to a unit direction vector in the xy-plane, following the assignment's convention for angle orientation.
def angle_to_vector(angle_degrees):
    
    radians = math.radians(angle_degrees)

    return {
        "x": math.cos(radians),
        "y": math.sin(radians),
        "height": 0
    }

# Returns the end point of the drawn lidar ray.
def clipped_lidar_end(point, direction, max_distance_cm, size_x, size_y, size_z):

    if max_distance_cm is None or max_distance_cm <= 0:
        return None

    max_t = float(max_distance_cm)

    bounds = {
        "x": (0.0, float(size_x)),
        "y": (0.0, float(size_y)),
        "height": (0.0, float(size_z)),
    }

    for axis in ["x", "y", "height"]:
        d = direction[axis]

        if abs(d) < 1e-12:
            continue

        low, high = bounds[axis]

        if d > 0:
            t = (high - point[axis]) / d
        else:
            t = (low - point[axis]) / d

        if t >= 0:
            max_t = min(max_t, t)

    max_t = max(0.0, max_t)

    return {
        "x": point["x"] + direction["x"] * max_t,
        "y": point["y"] + direction["y"] * max_t,
        "height": point["height"] + direction["height"] * max_t,
    }

# Builds Plotly line arrays for lidar center rays.
def make_lidar_line_arrays(points, lidar_max_range_cm, size_x, size_y, size_z):

    xs = []
    ys = []
    zs = []
    labels = []

    if lidar_max_range_cm is None or lidar_max_range_cm <= 0:
        return xs, ys, zs, labels

    for point in points:
        direction = angle_to_vector(point["angle"])

        end = clipped_lidar_end(
            point,
            direction,
            lidar_max_range_cm,
            size_x,
            size_y,
            size_z
        )

        if end is None:
            continue

        label = (
            f"step {point['step']}<br>{point['event']}<br>"
            f"lidarMaxRangeCm={lidar_max_range_cm}<br>"
            f"angle={point['angle']}"
        )

        xs.extend([point["x"], end["x"], None])
        ys.extend([point["y"], end["y"], None])
        zs.extend([point["height"], end["height"], None])
        labels.extend([label, label, None])

    return xs, ys, zs, labels

# Selects points from the route that likely represent lidar scans, and downsamples them if there are too many to keep the visualization responsive.
def choose_scan_points(route):

    scan_points = [
        point for point in route
        if "scan" in point["event"].lower()
    ]

    if len(scan_points) <= MAX_DRAWN_SCAN_RAYS:
        return scan_points, 1

    stride = math.ceil(len(scan_points) / MAX_DRAWN_SCAN_RAYS)
    return scan_points[::stride], stride


def format_optional_cm(value):
    if value is None:
        return "not found"

    if abs(value - round(value)) < 1e-9:
        return f"{int(round(value))} cm"

    return f"{value} cm"

# Main function that orchestrates reading the map, drone configuration, and drone path, and generates an HTML file with a 3D visualization of the drone's route and lidar rays using Plotly.
def create_visualization(base_path):
    base_path = Path(base_path)

    map_path = base_path / "map_input.txt"
    drone_config_path = base_path / "drone_config.txt"
    drone_path = base_path / "drone_log.csv"
    output_html = base_path / "drone_path_visualization.html"

    size_x, size_y, size_z, occupied_cells = read_map(map_path)
    drone_config = read_drone_config(drone_config_path)
    route = read_drone_path(drone_path)

    if not route:
        raise ValueError("drone_path.csv is empty")

    lidar_max_range_cm = drone_config["lidarMaxRangeCm"]
    lidar_min_range_cm = drone_config["lidarMinRangeCm"]

    occupied_x = [cell["x"] for cell in occupied_cells]
    occupied_y = [cell["y"] for cell in occupied_cells]
    occupied_z = [cell["height"] for cell in occupied_cells]

    route_x = [point["x"] for point in route]
    route_y = [point["y"] for point in route]
    route_z = [point["height"] for point in route]

    route_labels = [
        f"step {p['step']}<br>{p['event']}<br>"
        f"pos=({p['x']}, {p['y']}, {p['height']})<br>"
        f"angle={p['angle']}"
        for p in route
    ]

    last = route[-1]
    direction = angle_to_vector(last["angle"])

    final_lidar_x, final_lidar_y, final_lidar_z, final_lidar_labels = make_lidar_line_arrays(
        [last],
        lidar_max_range_cm,
        size_x,
        size_y,
        size_z
    )

    scan_points, scan_stride = choose_scan_points(route)
    scan_lidar_x, scan_lidar_y, scan_lidar_z, scan_lidar_labels = make_lidar_line_arrays(
        scan_points,
        lidar_max_range_cm,
        size_x,
        size_y,
        size_z
    )

    if lidar_max_range_cm is None:
        lidar_note = (
            "lidarMaxRangeCm was not found in drone_config.txt, "
            "so lidar range lines are empty."
        )
    else:
        lidar_note = (
            f"Lidar range lines use lidarMaxRangeCm={format_optional_cm(lidar_max_range_cm)} "
            "from drone_config.txt. The drawn lines are also clipped to the map box."
        )

    if scan_stride > 1:
        scan_note = f" Scan rays were downsampled: showing every {scan_stride}th scan event."
    else:
        scan_note = ""

    html = f"""
<!DOCTYPE html>
<html>
<head>
    <meta charset="utf-8">
    <title>Drone Mapper 3D Route Visualization</title>
    <script src="https://cdn.plot.ly/plotly-2.35.2.min.js"></script>
    <style>
        body {{
            font-family: Arial, sans-serif;
            margin: 20px;
        }}

        #plot {{
            width: 100%;
            height: 85vh;
        }}

        .note {{
            background: #f3f3f3;
            padding: 10px;
            border-radius: 8px;
            margin-bottom: 12px;
            line-height: 1.45;
        }}
    </style>
</head>
<body>
    <h1>Drone Mapper — 3D Route Visualization</h1>

    <div class="note">
        <b>Legend:</b><br>
        Red cubes/points = occupied cells from map_input.txt.<br>
        Blue line = drone route.<br>
        Green marker = start.<br>
        Purple marker = final drone position.<br>
        Orange cone = final drone direction.<br>
        Orange line = final lidar center-beam max range.<br>
        Thin orange scan lines = lidar center-beam max range at scan events, hidden by default. Click the legend to show them.<br>
        <b>{lidar_note}</b>{scan_note}<br>
        lidarMinRangeCm = {format_optional_cm(lidar_min_range_cm)}.
    </div>

    <div id="plot"></div>

    <script>
        const occupiedTrace = {{
            x: {json.dumps(occupied_x)},
            y: {json.dumps(occupied_y)},
            z: {json.dumps(occupied_z)},
            mode: "markers",
            type: "scatter3d",
            name: "Occupied cells",
            marker: {{
                size: 8,
                color: "red",
                symbol: "square",
                opacity: 0.8
            }}
        }};

        const routeTrace = {{
            x: {json.dumps(route_x)},
            y: {json.dumps(route_y)},
            z: {json.dumps(route_z)},
            mode: "lines+markers",
            type: "scatter3d",
            name: "Drone route",
            text: {json.dumps(route_labels)},
            hoverinfo: "text",
            line: {{
                width: 6,
                color: "blue"
            }},
            marker: {{
                size: 5,
                color: "blue"
            }}
        }};

        const startTrace = {{
            x: [{route[0]["x"]}],
            y: [{route[0]["y"]}],
            z: [{route[0]["height"]}],
            mode: "markers",
            type: "scatter3d",
            name: "Start",
            marker: {{
                size: 10,
                color: "green"
            }}
        }};

        const endTrace = {{
            x: [{last["x"]}],
            y: [{last["y"]}],
            z: [{last["height"]}],
            mode: "markers",
            type: "scatter3d",
            name: "Final drone position",
            marker: {{
                size: 12,
                color: "purple"
            }}
        }};

        const directionCone = {{
            type: "cone",
            name: "Final direction",
            x: [{last["x"]}],
            y: [{last["y"]}],
            z: [{last["height"]}],
            u: [{direction["x"]}],
            v: [{direction["y"]}],
            w: [{direction["height"]}],
            sizemode: "absolute",
            sizeref: 0.6,
            anchor: "tail",
            colorscale: [[0, "orange"], [1, "orange"]],
            showscale: false
        }};

        const finalLidarRangeTrace = {{
            x: {json.dumps(final_lidar_x)},
            y: {json.dumps(final_lidar_y)},
            z: {json.dumps(final_lidar_z)},
            mode: "lines",
            type: "scatter3d",
            name: "Final lidar max range",
            text: {json.dumps(final_lidar_labels)},
            hoverinfo: "text",
            line: {{
                width: 8,
                color: "orange"
            }}
        }};

        const scanLidarRangeTrace = {{
            x: {json.dumps(scan_lidar_x)},
            y: {json.dumps(scan_lidar_y)},
            z: {json.dumps(scan_lidar_z)},
            mode: "lines",
            type: "scatter3d",
            name: "Scan-event lidar max ranges",
            text: {json.dumps(scan_lidar_labels)},
            hoverinfo: "text",
            visible: "legendonly",
            line: {{
                width: 2,
                color: "orange"
            }},
            opacity: 0.35
        }};

        const data = [
            occupiedTrace,
            routeTrace,
            startTrace,
            endTrace,
            directionCone,
            finalLidarRangeTrace,
            scanLidarRangeTrace
        ];

        const layout = {{
            scene: {{
                xaxis: {{
                    title: "X",
                    range: [-1, {size_x + 1}]
                }},
                yaxis: {{
                    title: "Y",
                    range: [-1, {size_y + 1}]
                }},
                zaxis: {{
                    title: "Height",
                    range: [-1, {size_z + 1}]
                }},
                aspectmode: "cube"
            }},
            margin: {{
                l: 0,
                r: 0,
                b: 0,
                t: 20
            }}
        }};

        Plotly.newPlot("plot", data, layout);
    </script>
</body>
</html>
"""

    output_html.write_text(html, encoding="utf-8")
    print(f"Wrote visualization to: {output_html}")
    print(f"Visualized lidarMaxRangeCm: {format_optional_cm(lidar_max_range_cm)}")


def main():
    if len(sys.argv) >= 2:
        base_path = sys.argv[1]
    else:
        base_path = "."

    create_visualization(base_path)


if __name__ == "__main__":
    main()