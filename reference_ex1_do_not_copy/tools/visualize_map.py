#!/usr/bin/env python3

import sys
from pathlib import Path


CELL_LABELS = {
    -2: "OOB",
    -1: "?",
     0: ".",
     1: "#",
}

CELL_COLORS = {
    -2: "#cce5ff",  # out of bounds
    -1: "#d9d9d9",  # unknown
     0: "#ffffff",  # free
     1: "#ff9999",  # occupied
}


def read_map(path):
    """
    Reads a map file in our project format:

    sizeX sizeY sizeZ
    then sizeX * sizeY * sizeZ integer cell values.

    Returns:
        size_x, size_y, size_z, cells

    cells[height][y][x]
    """
    path = Path(path)

    with path.open("r", encoding="utf-8") as file:
        tokens = file.read().split()

    if len(tokens) < 3:
        raise ValueError(f"{path}: file does not contain sizeX sizeY sizeZ")

    size_x = int(tokens[0])
    size_y = int(tokens[1])
    size_z = int(tokens[2])

    expected_cells = size_x * size_y * size_z
    cell_tokens = tokens[3:]

    if len(cell_tokens) < expected_cells:
        print(
            f"Warning: {path} has only {len(cell_tokens)} cells, "
            f"expected {expected_cells}. Missing cells will be -1."
        )

    if len(cell_tokens) > expected_cells:
        print(
            f"Warning: {path} has {len(cell_tokens)} cells, "
            f"expected {expected_cells}. Extra cells will be ignored."
        )

    values = []

    for i in range(expected_cells):
        if i < len(cell_tokens):
            try:
                values.append(int(cell_tokens[i]))
            except ValueError:
                values.append(-1)
        else:
            values.append(-1)

    cells = []
    index = 0

    for height in range(size_z):
        layer = []

        for y in range(size_y):
            row = []

            for x in range(size_x):
                row.append(values[index])
                index += 1

            layer.append(row)

        cells.append(layer)

    return size_x, size_y, size_z, cells


def cell_to_html(value):
    label = CELL_LABELS.get(value, str(value))
    color = CELL_COLORS.get(value, "#ffff99")

    return (
        f'<td style="background:{color};">'
        f'{label}'
        f'</td>'
    )


def map_to_html(title, size_x, size_y, size_z, cells):
    html = [f"<h2>{title}</h2>"]
    html.append(f"<p>Size: {size_x} x {size_y} x {size_z}</p>")

    for height in range(size_z):
        html.append(f"<h3>Height layer {height}</h3>")
        html.append("<table>")

        for y in range(size_y):
            html.append("<tr>")

            for x in range(size_x):
                html.append(cell_to_html(cells[height][y][x]))

            html.append("</tr>")

        html.append("</table>")

    return "\n".join(html)


def create_html(input_map_path, output_map_path, html_path):
    input_size_x, input_size_y, input_size_z, input_cells = read_map(input_map_path)
    output_size_x, output_size_y, output_size_z, output_cells = read_map(output_map_path)

    html = f"""
<!DOCTYPE html>
<html>
<head>
    <meta charset="utf-8">
    <title>Drone Mapper Visualization</title>
    <style>
        body {{
            font-family: Arial, sans-serif;
            margin: 24px;
        }}

        table {{
            border-collapse: collapse;
            margin-bottom: 24px;
        }}

        td {{
            width: 28px;
            height: 28px;
            border: 1px solid #999;
            text-align: center;
            font-size: 13px;
            font-family: monospace;
        }}

        .legend td {{
            width: auto;
            padding: 6px 10px;
            font-family: Arial, sans-serif;
        }}

        .container {{
            display: flex;
            gap: 48px;
            align-items: flex-start;
        }}

        .map-block {{
            max-width: 50%;
        }}
    </style>
</head>
<body>
    <h1>Drone Mapper Visualization</h1>

    <h2>Legend</h2>
    <table class="legend">
        <tr><td style="background:#ffffff;">.</td><td>Free / empty cell</td></tr>
        <tr><td style="background:#ff9999;">#</td><td>Occupied cell</td></tr>
        <tr><td style="background:#d9d9d9;">?</td><td>Unknown / unmapped cell</td></tr>
        <tr><td style="background:#cce5ff;">OOB</td><td>Out of bounds</td></tr>
    </table>

    <div class="container">
        <div class="map-block">
            {map_to_html("Ground truth map_input.txt", input_size_x, input_size_y, input_size_z, input_cells)}
        </div>

        <div class="map-block">
            {map_to_html("Drone output map_output.txt", output_size_x, output_size_y, output_size_z, output_cells)}
        </div>
    </div>
</body>
</html>
"""

    Path(html_path).write_text(html, encoding="utf-8")
    print(f"Wrote visualization to: {html_path}")


def main():
    if len(sys.argv) < 2:
        base_path = Path(".")
    else:
        base_path = Path(sys.argv[1])

    input_map_path = base_path / "map_input.txt"
    output_map_path = base_path / "map_output.txt"
    html_path = base_path / "map_visualization.html"

    create_html(input_map_path, output_map_path, html_path)


if __name__ == "__main__":
    main()