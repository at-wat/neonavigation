#!/usr/bin/env python3

import json
import subprocess

try:
    import matplotlib.pyplot as plt
    from matplotlib.widgets import CheckButtons, RadioButtons, Slider
except ImportError as e:
    if "tkinter" in str(e) or "tk" in str(e):
        print("\n[ERROR] Python 'tkinter' module is missing.")
        print("Please install it using the following command:")
        print("    sudo apt-get update && sudo apt-get install python3-tk\n")
    else:
        print(f"\n[ERROR] Failed to import matplotlib: {e}\n")
    exit(1)
import numpy as np


def get_primitives(
    m_type, range_val, lin_res, ang_res, min_curve_radius, bezier_cp_dist, cp_mode
):
    cmd = [
        "rosrun",
        "planner_cspace",
        "primitive_exporter",
        str(m_type),
        str(range_val),
        str(lin_res),
        str(ang_res),
        str(min_curve_radius),
        str(bezier_cp_dist),
        str(cp_mode),
    ]
    try:
        result = subprocess.run(cmd, capture_output=True, text=True, check=True)
        return json.loads(result.stdout)
    except Exception as e:
        print(f"Error running exporter: {e}")
        return None


fig, ax = plt.subplots(figsize=(12, 10))
plt.subplots_adjust(bottom=0.35, left=0.25)

# Initial parameters
init_range = 8
init_lin_res = 0.1
init_ang_res = 0.392699  # PI/8
init_min_curve = 0.4
init_bezier_cp = 0.5
init_type = 0  # DEFAULT
init_start_yaw = 0
init_end_yaw = -1
init_prim_idx = -1  # -1 means show all
init_show_labels = True

ax_range = plt.axes([0.3, 0.22, 0.5, 0.02])
ax_min_curve = plt.axes([0.3, 0.18, 0.5, 0.02])
ax_bezier_cp = plt.axes([0.3, 0.14, 0.5, 0.02])
ax_start_yaw = plt.axes([0.3, 0.10, 0.5, 0.02])
ax_end_yaw = plt.axes([0.3, 0.06, 0.5, 0.02])
ax_prim_idx = plt.axes([0.3, 0.02, 0.5, 0.02])
ax_type = plt.axes([0.02, 0.60, 0.12, 0.08])
ax_ang_res_sel = plt.axes([0.02, 0.50, 0.12, 0.08])
ax_cp_mode = plt.axes([0.02, 0.40, 0.12, 0.08])
ax_grid_size_sel = plt.axes([0.02, 0.24, 0.12, 0.12])
ax_label_mode = plt.axes([0.02, 0.08, 0.12, 0.12])

s_range = Slider(ax_range, "Range", 1, 20, valinit=init_range, valstep=0.1)
s_min_curve = Slider(
    ax_min_curve, "Min Curve", 0.1, 2.0, valinit=init_min_curve, valstep=0.01
)
s_bezier_cp = Slider(
    ax_bezier_cp, "Bezier CP", 0.0, 1.5, valinit=init_bezier_cp, valstep=0.01
)
s_start_yaw = Slider(
    ax_start_yaw, "Start Yaw Idx", 0, 15, valinit=init_start_yaw, valstep=1
)
s_end_yaw = Slider(ax_end_yaw, "End Yaw Idx", -1, 15, valinit=init_end_yaw, valstep=1)
s_prim_idx = Slider(ax_prim_idx, "Prim Idx", -1, 100, valinit=init_prim_idx, valstep=1)
r_type = RadioButtons(ax_type, ("DEFAULT", "BEZIER"), active=init_type)
r_ang_res = RadioButtons(ax_ang_res_sel, ("22.5 deg", "45 deg"), active=0)
r_cp_mode = RadioButtons(ax_cp_mode, ("FIXED", "OPTIMIZE"), active=1)
r_grid_size = RadioButtons(
    ax_grid_size_sel, ("0.025", "0.05", "0.1", "0.2"), active=2
)  # Default 0.1
r_labels = RadioButtons(ax_label_mode, ("None", "Original", "Filtered"), active=1)

current_data = None


def update(val):
    global current_data
    ax.clear()
    m_type = 0 if r_type.value_selected == "DEFAULT" else 1
    ang_res = (
        3.1415926535 / 8 if r_ang_res.value_selected == "22.5 deg" else 3.1415926535 / 4
    )
    cp_mode = 0 if r_cp_mode.value_selected == "FIXED" else 1
    lin_res = float(r_grid_size.value_selected)

    # Reload data only if core params changed
    # For now simplicity: always reload
    current_data = get_primitives(
        m_type,
        int(s_range.val),
        lin_res,
        ang_res,
        s_min_curve.val,
        s_bezier_cp.val,
        cp_mode,
    )

    if current_data:
        num_yaw = current_data["map_info"]["angle"]
        s_start_yaw.valmax = num_yaw - 1
        s_end_yaw.valmax = num_yaw - 1

        start_yaw_idx = int(s_start_yaw.val) % num_yaw
        target_end_yaw = int(s_end_yaw.val)

        all_prims = current_data["primitives"][start_yaw_idx]

        # Filter primitives by end yaw if needed
        filtered_prims = []
        for p_idx, p in enumerate(all_prims):
            if target_end_yaw == -1 or p["end"]["yaw"] == target_end_yaw:
                filtered_prims.append((p_idx, p))

        num_filtered = len(filtered_prims)
        # Dynamically update Prim Idx slider range
        if num_filtered > 0:
            s_prim_idx.valmax = num_filtered - 1
            s_prim_idx.ax.set_xlim(-1, s_prim_idx.valmax)
        else:
            s_prim_idx.valmax = -1
            s_prim_idx.ax.set_xlim(-1, 0)  # Set range to [-1, 0] if no primitives

        prim_sel_idx = int(s_prim_idx.val)

        label_mode = r_labels.value_selected

        end_yaw_str = "All" if target_end_yaw == -1 else str(target_end_yaw)

        if prim_sel_idx == -1:
            # Show all filtered primitives
            for f_idx, (orig_idx, p) in enumerate(filtered_prims):
                path = p["path"]
                if not path:
                    continue
                xs = [pt["x"] for pt in path]
                ys = [pt["y"] for pt in path]
                ax.plot(xs, ys, "-", alpha=0.3)
                # Mark end point
                ax.plot([xs[-1]], [ys[-1]], "x", color="red", markersize=4)
                if label_mode == "Original":
                    ax.text(xs[-1], ys[-1], str(orig_idx), fontsize=8, color="blue")
                elif label_mode == "Filtered":
                    ax.text(xs[-1], ys[-1], str(f_idx), fontsize=8, color="green")
            ax.set_title(
                f"All Primitives (Start Yaw Idx: {start_yaw_idx}, End Yaw Idx: {end_yaw_str})"
            )
        else:
            if prim_sel_idx < len(filtered_prims):
                orig_idx, p = filtered_prims[prim_sel_idx]
                path = p["path"]
                if not path:
                    ax.set_title(
                        f"Primitive {orig_idx} (Start Yaw Idx: {start_yaw_idx}) - [Empty Path/Rotation]"
                    )
                    ax.plot([0], [0], "ro")
                else:
                    xs = [pt["x"] for pt in path]
                    ys = [pt["y"] for pt in path]
                    yaws = [pt["yaw"] for pt in path]
                    ax.plot(xs, ys, "b-o", markersize=3)
                    # Orientation arrows
                    ax.quiver(
                        xs,
                        ys,
                        np.cos(yaws),
                        np.sin(yaws),
                        color="red",
                        scale=20,
                        width=0.005,
                    )
                    ex, ey, eyaw = xs[-1], ys[-1], yaws[-1]
                    ax.set_title(
                        f"Primitive {orig_idx} (Start Yaw Idx: {start_yaw_idx})\nEnd: x={ex:.2f}, y={ey:.2f}, yaw={eyaw:.2f}"
                    )

    ax.set_aspect("equal")
    ax.grid(True)
    fig.canvas.draw_idle()


s_range.on_changed(update)
s_min_curve.on_changed(update)
s_bezier_cp.on_changed(update)
s_start_yaw.on_changed(update)
s_end_yaw.on_changed(update)
s_prim_idx.on_changed(update)
r_type.on_clicked(update)
r_ang_res.on_clicked(update)
r_cp_mode.on_clicked(update)
r_grid_size.on_clicked(update)
r_labels.on_clicked(update)

update(None)
plt.show()
