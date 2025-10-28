import matplotlib.pyplot as plt
import numpy as np
from matplotlib.widgets import Slider

L = [3.0, 2.5, 1.5]
theta_deg = [30.0, 45.0, 30.0]
ee_text = None  
last_text_pos = {'x': 0, 'y': 0, 'ha': 'center', 'va': 'bottom'} 

def get_parameters():
    return [L, theta_deg]

def _normalize_angle(a):
    return (a + np.pi) % (2 * np.pi) - np.pi

def _shortest_arc(start, end):
    start = _normalize_angle(start)
    end = _normalize_angle(end)
    delta = _normalize_angle(end - start)
    return start, start + delta

def draw_angle_arc(ax, base, a_ref, a_link, radius, col, txt):
    start, end = _shortest_arc(a_ref, a_link)
    arc = np.linspace(start, end, 80)
    x = base[0] + radius * np.cos(arc)
    y = base[1] + radius * np.sin(arc)
    line, = ax.plot(x, y, '--', color=col, linewidth=2, alpha=0.7, clip_on=True)
    
    mid = (start + end) / 2
    r_lab = radius + 0.25
    lx = base[0] + r_lab * np.cos(mid)
    ly = base[1] + r_lab * np.sin(mid)

    v_link = np.array([np.cos(a_link), np.sin(a_link)])
    proj = np.dot([lx - base[0], ly - base[1]], v_link)
    if proj > 0.1:
        r_lab += 0.35
        lx = base[0] + r_lab * np.cos(mid)
        ly = base[1] + r_lab * np.sin(mid)

    ha = 'center'
    if np.cos(mid) > 0.6: ha = 'left'
    if np.cos(mid) < -0.6: ha = 'right'

    text = ax.text(lx, ly, txt, fontsize=12, color=col, fontweight='500',
                   ha=ha, va='center', clip_on=True)
    return line, text

def draw_ref_extension(ax, base, ang, length=1.5, col='gray'):
    xf = base[0] + length * np.cos(ang)
    yf = base[1] + length * np.sin(ang)
    xb = base[0] - 0.4 * np.cos(ang)
    yb = base[1] - 0.4 * np.sin(ang)
    return ax.plot([xb, xf], [yb, yf], ':', color=col, linewidth=1.5, alpha=0.5, clip_on=True)[0]

def find_best_label_position(ee_pos, link_end_pos, link_angle):
    """
    Find the best position for the end-effector label to avoid overlap.
    Returns offset (dx, dy) and text alignment (ha, va).
    """
    perp_angle_up = link_angle + np.pi / 2
    perp_angle_down = link_angle - np.pi / 2

    offsets = [
        (1.2 * np.cos(perp_angle_up), 1.2 * np.sin(perp_angle_up), 'center', 'bottom'),
        (1.2 * np.cos(perp_angle_down), 1.2 * np.sin(perp_angle_down), 'center', 'top'),
        (1.2 * np.cos(link_angle), 1.2 * np.sin(link_angle), 'left', 'center'),
        (-1.2 * np.cos(link_angle), -1.2 * np.sin(link_angle), 'right', 'center'),
    ]
    
    dist_to_base = np.linalg.norm(ee_pos - link_end_pos)

    if dist_to_base > 1.0:
        if abs(np.cos(link_angle)) > 0.5:
            return offsets[0]
        else:
            if np.sin(link_angle) > 0:
                return offsets[2]
            else:
                return offsets[3]
    else:
        return offsets[1]

def smooth_lerp(current, target, factor=0.3):
    """Smooth interpolation between current and target values"""
    return current + (target - current) * factor

def draw_robot():
    global L, theta_deg, ee_text, last_text_pos

    fig = plt.figure(figsize=(13, 9), facecolor='#f8f9fa')
    ax = fig.add_subplot(111, facecolor='#ffffff')
    plt.subplots_adjust(left=0.08, right=0.82, bottom=0.08, top=0.92)
    ax.set_clip_on(True)

    ax.set_aspect('equal')
    ax.set_xlim(-10, 10)
    ax.set_ylim(0, 10)
    ax.set_xticks(np.arange(-10, 11, 1))
    ax.set_yticks(np.arange(0, 11, 1))
    ax.grid(True, alpha=0.15, linewidth=0.5, linestyle='-', color='#cccccc')
    ax.set_axisbelow(True)

    ax.set_xlabel('X-axis (ground)', fontsize=12, fontweight='500', color='#2c3e50')
    ax.set_ylabel('Y-axis', fontsize=12, fontweight='500', color='#2c3e50')
    ax.set_title('3R Planar Robot', fontsize=15, pad=20, 
                 fontweight='600', color='#2c3e50')

    ax.axhline(0, color='#34495e', lw=2, zorder=1)
    ax.axvline(0, color='#95a5a6', lw=0.8, alpha=0.5, zorder=1)
    ax.plot(0, 0, 'o', markersize=10, color='#2c3e50', 
            markeredgecolor='#1a252f', markeredgewidth=2, label='Base', zorder=5)

    colors = ['#1f77b4', '#d62728', '#2ca02c']

    link_lines = []
    ee_marker = None
    arc_lines = []
    arc_texts = []
    ref_lines = []

    def fk():
        th = np.radians(theta_deg)
        pts = [(0.0, 0.0)]
        prev = [0.0]
        for i in range(3):
            cur = prev[-1] + th[i]
            prev.append(cur)
            x = pts[-1][0] + L[i] * np.cos(cur)
            y = pts[-1][1] + L[i] * np.sin(cur)
            pts.append((x, y))
        return np.array(pts), prev[:-1], prev[1:]

    def update_plot():
        global ee_text, last_text_pos
        nonlocal ee_marker

        for line in link_lines + arc_lines + ref_lines:
            line.remove()
        for text in arc_texts:
            text.remove()
        if ee_marker:
            ee_marker.remove()
        if ee_text:
            ee_text.remove()

        link_lines.clear()
        arc_lines.clear()
        arc_texts.clear()
        ref_lines.clear()

        pts, prev_dirs, next_dirs = fk()

        for i in range(3):
            x0, y0 = pts[i]
            x1, y1 = pts[i+1]
            line, = ax.plot([x0, x1], [y0, y1], '-o', color=colors[i], lw=3.5, 
                           markersize=7, markeredgecolor='white', markeredgewidth=1.5,
                           label=f'Link {i+1}', zorder=3, clip_on=True)
            link_lines.append(line)

        ee_marker, = ax.plot(pts[-1][0], pts[-1][1], 'o', color='#000000', markersize=12,
                             markeredgecolor='#ffffff', markeredgewidth=2.5, 
                             label='End-Effector', zorder=4, clip_on=True)

        from kinematics.forward_kinematics import compute_fk
        x, y = compute_fk(get_parameters())
        
        last_link_angle = next_dirs[-1]

        dx, dy, ha, va = find_best_label_position(
            np.array([pts[-1][0], pts[-1][1]]),
            np.array([pts[-2][0], pts[-2][1]]),
            last_link_angle
        )

        target_x = pts[-1][0] + dx
        target_y = pts[-1][1] + dy

        if last_text_pos['ha'] == ha and last_text_pos['va'] == va:
            text_x = smooth_lerp(last_text_pos['x'], target_x, 0.35)
            text_y = smooth_lerp(last_text_pos['y'], target_y, 0.35)
        else:
            text_x = target_x
            text_y = target_y

        last_text_pos['x'] = text_x
        last_text_pos['y'] = text_y
        last_text_pos['ha'] = ha
        last_text_pos['va'] = va

        ee_text = ax.text(
            text_x, text_y,
            f"({x:.3f}, {y:.3f})",
            fontsize=10, color='#000000', fontweight='normal',
            ha=ha, va=va, fontfamily='monospace',
            bbox=dict(facecolor='white', alpha=0.95, edgecolor='#000000', 
                     pad=3, linewidth=1.2, boxstyle='round,pad=0.4'),
            zorder=10,
            clip_on=True 
        )

        arc_radii = [1.6, 1.4, 1.2]
        for j in range(3):
            base = pts[j]
            ref_line = draw_ref_extension(ax, base, prev_dirs[j], 2.2, '#95a5a6')
            link_line = draw_ref_extension(ax, base, next_dirs[j], 1.3, colors[j])
            arc_line, arc_text = draw_angle_arc(ax, base, prev_dirs[j], next_dirs[j],
                                                arc_radii[j], colors[j], f"$\\theta_{{{j+1}}}$")
            ref_lines.extend([ref_line, link_line])
            arc_lines.append(arc_line)
            arc_texts.append(arc_text)

        if not hasattr(update_plot, "legend"):
            ax.legend(loc='upper left', fontsize=10, frameon=True, fancybox=True, 
                     shadow=True, framealpha=0.95, edgecolor='#bdc3c7')
            update_plot.legend = True

        fig.canvas.draw_idle()

    slider_width = 0.08
    slider_height = 0.015
    start_x = 0.87
    start_y = 0.82

    sliders = []
    names = ['L₁', 'L₂', 'L₃', 'θ₁', 'θ₂', 'θ₃']
    init = L + theta_deg
    mins = [0.5, 0.5, 0.5, -180, -180, -180]
    maxs = [8.0, 8.0, 8.0,  180,  180,  180]

    for i, (name, val, mn, mx) in enumerate(zip(names, init, mins, maxs)):
        y_pos = start_y - i * (slider_height + 0.02)
        ax_slider = fig.add_axes([start_x, y_pos, slider_width, slider_height])
        
        valfmt = '%.2f' if i < 3 else '%.1f'  
        
        slider = Slider(
            ax_slider, name, mn, mx,
            valinit=val,
            valfmt=valfmt,
            facecolor='none'
        )
        
        slider.label.set_fontsize(10)
        slider.label.set_fontweight('500')
        slider.label.set_color('#2c3e50')
        slider.valtext.set_fontsize(9)
        slider.valtext.set_fontweight('normal')
        slider.valtext.set_color('#2c3e50')
        slider.poly.set_alpha(0.4)
        slider.track.set_linewidth(2)
        
        sliders.append(slider)

    def on_change(val):
        L[:] = [sliders[i].val for i in range(3)]
        theta_deg[:] = [sliders[i].val for i in range(3, 6)]
        update_plot()

    for s in sliders:
        s.on_changed(on_change)

    update_plot()
    plt.show()