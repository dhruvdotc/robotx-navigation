#!/usr/bin/env python3
"""Drive the scan-the-code light buoy through its colour sequence.

The RobotX "Scan the Code" buoy displays a repeating sequence of light colours
that the vehicle must read and report. The course worlds model the buoy as
static SDF with DARK LED panels; every --period seconds this script advances
red -> green -> blue by spawning a small emissive glow-panel model on the buoy
top and removing the previous one (/world/<world>/create + /remove, provided
by the UserCommands plugin every course world already loads).

Why entity swap instead of just recolouring the panel visuals: gz-sim Harmonic
ACCEPTS /world/<w>/visual_config material changes (the service returns true,
and the GUI would show them) but never applies them to the SENSOR render
scene -- verified empirically here with a 100 s hover over the buoy whose
"cycling" panel stayed red to the drone camera the whole time. Entity
create/remove propagates to sensor scenes reliably, so colour is delivered by
swapping a fresh model per phase. The new panel is spawned a hair above the
old one before the old is removed, so there is no dark flash mid-sequence.

Colour values mirror the gate buoys exactly (see the channel-gates comment in
robotx_uav_course.sdf): green is the spring-green emissive at OpenCV hue ~81
because pure green (hue 60) sits OUTSIDE the HSV detector's green range
(75-105), and blue targets hue ~114 to sit inside the detector's blue range
(100-130) while staying clear of green's upper bound. If these drift from the
gate materials, autolabelled training classes stop matching what the live
detector sees.

run_course.sh launches this automatically (it knows each course's buoy pose).
Manual use:
  python3 simulation/light_buoy_cycler.py --world robotx_uav_course --pose "50 0 0.76"
  python3 simulation/light_buoy_cycler.py --world course_2_search_field --pose "55 2 0.76"
"""

from __future__ import annotations

import argparse
import signal
import subprocess
import sys
import time

# (name, ambient, diffuse, emissive) per phase, as (r, g, b) -- alpha always 1.
# These are tuned against what the SENSOR actually renders, not colour theory
# (verified with a static observer camera over the buoy, one colour at a time):
#   red   -- the exact emissive the original led_top used; renders ~hue 0 with
#            a white-ish lighting wash, detected at conf 0.98 from nadir.
#   green -- verbatim gate-buoy spring green (rendered hue 86, inside 75-105).
#   blue  -- deliberately DARK pure blue. Two traps live here: (1) a bright
#            blue-with-green emissive renders ~hue 102, inside the green/blue
#            HSV overlap at 100-105, and classifies as green (green is checked
#            first; ties break by strict >). (2) a bright PURE blue renders at
#            the right hue (~114) but its GRAYSCALE (~58) sits too close to the
#            water's (~103) -- after resize/blur/JPEG the Canny stage finds no
#            edge, so the detector never even proposes it (blue-on-blue-water,
#            the same physics that makes blue the weakest class on real water).
#            Dark blue keeps hue ~114 AND drops grayscale to ~20, giving a
#            stronger edge than the green panel has.
COLOR_SEQUENCE: list[tuple[str, tuple, tuple, tuple]] = [
    ("red",   (0.5, 0.05, 0.05), (0.8, 0.08, 0.08), (1.0, 0.12, 0.12)),
    ("green", (0.0, 0.4, 0.2), (0.0, 0.6, 0.31), (0.0, 1.0, 0.51)),
    ("blue",  (0.0, 0.0, 0.3), (0.0, 0.0, 0.5), (0.0, 0.0, 0.65)),
]

# Nearly the full 0.5 m buoy top: a real scan-the-code light tower is the
# dominant feature of the buoy, and the larger blob keeps the HSV detector's
# colour-ratio and size gates comfortably satisfied from 10 m up.
PANEL_SIZE = "0.45 0.45 0.02"

_stop = False


def log(msg: str) -> None:
    print(f"[CYCLER {time.strftime('%H:%M:%S')}] {msg}", flush=True)


def _gz_service(service: str, reqtype: str, req: str, timeout_ms: int = 2000) -> bool:
    """Call a gz service; True iff it replied `data: true` within the timeout."""
    try:
        out = subprocess.run(
            ["gz", "service", "-s", service, "--reqtype", reqtype,
             "--reptype", "gz.msgs.Boolean", "--timeout", str(timeout_ms),
             "--req", req],
            capture_output=True, text=True, timeout=timeout_ms / 1000 + 2.0,
        )
        return "data: true" in out.stdout
    except (subprocess.TimeoutExpired, OSError):
        return False


def _fmt_rgba(rgb: tuple) -> str:
    r, g, b = rgb
    return f"{r} {g} {b} 1"


def build_panel_sdf(model_name: str, pose_xyz: str, ambient: tuple,
                    diffuse: tuple, emissive: tuple) -> str:
    return (
        '<?xml version="1.0"?>'
        '<sdf version="1.9">'
        f'<model name="{model_name}"><static>true</static>'
        f'<pose>{pose_xyz} 0 0 0</pose>'
        '<link name="link"><visual name="v">'
        f'<geometry><box><size>{PANEL_SIZE}</size></box></geometry>'
        f'<material><ambient>{_fmt_rgba(ambient)}</ambient>'
        f'<diffuse>{_fmt_rgba(diffuse)}</diffuse>'
        f'<emissive>{_fmt_rgba(emissive)}</emissive></material>'
        '</visual></link></model></sdf>'
    )


def spawn_panel(world: str, model_name: str, pose_xyz: str,
                ambient: tuple, diffuse: tuple, emissive: tuple) -> bool:
    sdf = build_panel_sdf(model_name, pose_xyz, ambient, diffuse, emissive)
    req = 'sdf: "{}"'.format(sdf.replace('"', '\\"'))
    return _gz_service(f"/world/{world}/create", "gz.msgs.EntityFactory", req)


def remove_model(world: str, model_name: str) -> bool:
    req = f'name: "{model_name}" type: MODEL'
    return _gz_service(f"/world/{world}/remove", "gz.msgs.Entity", req)


def _on_signal(signum, frame) -> None:  # noqa: ARG001 - signal handler shape
    global _stop
    _stop = True


def main() -> int:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--world", required=True,
                        help="Gazebo world name (e.g. robotx_uav_course).")
    parser.add_argument("--pose", required=True, metavar="'X Y Z'",
                        help="Glow-panel centre in world coords, just above the "
                             "buoy top (e.g. '50 0 0.76' for course 1).")
    parser.add_argument("--period", type=float, default=3.0,
                        help="Seconds each colour is displayed (default 3.0).")
    args = parser.parse_args()

    x, y, z = (float(v) for v in args.pose.split())

    signal.signal(signal.SIGTERM, _on_signal)
    signal.signal(signal.SIGINT, _on_signal)

    log(f"cycling light_buoy glow panel in world '{args.world}' at ({x} {y} {z}) "
        f"every {args.period:.1f}s "
        f"(sequence: {' -> '.join(c[0] for c in COLOR_SEQUENCE)})")

    prev_name: str | None = None
    consecutive_failures = 0
    phase = 0

    while not _stop:
        name, ambient, diffuse, emissive = COLOR_SEQUENCE[phase % len(COLOR_SEQUENCE)]
        model_name = f"led_glow_{phase}"
        # Alternate the new panel a hair above/below the previous one so the two
        # never z-fight during the brief overlap before the old one is removed.
        z_offset = 0.004 if phase % 2 else 0.0
        pose_xyz = f"{x} {y} {z + z_offset}"

        spawned = spawn_panel(args.world, model_name, pose_xyz, ambient, diffuse, emissive)
        removed = True
        if spawned and prev_name is not None:
            # A failed remove leaves a stale panel COVERING the new one (the
            # topmost panel wins visually), so surface it in the log.
            removed = remove_model(args.world, prev_name)
        if spawned:
            prev_name = model_name
            consecutive_failures = 0
            log(f"light_buoy -> {name}" + ("" if removed else "  (WARN: stale panel remove FAILED)"))
        else:
            consecutive_failures += 1
            if consecutive_failures == 10:
                log(f"WARN: 10 consecutive spawn failures in world "
                    f"'{args.world}' -- is the world name right and Gazebo up?")

        phase += 1
        # Sleep in small steps so SIGTERM lands promptly.
        deadline = time.time() + args.period
        while not _stop and time.time() < deadline:
            time.sleep(0.1)

    # Leave the buoy dark (its authentic between-sequences state) on exit.
    if prev_name is not None:
        remove_model(args.world, prev_name)
    log("stopped.")
    return 0


if __name__ == "__main__":
    sys.exit(main())
