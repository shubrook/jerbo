"""Fusion 360 script: parametric open-frame mic array mount.

Generates a flat cross (or 2-mic bar) with a mic pad at each arm tip and a
central hub for bolting to the pan/tilt bracket. Run via
Utilities -> Add-Ins -> Scripts and Add-Ins -> (+) -> pick this file.

Design intent:
- Open frame, thin arms: minimal surface near the mics to avoid
  reflections that smear the cross-correlation.
- Each pad has a center through-hole; the INMP441's sound inlet is the
  small hole in the PCB (bottom port), so mount the module with that hole
  centered over the pad hole, facing the sound. Zip-tie slots flank it.
- Bolt the hub to the bracket through rubber grommets or a TPU pad —
  servo vibration conducted through the frame correlates at zero delay
  and biases the solver toward center.

After printing, measure the actual port-to-port distances with calipers
and put them in MIC_POSITIONS in pi/tracker.py.

Tweak the numbers below, re-run, done. Measure your INMP441 breakout
first — board sizes vary between sellers.
"""
import traceback

import adsk.core
import adsk.fusion

# All dimensions in mm.
ARM_LENGTH = 120.0    # hub center to mic pad center; 318 mm overall at 150,
                      # rotate 45 deg on the bed if your printer is small
ARM_WIDTH = 8.0
THICKNESS = 5.0
CROSS = True          # False = horizontal 2-mic bar for the INMP441 pair

PAD_SIZE = 18.0       # square pad at each tip; >= your breakout board
PORT_HOLE_D = 6.0     # through-hole under the mic's sound inlet
SLOT_W, SLOT_L = 2.2, 4.5   # zip-tie slots, one each side of the pad
SLOT_OFFSET = 7.5     # slot center distance from pad center

HUB_DIAMETER = 36.0
HUB_BOLT_CIRCLE = 25.0
HUB_HOLE_D = 3.4      # M3 clearance


def cm(mm):
    return mm / 10.0  # Fusion's API works in centimeters


def run(context):
    ui = None
    try:
        app = adsk.core.Application.get()
        ui = app.userInterface
        design = adsk.fusion.Design.cast(app.activeProduct)
        root = design.rootComponent
        comp = root.occurrences.addNewComponent(
            adsk.core.Matrix3D.create()).component
        comp.name = 'mic_array_mount'

        tips = [(ARM_LENGTH, 0), (-ARM_LENGTH, 0)]
        if CROSS:
            tips += [(0, ARM_LENGTH), (0, -ARM_LENGTH)]

        def point(x, y):
            return adsk.core.Point3D.create(cm(x), cm(y), 0)

        def rect(sketch, cx, cy, w, h):
            sketch.sketchCurves.sketchLines.addTwoPointRectangle(
                point(cx - w / 2, cy - h / 2), point(cx + w / 2, cy + h / 2))

        # --- body: arms + pads + hub, one sketch, one extrude ---
        body_sk = comp.sketches.add(comp.xYConstructionPlane)
        rect(body_sk, 0, 0, 2 * ARM_LENGTH, ARM_WIDTH)
        if CROSS:
            rect(body_sk, 0, 0, ARM_WIDTH, 2 * ARM_LENGTH)
        for tx, ty in tips:
            rect(body_sk, tx, ty, PAD_SIZE, PAD_SIZE)
        body_sk.sketchCurves.sketchCircles.addByCenterRadius(
            point(0, 0), cm(HUB_DIAMETER / 2))

        profiles = adsk.core.ObjectCollection.create()
        for p in body_sk.profiles:
            profiles.add(p)
        comp.features.extrudeFeatures.addSimple(
            profiles, adsk.core.ValueInput.createByReal(cm(THICKNESS)),
            adsk.fusion.FeatureOperations.NewBodyFeatureOperation)

        # --- holes: port + zip-tie slots per pad, bolt circle on the hub ---
        hole_sk = comp.sketches.add(comp.xYConstructionPlane)
        circles = hole_sk.sketchCurves.sketchCircles
        for tx, ty in tips:
            circles.addByCenterRadius(point(tx, ty), cm(PORT_HOLE_D / 2))
            along_x = ty != 0  # slots straddle the arm direction
            for sign in (-1, 1):
                ox = sign * SLOT_OFFSET if along_x else 0
                oy = 0 if along_x else sign * SLOT_OFFSET
                w, h = (SLOT_W, SLOT_L) if along_x else (SLOT_L, SLOT_W)
                rect(hole_sk, tx + ox, ty + oy, w, h)
        for hx, hy in [(1, 1), (-1, 1), (-1, -1), (1, -1)]:
            r = HUB_BOLT_CIRCLE / 2 * 0.7071
            circles.addByCenterRadius(point(hx * r, hy * r), cm(HUB_HOLE_D / 2))

        cuts = adsk.core.ObjectCollection.create()
        for p in hole_sk.profiles:
            cuts.add(p)
        comp.features.extrudeFeatures.addSimple(
            cuts, adsk.core.ValueInput.createByReal(cm(THICKNESS)),
            adsk.fusion.FeatureOperations.CutFeatureOperation)

        ui.messageBox('mic_array_mount generated.\n'
                      'Pad centers are at +/-{} mm — after printing, '
                      'measure the real port-to-port distance and update '
                      'MIC_POSITIONS in pi/tracker.py.'.format(ARM_LENGTH))
    except Exception:
        if ui:
            ui.messageBox('Failed:\n{}'.format(traceback.format_exc()))
