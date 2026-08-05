# MuJoCo Wire Modeling — Variable Stiffness Suggestions

> Based on a full review of official MuJoCo documentation:  
> **Modeling** (pp. 1–40) · **Extensions** (all 15 pp.) · **XML Reference** (pp. 1–35) · **Playground Technical Report (2025)**

---

## 1. Problem Statement

Your model **ParallelWires** uses the `mujoco.elasticity.cable` plugin for two cables
(**Schaltbet_A** and **Schaltbet_B**) with 41 vertices (40 segments) each. The current config applies the same
`bend=8e7` and `twist=1e7` uniformly across all segments. You need **different stiffness values
at cable ends versus the middle** — e.g. stiffer near the gripper attachment points,
more flexible in the centre.

---

## 2. Why the Plugin Cannot Do This Natively

After reading the full MuJoCo **Extensions** documentation, the cable plugin architecture is:

- The `mujoco.elasticity.cable` plugin is a **passive force plugin** — it runs once per step over all elements of **one** composite instance.
- Its config keys (`bend`, `twist`, `vmax`) are set **once per plugin instance** (one `<composite>` = one config).
- There is **no per-element or per-body config mechanism** — the plugin reads a single stiffness value and applies it to every segment.
- Modeling doc (p. 36) confirms: *"stiffness and inertia properties are computed directly from the given parameters and the shape of the cross section"* — this is a global, per-composite operation.

> ⚠️ **Hard limit:** There is no attribute in `mujoco.elasticity.cable` to assign different `bend` or `twist` values to individual segments. This is an architectural limit of the current plugin design.

---

## 3. Four Approaches — Ranked by Ease

---

### ✅ Approach 1 (Recommended): Split Cable into Stiffness Zones

Divide each 41-vertex cable into **2–3 sub-composites**, each with its own `<plugin>` config and different
`bend`/`twist` values. Connect them at junction bodies with `<equality><weld>`.
**No code required — pure MJCF.**

#### How it works

Your cable has vertices from x = −1.0 to x = 1.0 in steps of 0.05 (41 vertices = 40 segments).

Example 3-zone split:
| Zone | Prefix | Vertices | Segments | bend | twist |
|------|--------|----------|----------|------|-------|
| Left end | `Sch_A_L` | 0 – 9 (x = −1.0 to −0.55) | 9 | `2e8` (stiffer) | `5e7` |
| Middle | `Sch_A_M` | 9 – 31 (x = −0.55 to 0.55) | 22 | `8e7` (current) | `1e7` |
| Right end | `Sch_A_R` | 31 – 40 (x = 0.55 to 1.0) | 9 | `2e8` (stiffer) | `5e7` |

**Junction rule:** the last vertex of zone N = first vertex of zone N+1 (shared position).

#### Example XML (Schaltbet_A)

```xml
<!-- Zone 1: stiff left end (10 vertices = 9 segments) -->
<composite prefix="Sch_A_L" type="cable" count="10 1 1"
           size="0.005 0.005 0.005" offset="0 0.5 1.1">
  <plugin plugin="mujoco.elasticity.cable">
    <config key="twist" value="5e7"/>
    <config key="bend"  value="2e8"/>   <!-- stiffer at ends -->
    <config key="vmax"  value="0.05"/>
  </plugin>
  <joint kind="main" damping="0.15" armature="0.01"/>
  <geom type="capsule" size=".005" rgba=".8 .2 .1 1"/>
  <vertex>
    -1.00 0.1 0  -0.95 0.1 0  -0.90 0.1 0  -0.85 0.1 0  -0.80 0.1 0
    -0.75 0.1 0  -0.70 0.1 0  -0.65 0.1 0  -0.60 0.1 0  -0.55 0.1 0
  </vertex>
</composite>

<!-- Zone 2: flexible middle (23 vertices = 22 segments) -->
<composite prefix="Sch_A_M" type="cable" count="23 1 1"
           size="0.005 0.005 0.005" offset="0 0.5 1.1">
  <plugin plugin="mujoco.elasticity.cable">
    <config key="twist" value="1e7"/>
    <config key="bend"  value="8e7"/>   <!-- flexible middle (your current value) -->
    <config key="vmax"  value="0.05"/>
  </plugin>
  <joint kind="main" damping="0.15" armature="0.01"/>
  <geom type="capsule" size=".005" rgba=".8 .2 .1 1"/>
  <vertex>
    -0.55 0.1 0  -0.50 0.1 0  -0.45 0.1 0  -0.40 0.1 0  -0.35 0.1 0
    -0.30 0.1 0  -0.25 0.1 0  -0.20 0.1 0  -0.15 0.1 0  -0.10 0.1 0
    -0.05 0.1 0   0.00 0.1 0   0.05 0.1 0   0.10 0.1 0   0.15 0.1 0
     0.20 0.1 0   0.25 0.1 0   0.30 0.1 0   0.35 0.1 0   0.40 0.1 0
     0.45 0.1 0   0.50 0.1 0   0.55 0.1 0
  </vertex>
</composite>

<!-- Zone 3: stiff right end (10 vertices = 9 segments) -->
<composite prefix="Sch_A_R" type="cable" count="10 1 1"
           size="0.005 0.005 0.005" offset="0 0.5 1.1">
  <plugin plugin="mujoco.elasticity.cable">
    <config key="twist" value="5e7"/>
    <config key="bend"  value="2e8"/>   <!-- stiffer at ends -->
    <config key="vmax"  value="0.05"/>
  </plugin>
  <joint kind="main" damping="0.15" armature="0.01"/>
  <geom type="capsule" size=".005" rgba=".8 .2 .1 1"/>
  <vertex>
    0.55 0.1 0  0.60 0.1 0  0.65 0.1 0  0.70 0.1 0  0.75 0.1 0
    0.80 0.1 0  0.85 0.1 0  0.90 0.1 0  0.95 0.1 0  1.00 0.1 0
  </vertex>
</composite>

<!-- Junction welds: connect adjacent zone endpoints -->
<equality>
  <weld body1="Sch_A_LB_last"  body2="Sch_A_MB_first"
        solimp="0.999 0.9999 0.001" solref="0.004 1"/>
  <weld body1="Sch_A_MB_last"  body2="Sch_A_RB_first"
        solimp="0.999 0.9999 0.001" solref="0.004 1"/>
</equality>
```

Do the same for **Schaltbet_B** (same x-vertices, change y from 0.1 to 0.0):
```
Sch_B_L, Sch_B_M, Sch_B_R
```

#### MuJoCo composite body naming (from official docs)

| Pattern | Meaning |
|---------|---------|
| `{prefix}B_first` | First body of the composite |
| `{prefix}B_last` | Last body of the composite |
| `{prefix}B_{n}` | Intermediate body at index n (0-based) |

#### Update your existing equality constraints

```xml
<!-- OLD (single cable): -->
<connect body1="MODULE_BOX_1" body2="Schaltbet_AB_first" .../>
<connect body1="Sch_A"        body2="Schaltbet_AB_last"  .../>

<!-- NEW (split cable): -->
<connect body1="MODULE_BOX_1" body2="Sch_A_LB_first" .../>   <!-- left endpoint -->
<connect body1="Sch_A"        body2="Sch_A_RB_last"  .../>   <!-- right endpoint -->
```

> 💡 The junction vertex must be **shared** (same coordinates at both ends). This ensures the bodies start co-located before the weld constraint activates.

#### Pros / Cons

**Pros:**
- Pure MJCF — no code, no build toolchain
- Per-zone independent `bend` and `twist` stiffness
- Compatible with existing gripper equality constraints
- Same DOF count as current model (composite ball joints)
- Can have any number of zones

**Cons:**
- Stiffness is step-wise between zones, not a smooth gradient
- More verbose XML — recommend Python to generate vertex lists (see §5)
- Junction welds add minor solver overhead

---

### Approach 2: Rigid Body Chain with Per-Joint Native Stiffness

Replace the composite with an explicit chain of `<body>` elements connected by `<joint type="hinge">` with
MuJoCo's native **`stiffness`** attribute. Gives true **per-segment stiffness control** without any plugin.

#### How it works

Each segment = 1 body with 3 hinge joints:
- `axis="0 1 0"` — bending in XZ plane (`stiffness` = bend stiffness)
- `axis="0 0 1"` — bending in XY plane (`stiffness` = bend stiffness)
- `axis="1 0 0"` — twist (`stiffness` = twist stiffness)

MuJoCo native spring force: **F = −stiffness × (q − q₀)**

```xml
<body name="W_seg_00" pos="-1.0 0.1 1.1">
  <joint name="W_00_bendY" type="hinge" axis="0 1 0" stiffness="160000" damping="0.1"/>
  <joint name="W_00_bendZ" type="hinge" axis="0 0 1" stiffness="160000" damping="0.1"/>
  <joint name="W_00_twist" type="hinge" axis="1 0 0" stiffness="20000"  damping="0.1"/>
  <geom type="capsule" fromto="0 0 0  0.05 0 0" size="0.005"/>
  <body name="W_seg_01" pos="0.05 0 0">
    <!-- segments 1–9 same stiffness (stiff end zone) -->
    ...
    <body name="W_seg_10" pos="0.05 0 0">
      <!-- segments 10–30: flexible middle -->
      <joint name="W_10_bendY" type="hinge" axis="0 1 0" stiffness="80000" damping="0.1"/>
      ...
    </body>
  </body>
</body>
```

**Stiffness profile:**
| Segments | Zone | bend stiffness | twist stiffness |
|----------|------|----------------|-----------------|
| 0 – 9 | Left end (gripper) | 160000 | 20000 |
| 10 – 30 | Flexible middle | 80000 | 10000 |
| 31 – 39 | Right end (gripper) | 160000 | 20000 |

> 💡 Use a Python script to generate the nested `<body>` XML. 40 bodies × 3 joints = 120 lines.

**Pros:** True per-segment control · No plugin · Uses stable, native MuJoCo spring mechanics

**Cons:** 3 DOFs/segment × 40 segments = **120 DOFs per cable** (3× more than composite ball joints) · Cable is not inextensible by default · Very verbose XML

---

### Approach 3: Flexcomp 1D (Modern Deformable Object)

MuJoCo 3.0 introduced `<flexcomp>` for true deformable objects. A 1D flex is a chain of capsules with a
continuum hyperelastic model (Saint Venant-Kirchhoff). Stiffness is set via **Young's modulus** (`young`) and
**Poisson's ratio** (`poisson`).

```xml
<flexcomp type="grid" count="41 1 1" spacing="0.05 0.05 0.05"
          pos="-1.0 0.1 1.1" radius="0.005" dim="1" mass="0.1"
          rgba=".8 .2 .1 1" name="Sch_A_flex">
  <contact condim="3" solref="0.01 1" solimp=".95 .99 .0001" selfcollide="none"/>
  <edge damping="1"/>
  <elasticity poisson="0.2" young="5e4"/>  <!-- adjust young for stiffness -->
</flexcomp>
```

For variable stiffness: split into multiple `<flexcomp>` elements with different `young` values,
exactly as in Approach 1. Connect with `<equality><weld>` at junction bodies.

> ⚠️ Flexcomp is newer and less battle-tested for cable simulation. Still requires zone splitting for variable stiffness. Not compatible with MJX (GPU simulation) if that is a future goal.

**Pros:** Physics-accurate continuum model · Supports stretching · Better self-collision

**Cons:** Still needs zone splitting for variable stiffness · Less documentation · Not MJX-compatible

---

### Approach 4: Custom C++ Plugin with Per-Body Stiffness

Write a custom passive force plugin that reads per-body stiffness from `mjModel.body_user`.
This gives **true per-segment stiffness** with no junction constraints.

**Implementation outline:**
1. In MJCF: `<size nuser_body="2"/>` — allocates 2 user floats per body (`bend`, `twist`).
2. On each cable body: `<body user="160000000 50000000">` for stiff zones, `<body user="80000000 10000000">` for flexible.
3. In C++ plugin: read `mjModel->body_user[id*2]` and `[id*2+1]` per body instead of a global config value.
4. Compile as shared library, load via `<extension><plugin plugin="my_cable_plugin"/></extension>`.

> ⚠️ Requires C++ development and the MuJoCo plugin API. Longest implementation time but maximum flexibility and cleanest MJCF.

**Pros:** True per-segment smooth stiffness · No junction constraints · Keeps the geometrically exact Bishop-frame model

**Cons:** C++ required · Must compile and link as shared library · Significant development effort

---

## 4. Comparison Table

| Approach | Variable Stiffness | Code Required | Complexity | DOF Impact | Recommended |
|---|---|---|---|---|---|
| 1. Split Composites | Zone-wise | None (MJCF only) | Low | No change | ⭐⭐⭐⭐⭐ |
| 2. Hinge Chain | Per-segment | None (MJCF only) | Medium | 3× more DOFs | ⭐⭐⭐☆☆ |
| 3. Flexcomp 1D | Zone-wise | None (MJCF only) | Medium | Similar | ⭐⭐⭐☆☆ |
| 4. Custom Plugin | Per-segment | C++ required | High | No change | ⭐⭐⭐⭐⭐ |

---

## 5. Concrete Action Plan (Approach 1)

### Step 1 — Decide your stiffness profile

Choose how many zones and what stiffness values:
- **3-zone profile** is recommended for a cable gripped at both ends.
- Suggested: end zones cover 8–10 segments each; middle covers remaining 22 segments.
- Suggested stiffness: ends at `bend=1.6e8` (2× current), middle at `bend=8e7` (unchanged).

### Step 2 — Generate vertex lists with Python

```python
import numpy as np

# Generate vertex string for a zone of Schaltbet_A (y=0.1)
def zone_vertices(x_start, x_end, n, y=0.1):
    xs = np.round(np.linspace(x_start, x_end, n), 4)
    return "  ".join(f"{x} {y} 0" for x in xs)

# Left end zone: x from -1.0 to -0.55, 10 vertices
print(zone_vertices(-1.0, -0.55, 10))
# Middle zone: x from -0.55 to 0.55, 23 vertices
print(zone_vertices(-0.55, 0.55, 23))
# Right end zone: x from 0.55 to 1.0, 10 vertices
print(zone_vertices(0.55, 1.0, 10))
```

For **Schaltbet_B**: same x-values, use `y=0.0` instead of `y=0.1`.

### Step 3 — Rename and restructure your MJCF

1. Replace `Schaltbet_A` composite with three composites: `Sch_A_L`, `Sch_A_M`, `Sch_A_R`
2. Replace `Schaltbet_B` composite with three composites: `Sch_B_L`, `Sch_B_M`, `Sch_B_R`
3. Add junction `<weld>` constraints for each pair of adjacent zones
4. Update existing endpoint constraints to reference new `_B_first`/`_B_last` names

### Step 4 — Debug body names

After loading, verify generated body names match your equality constraints:

```python
import mujoco
model = mujoco.MjModel.from_xml_path("ParallelWires.xml")
for i in range(model.nbody):
    name = mujoco.mj_id2name(model, mujoco.mjtObj.mjOBJ_BODY, i)
    if name and 'Sch' in name:
        print(i, name)
```

### Step 5 — Tune junction welds

If the junction weld causes solver oscillation or instability:
- Tighten `solimp`: try `"0.9999 0.99999 0.0001"`
- Reduce `solref` time constant: try `"0.002 1"` (= 1× timestep)
- Reduce timestep to `0.001` if instability persists

---

## 6. General Simulation Quality Tips

**Solver stability:**
- Keep the Newton solver (default). It handles cable constraints best (confirmed in Modeling doc).
- Junction welds are equality constraints — they are prioritised in the Newton solve automatically.

**Verifying in the MuJoCo viewer:**
- Press `Ctrl+G` to toggle group visibility — enable Group 3 to see internal capsule geoms.
- Right-click a body → Info to see its name, which helps verify naming matches equality refs.
- Use the Pause button and step through frames to see cable dynamics frame-by-frame.

**MJCF modularity:**
- Use `<include file="cable_A.xml"/>` to keep cable definitions in separate files.
- MuJoCo's `<include>` merges multiple `<worldbody>` elements — this makes the 3-zone structure clean.
- Element names must be **globally unique** across all included files (names are type-scoped but prefix collisions will error).

**Timestep recommendation:**
- Current: `0.002 s` — reasonable for composite cables.
- After zone splitting: start with `0.002 s`, reduce to `0.001 s` only if junction welds cause instability.

---

*Document generated from: MuJoCo Modeling (pp. 1–40), Extensions (all 15 pp.), XML Reference (pp. 1–35), Playground Technical Report 2025.*
