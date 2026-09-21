
import os
import jinja2
import math

SCRIPT_DIR = os.path.dirname(os.path.realpath(__file__))

MM_TO_M = 0.001
def scale_mm(xyz: list[float]) -> str:
    return " ".join(str(v * MM_TO_M) for v in xyz)

# Datasheet
MASS = 0.566
MASS_W_PAYLOAD = 0.930 
NROT = 4
KV = 1500
VBAT_MAX = 16.8
VBAT_NOM = 14.8
T2W = 3.0
RPM_MAX = KV * VBAT_MAX * 0.85

# Measured Quantities
# Drone BBOX
SIZE_X = 217 * MM_TO_M
SIZE_Y = 287 * MM_TO_M
SIZE_Z = 129 * MM_TO_M

# Motor shafts relative to baselink (from datasheet - z is measured)
PROP0 = scale_mm([95.0, 130.0, 24.0])
PROP1 = scale_mm([95.0, -130.0, 24.0])
PROP2 = scale_mm([-95.0, -130.0, 24.0])
PROP3 = scale_mm([-95.0, 130.0, 24.0])

# Tether attachment points: equilateral triangle centered on base_link (xy),
# 2 cm below base_link, 5 cm side length.
ATTACH = True
ATTACH_SIDE = 0.05 # [m]
ATTACH_Z = -0.02 # [m]

def attachment_points(side: float, z: float) -> list[str]:
    r = side / math.sqrt(3)
    return [
        f"{round(r * math.cos(math.radians(angle)), 12)} {round(r * math.sin(math.radians(angle)), 12)} {z}"
        for angle in (0, 120, 240)
    ]

ATTACH0, ATTACH1, ATTACH2 = attachment_points(ATTACH_SIDE, ATTACH_Z)

def box_inertia(mass: float, size_x: float, size_y: float, size_z: float) -> tuple[float, float, float]:
    ixx = mass / 12.0 * (pow(size_y,2) + pow(size_z,2))
    iyy = mass / 12.0 * (pow(size_x,2) + pow(size_z,2))
    izz = mass / 12.0 * (pow(size_x,2) + pow(size_y,2))
    return ixx, iyy, izz

def coefs(mass: float, nrot: int, rpm_max: float, thrust2weight: float) -> tuple[float, float]:
    g = 9.82
    weight = mass * g

    # Maximum thrust required
    thrust_max_total = weight * thrust2weight
    thrust_max = thrust_max_total / nrot

    # T = kf * RPM²
    kf = thrust_max / rpm_max**2

    # TODO: derive from propeller torque data
    km = 9.9e-9
    return kf, km

def generate() -> str:
    ixx, iyy, izz = box_inertia(MASS, SIZE_X, SIZE_Y, SIZE_Z)
    kf, km = coefs(MASS, NROT, RPM_MAX, T2W)

    with open(os.path.join(SCRIPT_DIR, "starling2max.urdf.j2")) as f:
        template = jinja2.Template(f.read(), undefined=jinja2.StrictUndefined)

    urdf = template.render(
        kf=kf,
        km=km,
        mass=MASS,
        ixx=ixx,
        iyy=iyy,
        izz=izz,
        size_x=SIZE_X,
        size_y=SIZE_Y,
        size_z=SIZE_Z,
        scale_x=MM_TO_M,
        scale_y=MM_TO_M,
        scale_z=MM_TO_M,
        prop0=PROP0,
        prop1=PROP1,
        prop2=PROP2,
        prop3=PROP3,
        attach=ATTACH,
        attach0=ATTACH0,
        attach1=ATTACH1,
        attach2=ATTACH2,
    )

    with open(os.path.join(SCRIPT_DIR, "starling2max.urdf"), "w") as f:
        f.write(urdf)
    print("Wrote starling2max.urdf")

if __name__ == "__main__":
    generate()