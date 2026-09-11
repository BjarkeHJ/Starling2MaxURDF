
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
T2W = 2.0
PROP_RADISU = 0.09 # [m]
RPM_MAX = KV * VBAT_MAX * 0.85

# Measured Quantities
# Drone BBOX
SIZE_X = 217 * MM_TO_M
SIZE_Y = 287 * MM_TO_M
SIZE_Z = 129 * MM_TO_M

# Motor shafts relative to mesh origin
PROP0 = scale_mm([94.96, 129.91, 24.06])
PROP1 = scale_mm([94.96, -129.91, 24.06])
PROP2 = scale_mm([-94.96, -129.91, 24.06])
PROP3 = scale_mm([-94.96, 129.91, 24.06])

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
    )

    with open(os.path.join(SCRIPT_DIR, "starling2max.urdf"), "w") as f:
        f.write(urdf)
    print("Wrote starling2max.urdf")

if __name__ == "__main__":
    generate()