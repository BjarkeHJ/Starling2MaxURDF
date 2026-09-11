
import os
import argparse
import jinja2

SCRIPT_DIR = os.path.dirname(os.path.realpath(__file__))
MM_TO_M = 0.001

def scale_mm(xyz: list[float]) -> str:
    return " ".join(str(v * MM_TO_M) for v in xyz)

# Measured Quantities
SIZE_X = 217 * MM_TO_M
SIZE_Y = 287 * MM_TO_M
SIZE_Z = 129 * MM_TO_M

PROP0 = scale_mm([94.96, 129.91, 24.06])
PROP1 = scale_mm([94.96, -129.91, 24.06])
PROP2 = scale_mm([-94.96, -129.91, 24.06])
PROP3 = scale_mm([-94.96, 129.91, 24.06])

def box_inertia(mass: float, size_x: float, size_y: float, size_z: float) -> tuple[float, float, float]:
    ixx = mass / 12.0 * (size_y**2 + size_z**2)
    iyy = mass / 12.0 * (size_x**2 + size_z**2)
    izz = mass / 12.0 * (size_x**2 + size_y**2)
    return ixx, iyy, izz


def generate(mass: float) -> str:


    ixx, iyy, izz = box_inertia(mass,SIZE_X, SIZE_Y, SIZE_Z)

    with open(os.path.join(SCRIPT_DIR, "starling2max.urdf.j2")) as f:
        template = jinja2.Template(f.read(), undefined=jinja2.StrictUndefined)


    urdf = template.render(
        mass=mass,
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
    generate(0.566)