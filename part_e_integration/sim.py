"""Small MuJoCo interface shared by the pipeline, Part A's model check and Part B.

load() applies the floor-contact correction from Part D (fix_floor_contacts), so every
part drives the same robot. truth() and collided() are for evaluation only.
"""

from pathlib import Path

import mujoco
import numpy as np

SCENE = Path(__file__).resolve().parents[1] / "robotis_tb3" / "sceneforturtle.xml"
CAM_W, CAM_H = 640, 480
CAM_FOVY_DEG = 60.0
CONTROL_DT = 0.02


def fix_floor_contacts(model, mj=mujoco):
    """Stop the Waffle Pi slipping: Part D's contact correction, applied to the loaded model only.

    The ball casters get contact priority so their frictionless contact is used, and the
    chassis collision mesh is lifted to 2 mm above the floor, so only wheels and casters
    touch it (see PART_D_CONTROL.md, section 3.1). The xml files are not changed.
    """
    base = mj.mj_name2id(model, mj.mjtObj.mjOBJ_BODY, "base")
    for geom in range(model.ngeom):
        if model.geom_bodyid[geom] != base:
            continue
        if model.geom_type[geom] == mj.mjtGeom.mjGEOM_SPHERE and model.geom_condim[geom] == 1:
            model.geom_priority[geom] = 1
        mesh = model.geom_dataid[geom]
        if (model.geom_type[geom] != mj.mjtGeom.mjGEOM_MESH
                or not model.geom_contype[geom]
                or mj.mj_id2name(model, mj.mjtObj.mjOBJ_MESH, mesh) != "waffle_pi_base"):
            continue
        first = model.mesh_vertadr[mesh]
        vertices = model.mesh_vert[first:first + model.mesh_vertnum[mesh]]
        rotation = np.zeros(9)
        mj.mju_quat2Mat(rotation, model.geom_quat[geom])
        lowest = (vertices @ rotation.reshape(3, 3).T)[:, 2].min() + model.geom_pos[geom, 2]
        model.geom_pos[geom, 2] += max(0.0, 0.002 - lowest)


def load():
    model = mujoco.MjModel.from_xml_path(str(SCENE))
    fix_floor_contacts(model)
    data = mujoco.MjData(model)
    mujoco.mj_forward(model, data)
    return model, data


def set_pose(model, data, x, y, theta):
    """For trial setup or calibration image collection, never estimation."""
    mujoco.mj_resetData(model, data)
    data.qpos[:3] = [x, y, 0.0]
    data.qpos[3:7] = [np.cos(theta / 2), 0, 0, np.sin(theta / 2)]
    mujoco.mj_forward(model, data)


def make_renderer(model):
    return mujoco.Renderer(model, CAM_H, CAM_W), mujoco.MjvOption()


def read_camera(renderer, options, data):
    renderer.update_scene(data, camera="rgb", scene_option=options)
    return renderer.render()


class Simulator:
    def __init__(self, wheel_gain=(1.0, 1.0)):
        self.model, self.data = load()
        self.renderer, self.options = make_renderer(self.model)
        self.wheel_gain = np.asarray(wheel_gain, dtype=float)
        self.substeps = round(CONTROL_DT / self.model.opt.timestep)
        if not np.isclose(self.substeps * self.model.opt.timestep, CONTROL_DT):
            raise ValueError("Physics timestep must divide the 20 ms control period.")
        self.encoder_addresses = [self.model.sensor(name).adr[0] for name in ("enc_left", "enc_right")]
        self.lidar_addresses = [self.model.sensor(f"scan{i}").adr[0] for i in range(36)]
        self.lidar_angles = np.arange(36) * 2 * np.pi / 36
        self.lidar_position = self.model.body("lidar_mount").pos[:2].copy()
        camera = self.model.camera("rgb")
        self.camera_position = camera.pos.copy()
        self.marker = self.model.geom("dock_marker").pos[:2].copy()
        dock = self.model.geom("dock").pos[:2]
        self.marker_normal = (self.marker - dock) / np.linalg.norm(self.marker - dock)
        # markermake.py places a 200 px black square on a 250 px white plate.
        self.marker_size = 2 * self.model.geom("dock_marker").size[1] * 200 / 250

    def reset(self, pose):
        set_pose(self.model, self.data, *pose)

    def wheel_angles(self):
        return self.data.sensordata[self.encoder_addresses].copy()

    def lidar(self):
        return self.data.sensordata[self.lidar_addresses].copy()

    def image(self):
        return read_camera(self.renderer, self.options, self.data)

    def drive(self, left, right):
        self.data.ctrl[:] = np.clip(np.array([left, right]) * self.wheel_gain, -7.88, 7.88)
        for _ in range(self.substeps):
            mujoco.mj_step(self.model, self.data)

    def truth(self):
        """Evaluation only. The navigation and controller never receive this."""
        w, x, y, z = self.data.qpos[3:7]
        yaw = np.arctan2(2 * (w * z + x * y), 1 - 2 * (y * y + z * z))
        return np.array([*self.data.qpos[:2], yaw])

    def collided(self):
        """Evaluation only: robot contact with anything except the floor."""
        base = self.model.body("base").id
        for contact in self.data.contact:
            first, second = contact.geom
            first_robot = self.model.body_rootid[self.model.geom_bodyid[first]] == base
            second_robot = self.model.body_rootid[self.model.geom_bodyid[second]] == base
            other = second if first_robot else first
            if first_robot != second_robot and self.model.geom(other).name != "floor":
                return True
        return False

    def close(self):
        self.data.ctrl[:] = 0
        self.renderer.close()
