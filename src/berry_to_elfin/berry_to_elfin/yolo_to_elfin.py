#!/usr/bin/env python3
import os
import json
import time
import subprocess

import rclpy
from rclpy.node import Node
from rclpy.duration import Duration

from std_msgs.msg import String as RosString
from geometry_msgs.msg import PointStamped
from tf2_ros import Buffer, TransformListener
from tf2_geometry_msgs import do_transform_point


class OneShotToElfin(Node):
    """
    One-shot:
      1) Espera una detección con XYZ.
      2) Transforma a target_frame (elfin_base).
      3) Construye el comando de MoveIt.
      4) Si auto_execute=True: ejecuta. Si rc==0 y use_gripper=True: cierra/abre gripper.
      5) Imprime el comando y termina.
    """

    # -------- helpers --------
    def _as_bool(self, v):
        """Convierte cualquier valor a booleano de forma segura."""
        if isinstance(v, bool):
            return v
        if isinstance(v, (int, float)):
            return v != 0
        if isinstance(v, str):
            return v.strip().lower() in ('1', 'true', 'yes', 'on', 'y', 't')
        return bool(v)

    def __init__(self):
        super().__init__('yolo_to_elfin_oneshot')

        # -------------------- Parámetros --------------------
        self.declare_parameter('detections_topic', '/blackberry/detections/xyz')
        self.declare_parameter('camera_frame', 'camera_link')
        self.declare_parameter('target_frame', 'elfin_base')
        self.declare_parameter('workspace_root', '/home/francisco/workspace/ws_elfin_cpp')

        self.declare_parameter('class_filter', -1)
        self.declare_parameter('min_z', 0.05)
        self.declare_parameter('max_z', 2.0)
        self.declare_parameter('offset_x_base', 0.20)

        self.declare_parameter('roll_deg', 0.0)
        self.declare_parameter('pitch_deg', -90.0)
        self.declare_parameter('yaw_deg', 0.0)

        self.declare_parameter('wait_sec', 5.0)           # espera máx. por una detección
        self.declare_parameter('auto_execute', False)     # si False: solo imprime comando

        # ---- Gripper ----
        self.declare_parameter('use_gripper', True)       # si False: no opera el gripper
        self.declare_parameter('close_force', 40)
        self.declare_parameter('close_width', 0)
        self.declare_parameter('open_force', 40)
        self.declare_parameter('open_width', 900)
        self.declare_parameter('grip_delay', 1.5)         # s entre cerrar y abrir

        # -------------------- Lectura --------------------
        self.det_topic  = self.get_parameter('detections_topic').value
        self.cam_frame  = self.get_parameter('camera_frame').value
        self.base_frame = self.get_parameter('target_frame').value
        self.ws_root    = os.path.expanduser(self.get_parameter('workspace_root').value)

        self.class_filter = int(self.get_parameter('class_filter').value)
        self.min_z  = float(self.get_parameter('min_z').value)
        self.max_z  = float(self.get_parameter('max_z').value)
        self.offset_x = float(self.get_parameter('offset_x_base').value)

        self.roll_deg  = float(self.get_parameter('roll_deg').value)
        self.pitch_deg = float(self.get_parameter('pitch_deg').value)
        self.yaw_deg   = float(self.get_parameter('yaw_deg').value)

        self.wait_sec   = float(self.get_parameter('wait_sec').value)
        self.auto_exec  = self._as_bool(self.get_parameter('auto_execute').value)

        self.use_grip   = self._as_bool(self.get_parameter('use_gripper').value)
        self.close_force = int(self.get_parameter('close_force').value)
        self.close_width = int(self.get_parameter('close_width').value)
        self.open_force  = int(self.get_parameter('open_force').value)
        self.open_width  = int(self.get_parameter('open_width').value)
        self.grip_delay  = float(self.get_parameter('grip_delay').value)

        # TF
        self.buf = Buffer(cache_time=Duration(seconds=5.0))
        self.tfl = TransformListener(self.buf, self)

        # Subscripción a detecciones
        self.latest = None   # (x,y,z,frame_id)
        self.sub = self.create_subscription(RosString, self.det_topic, self.on_det, 10)

        # Timer de “polling” + timeout
        self.start_t = time.time()
        self.create_timer(0.05, self.poll_once)

        self.get_logger().info(
            f"[oneshot] Esperando detección en {self.det_topic} hasta {self.wait_sec:.1f}s..."
        )

    # -------------------- Utilidad: extraer XYZ --------------------
    def _extract(self, data):
        # Formato plano
        if 'detections' not in data:
            xyz = data.get('xyz')
            if xyz and len(xyz) == 3:
                x, y, z = xyz
                if self.min_z <= z <= self.max_z:
                    return (x, y, z, data.get('frame_id', self.cam_frame))
            return None
        # Lista: toma la más cercana (z mínima)
        best = None
        best_z = 1e9
        for d in data.get('detections', []):
            xyz = d.get('xyz')
            if not xyz or len(xyz) != 3:
                continue
            x, y, z = xyz
            if not (self.min_z <= z <= self.max_z):
                continue
            if z < best_z:
                best = (x, y, z)
                best_z = z
        return (*best, data.get('frame_id', self.cam_frame)) if best else None

    # -------------------- Callback detección --------------------
    def on_det(self, msg: RosString):
        try:
            data = json.loads(msg.data)
        except Exception as e:
            self.get_logger().warn(f"JSON inválido: {e}")
            return
        got = self._extract(data)
        if got:
            self.latest = got

    # -------------------- Bucle principal --------------------
    def poll_once(self):
        # timeout sin detección
        if (time.time() - self.start_t) > self.wait_sec and not self.latest:
            self.get_logger().error("No llegó detección válida a tiempo. Saliendo.")
            rclpy.shutdown()
            return

        # si aún no hay detección, sigue esperando
        if not self.latest:
            return

        # Procesa UNA vez y termina
        x, y, z, fid = self.latest
        self.get_logger().info(f"Detección: ({x:.3f},{y:.3f},{z:.3f}) en {fid}")

        try:
            ps = PointStamped()
            ps.header.frame_id = fid
            ps.point.x, ps.point.y, ps.point.z = x, y, z
            tf = self.buf.lookup_transform(self.base_frame, fid, rclpy.time.Time(),
                                           timeout=Duration(seconds=1.0))
            pt = do_transform_point(ps, tf)
            # X = pt.point.x + self.offset_x
            X = pt.point.x 
            Y = pt.point.y + self.offset_x
            Z = pt.point.z
            self.get_logger().info(f"En {self.base_frame} (+offset X): ({X:.3f},{Y:.3f},{Z:.3f})")
        except Exception as e:
            self.get_logger().error(f"TF {self.base_frame} <- {fid} falló: {e}")
            rclpy.shutdown()
            return

        # Comando para copy/paste (o ejecutar)
        cmd = (
            "ros2 launch elfin_moveit_app elfin_go_to_pose.launch.py "
            f"x:={X:.4f} y:={Y:.4f} z:={Z:.4f} "
            f"roll_deg:={self.roll_deg:.1f} pitch_deg:={self.pitch_deg:.1f} yaw_deg:={self.yaw_deg:.1f}"
        )
        self.get_logger().info("\n=== Comando listo para copiar/pegar ===\n" + cmd + "\n=======================================")

        # Ejecutar movimiento si se pidió
        ret = None
        if self.auto_exec:
            full = (
                "source /opt/ros/humble/setup.bash && "
                "source /home/francisco/workspace/elfin_ws/install/setup.bash && "
                "source /home/francisco/workspace/ws_elfin_cpp/install/setup.bash && "
                f"{cmd}"
            )
            self.get_logger().info("Ejecutando movimiento...")
            ret = subprocess.run(['bash', '-lc', full], check=False)
            self.get_logger().info(f"Movimiento terminado. rc={ret.returncode}")

        # Operar gripper sólo si movimos y fue exitoso
        if ret is not None and ret.returncode == 0 and self.use_grip:
            self._operate_gripper()

        # Terminar siempre
        rclpy.shutdown()

    # -------------------- Gripper: cerrar y abrir --------------------
    def _operate_gripper(self):
        self.get_logger().info("Cerrando gripper...")
        subprocess.run([
            'ros2', 'topic', 'pub', '--once',
            '/gripper/command', 'onrobot_rg2ft_msgs/msg/RG2FTCommand',
            f"{{target_force: {self.close_force}, target_width: {self.close_width}, control: 1}}"
        ])

        time.sleep(self.grip_delay)

        self.get_logger().info("Abriendo gripper...")
        subprocess.run([
            'ros2', 'topic', 'pub', '--once',
            '/gripper/command', 'onrobot_rg2ft_msgs/msg/RG2FTCommand',
            f"{{target_force: {self.open_force}, target_width: {self.open_width}, control: 1}}"
        ])


def main():
    rclpy.init()
    rclpy.spin(OneShotToElfin())
