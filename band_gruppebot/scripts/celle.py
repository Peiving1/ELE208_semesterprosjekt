#!/usr/bin/env python3
""" Dette scriptet er blitt laget av Claude fable 5.2
    celle.py - det som i virkeligheten er utstyr rundt roboten, simulert for Gazebo.

  1. Samlebånd:  holder båndfarten (0,3 m/s) og mater esker inn på båndet med
                 tilfeldig rotasjon og sideposisjon.
  2. Kamera:     publiserer hvor eskene i synsfeltet er, målt i kameraramma
                 (kamera_optisk) med målefeil innenfor +-5 mm.
                 -> /kamera/deteksjoner (geometry_msgs/PoseArray)
  3. Vakuum:     /vakuum (std_msgs/Bool). Når vakuumet er på og sugekoppene står på
                 en eske, festes eska til griperen. Av = slipp.
                 -> /vakuum/status (std_msgs/String): navnet på eska som holdes, ellers tom.

Kameraet "ser" ved å lese eskenes posisjon i Gazebo (/model/<eske>/pose) og legge på
støy. Roboten bruker bare /kamera/deteksjoner, aldri fasiten.
"""
import math

import numpy as np
import rclpy
from geometry_msgs.msg import Pose, PoseArray, PoseStamped
from rclpy.node import Node
from rclpy.time import Time
from ros_gz_interfaces.msg import Entity
from ros_gz_interfaces.srv import SetEntityPose
from sensor_msgs.msg import JointState
from std_msgs.msg import Bool, Empty, Float64, String
from tf2_ros import Buffer, TransformException, TransformListener

Z0 = 0.02    # høyden til basisramma {0} over gulvet (tykkelsen på fotplata) [m]

# Kameraet i robotens basisramme {0}: T_C = transl(1.0, -1.0, 1.5) * trotx(pi), som i MATLAB.
# Gazebo regner i world (gulvet), som ligger Z0 = 0,02 m under {0}.
KAMERA_POS = np.array([1.0, -1.0, 1.5 + Z0])
SYNSFELT_LANGS = 1.05    # halv lengde av synsfeltet langs båndet [m]  (2,1 m totalt)
SYNSFELT_TVERS = 0.60    # halv bredde på tvers av båndet [m]         (1,2 m totalt)

ESKE_L, ESKE_B, ESKE_H = 0.30, 0.20, 0.10
BAND_TOPP = 0.30 + Z0   # høyde på båndet over gulvet [m] (0,30 m i {0})
BAND_X = 1.0             # båndets midtlinje
# Faste tall for cellen
ESKER = ['eske1', 'eske2', 'eske3', 'eske4', 'eske5', 'eske6']
BAANDFART = 0.3          # [m/s]
PERIODE = 9.0            # s mellom hver eske som mates inn
MATE_Y = -3.70           # her settes nye esker ned på båndet


def yaw_fra_kvaternion(q):
    return math.atan2(2*(q.w*q.z + q.x*q.y), 1 - 2*(q.y*q.y + q.z*q.z))


class Eske:
    def __init__(self, navn):
        self.navn = navn
        self.p = None            # posisjon til eskas sentrum i world
        self.yaw = 0.0
        self.t = None            # tidspunkt for siste måling [s]
        self.v = np.zeros(3)     # fart (fra de to siste målingene)
        self.festet = False      # status fra Gazebo (DetachableJoint)
        self.t_staar = None      # siden når eska har ligget stille på båndet

    def posisjon(self, t):
        """Eskas posisjon ved tid t (regnet fram fra siste måling)."""
        return self.p + self.v*(t - self.t)


class Celle(Node):
    def __init__(self):
        super().__init__('celle')
        self.declare_parameter('mate', True)
        self.declare_parameter('kamera_stoy', 0.005)  # +-5 mm (kravet til sensoren)
        self.declare_parameter('kamera_hz', 30.0)
        self.declare_parameter('froe', 1)             # startverdi for tilfeldige tall
        self.stoy = float(self.get_parameter('kamera_stoy').value)
        self.rng = np.random.default_rng(int(self.get_parameter('froe').value))

        self.esker = {n: Eske(n) for n in ESKER}
        self.tcp = None            # siste TCP-posisjon i world, fra TF
        self.psi = 0.0             # retningen griperen peker sett ovenfra
        self.t_tcp = None          # tidspunktet TCP-posisjonen gjelder for [s]
        self.tf_buffer = Buffer()
        self.tf_lytter = TransformListener(self.tf_buffer, self)
        self.vakuum = False        # ønsket vakuum (fra roboten)
        self.holder = ''           # eska som er festet nå
        self.klar = False
        self.t_alle_sett = None

        self.pub_fart = self.create_publisher(Float64, '/samleband/fart', 10)
        self.pub_det = self.create_publisher(PoseArray, '/kamera/deteksjoner', 10)
        self.pub_status = self.create_publisher(String, '/vakuum/status', 10)
        self.pub_fest = {}
        self.pub_slipp = {}
        for n in ESKER:
            self.pub_fest[n] = self.create_publisher(Empty, f'/model/{n}/detachable_joint/attach', 10)
            self.pub_slipp[n] = self.create_publisher(Empty, f'/model/{n}/detachable_joint/detach', 10)
            self.create_subscription(PoseStamped, f'/model/{n}/pose',
                                     lambda m, n=n: self.ny_pose(n, m), 10)
            self.create_subscription(String, f'/model/{n}/detachable_joint/state',
                                     lambda m, n=n: self.ny_tilstand(n, m), 10)
        self.create_subscription(JointState, '/joint_states', self.ny_leddtilstand, 10)
        self.create_subscription(Bool, '/vakuum', self.ny_vakuum, 10)
        self.flytt = self.create_client(SetEntityPose, '/world/samleband/set_pose')

        self.create_timer(0.05, self.oppstart)
        self.create_timer(0.5, self.hold_baandfart)
        self.create_timer(1.0/float(self.get_parameter('kamera_hz').value), self.kamera)
        self.t_matet = None        # tidspunkt for siste eske som ble matet inn
        self.t_matet_start = None  # tidspunktet båndet startet
        self.create_timer(0.1, self.mat_inn)
        self.create_timer(0.2, self.sjekk_baandet)
        self.get_logger().info('Venter på eskene ...')

    def naa(self):
        return self.get_clock().now().nanoseconds*1e-9

    # ------------------------------------------------------------------ innganger
    def ny_pose(self, navn, msg):
        e = self.esker[navn]
        t = Time.from_msg(msg.header.stamp).nanoseconds*1e-9
        p = np.array([msg.pose.position.x, msg.pose.position.y, msg.pose.position.z])
        if e.t is not None and t - e.t > 1e-4:
            v = (p - e.p)/(t - e.t)
            e.v = v if np.linalg.norm(v) < 5.0 else np.zeros(3)   # hopp når eska flyttes
        e.p, e.t = p, t
        e.yaw = yaw_fra_kvaternion(msg.pose.orientation)
        if self.vakuum and not self.holder:
            self.proev_grep()

    def ny_tilstand(self, navn, msg):
        e = self.esker[navn]
        e.festet = (msg.data == 'attached')
        if e.festet and self.holder != navn:
            # Gazebo fester eska av seg selv når den dukker opp. Slipp den igjen.
            self.pub_slipp[navn].publish(Empty())

    def ny_leddtilstand(self, _msg):
        """Hent TCP-posisjonen fra TF hver gang leddene har flyttet seg.

        robot_state_publisher regner ut TF fra URDF-en, så vi trenger ingen egen
        kinematikk i Python.
        """
        try:
            tf = self.tf_buffer.lookup_transform('world', 'tcp_link', Time())   # Time() = nyeste
        except TransformException:
            return
        p = tf.transform.translation
        self.tcp = np.array([p.x, p.y, p.z])
        self.psi = yaw_fra_kvaternion(tf.transform.rotation)
        self.t_tcp = Time.from_msg(tf.header.stamp).nanoseconds*1e-9
        if self.vakuum and not self.holder:
            self.proev_grep()

    def ny_vakuum(self, msg):
        if msg.data and not self.vakuum:
            self.vakuum = True
            self.proev_grep(logg_bom=True)
        elif not msg.data and self.vakuum:
            self.vakuum = False
            if self.holder:
                self.pub_slipp[self.holder].publish(Empty())
                self.get_logger().info(f'Vakuum av: slapp {self.holder}')
                self.holder = ''
            self.publiser_status()

    # ------------------------------------------------------------------ oppstart
    def oppstart(self):
        """Vent til alle eskene er på plass, og sørg for at ingen er festet til griperen."""
        if self.klar:
            return
        for n in self.esker:
            self.pub_slipp[n].publish(Empty())
        if any(e.p is None for e in self.esker.values()) or self.tcp is None:
            return
        if self.t_alle_sett is None:
            self.t_alle_sett = self.naa()
        if self.naa() - self.t_alle_sett > 1.0:
            self.klar = True
            self.t_matet = self.naa() - PERIODE + 1.0     # første eske mates inn etter 1 s
            self.t_matet_start = self.naa()                     # båndet starter nå
            self.get_logger().info(f'Klar: {len(self.esker)} esker, båndet går med {BAANDFART:.2f} m/s')

    def hold_baandfart(self):
        if self.klar:
            self.pub_fart.publish(Float64(data=BAANDFART))
            self.publiser_status()

    # ------------------------------------------------------------------ mating
    def mat_inn(self):
        """Flytt neste eske til starten av båndet, med tilfeldig rotasjon og sideposisjon.

        Tar først eskene på lageret. Esker som roboten ikke rakk, og som har falt av
        enden på båndet, sendes rundt på nytt.
        """
        if not self.klar or not self.get_parameter('mate').value:
            return
        if self.naa() - self.t_matet < PERIODE or not self.flytt.service_is_ready():
            return
        paa_lager = [e for e in self.esker.values() if e.p[0] > 1.6]
        paa_gulvet = [e for e in self.esker.values()
                      if e.p[2] < 0.2 and e.p[0] <= 1.6 and e.navn != self.holder]
        if not paa_lager and not paa_gulvet:
            return
        e = min(paa_lager or paa_gulvet, key=lambda e: e.navn)
        self.t_matet = self.naa()
        yaw = self.rng.uniform(-math.pi/2, math.pi/2)
        x = BAND_X + self.rng.uniform(-0.06, 0.06)
        self.sett_ned(e, x, MATE_Y, yaw)
        self.get_logger().info(f'{e.navn} inn på båndet: x = {x:.3f} m, rotasjon = {math.degrees(yaw):.0f} grader')

    def sett_ned(self, e, x, y, yaw):
        """Sett eska ned på båndet i (x, y) med rotasjon yaw (flyttes direkte i Gazebo)."""
        req = SetEntityPose.Request()
        req.entity = Entity(name=e.navn, type=Entity.MODEL)
        req.pose.position.x = float(x)
        req.pose.position.y = float(y)
        req.pose.position.z = BAND_TOPP + ESKE_H/2 + 0.005
        req.pose.orientation.z = math.sin(yaw/2)
        req.pose.orientation.w = math.cos(yaw/2)
        self.flytt.call_async(req)
        e.p = np.array([x, y, req.pose.position.z])
        e.v = np.zeros(3)
        e.t_staar = None

    def sjekk_baandet(self):
        """Esker som ligger på båndet uten å bli med (Gazebo mister av og til taket), settes ned på nytt."""
        if not self.klar or not self.flytt.service_is_ready():
            return
        t = self.naa()
        for e in self.esker.values():
            paa_baandet = (abs(e.p[2] - (BAND_TOPP + ESKE_H/2)) < 0.01 and abs(e.p[0] - BAND_X) < 0.2
                           and -3.9 < e.p[1] < 0.8 and e.navn != self.holder)
            if not paa_baandet or abs(e.v[1] - BAANDFART) < 0.1 or t - self.t_matet_start < 1.0:
                e.t_staar = None
            elif e.t_staar is None:
                e.t_staar = t
            elif t - e.t_staar > 0.6:
                self.get_logger().warn(f'{e.navn} ble ikke med båndet, setter den ned på nytt')
                self.sett_ned(e, e.p[0], e.p[1], e.yaw)

    # ------------------------------------------------------------------ kamera
    def kamera(self):
        """Publiser eskene i synsfeltet, målt i kameraramma med støy."""
        if not self.klar:
            return
        t = max(e.t for e in self.esker.values())
        msg = PoseArray()
        msg.header.frame_id = 'kamera_optisk'
        msg.header.stamp = Time(nanoseconds=int(t*1e9)).to_msg()
        for e in self.esker.values():
            if e.navn == self.holder:
                continue
            topp = e.posisjon(t) + [0, 0, ESKE_H/2]              # midt på toppen av eska
            if abs(topp[2] - (BAND_TOPP + ESKE_H)) > 0.03:       # ligger ikke på båndet
                continue
            # world -> kamera: T_C = transl(KAMERA_POS) * Rx(pi)  =>  p_C = Rx(pi) * (p_W - KAMERA_POS)
            d = topp - KAMERA_POS
            p_c = np.array([d[0], -d[1], -d[2]])
            if abs(p_c[0]) > SYNSFELT_TVERS or abs(p_c[1]) > SYNSFELT_LANGS:
                continue
            p_c += self.rng.uniform(-self.stoy, self.stoy, 3)    # målefeil +-5 mm
            yaw_c = -e.yaw + math.radians(self.rng.uniform(-0.5, 0.5))
            pose = Pose()
            pose.position.x, pose.position.y, pose.position.z = p_c
            pose.orientation.z = math.sin(yaw_c/2)
            pose.orientation.w = math.cos(yaw_c/2)
            msg.poses.append(pose)
        self.pub_det.publish(msg)

    # ------------------------------------------------------------------ vakuum
    def proev_grep(self, logg_bom=False):
        """Fest eska hvis sugekoppene (TCP) står på toppen av den."""
        if self.tcp is None or not self.klar:
            return
        best = None
        for e in self.esker.values():
            if e.p is None:
                continue
            topp = e.posisjon(self.t_tcp) + [0, 0, ESKE_H/2]
            d = self.tcp - topp
            # avviket sett i eskas egne akser (langs og på tvers av eska)
            langs = d[0]*math.cos(e.yaw) + d[1]*math.sin(e.yaw)
            tvers = -d[0]*math.sin(e.yaw) + d[1]*math.cos(e.yaw)
            if best is None or np.hypot(d[0], d[1]) < best[1]:
                best = (e, np.hypot(d[0], d[1]), d[2], langs, tvers, self.psi - e.yaw)
        e, feil, hoyde, langs, tvers, vri = best
        paa_eska = abs(langs) < ESKE_L/2 - 0.02 and abs(tvers) < ESKE_B/2 - 0.02
        if paa_eska and -0.005 < hoyde < 0.015:
            self.pub_fest[e.navn].publish(Empty())
            self.holder = e.navn
            vri = (vri + math.pi/2) % math.pi - math.pi/2
            self.get_logger().info(
                f'Vakuum på: grep {e.navn}. Plukkfeil = {feil*1000:.1f} mm '
                f'(krav < 10 mm), høyde over eska = {hoyde*1000:.1f} mm, '
                f'vinkel mot eska = {math.degrees(vri):.1f} grader')
            self.publiser_status()
        elif logg_bom:
            self.get_logger().warn(
                f'Vakuum på, men ingen eske under sugekoppene ennå (nærmeste: {e.navn}, '
                f'{feil*1000:.0f} mm unna, {hoyde*1000:.0f} mm over)')

    def publiser_status(self):
        self.pub_status.publish(String(data=self.holder))


def main():
    rclpy.init()
    node = Celle()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass


if __name__ == '__main__':
    main()
