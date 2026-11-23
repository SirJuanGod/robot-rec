"""Rutina del ROBOT-REC (base diferencial + Kinova Gen3): recorrer la zona y, en cada papelera de malla,
tomarla por el aro superior, vaciarla en la tolva volteándola y devolverla a su sitio.

    python simular.py                 # auto: Genesis si hay GPU CUDA, si no MuJoCo
    python simular.py --motor mujoco  # forzar backend
    python simular.py --sin-ventana   # sin visor (prueba rápida); sale con código 1 si falla
"""
import argparse
import contextlib
import io
import math
import os
import sys
import time

import mujoco
import numpy as np

import oficina

AQUI = os.path.dirname(os.path.abspath(__file__))
BRAZO = [f"joint_{i}" for i in range(1, 8)]
DEDOS = ["dedo_izq", "dedo_der"]
RUEDAS = ["rueda_izq", "rueda_der"]      # tracción diferencial; las 4 ruedas locas son pasivas
GEN3_HOME = np.array([0, 0.26179939, 3.14159265, -2.26892803, 0, 0.95993109, 1.57079633])  # Menagerie
DEDOS_ABIERTOS, DEDOS_CERRADOS = 0.0, 0.05   # cerrado = tope: los dedos aprietan el aro con ~110 N
DEDOS_AFLOJADOS = 0.02                       # dedos a ~1 cm del aro: ya no aprietan pero siguen a sus lados
# Todo lo que sigue está en el marco del chasis (x adelante, z=0 a media altura del cuerpo)
BRAZO_X = 0.32                   # x de la base del Gen3: el carro se detiene con la papelera a su costado
LADO_Y = 0.55                    # las papeleras quedan a la izquierda del camino, a esta distancia del eje del carro
VEL_RUEDAS = 2.7                 # rad/s (~0.32 m/s con rueda de 12 cm)
REPOSO = np.array([0.55, 0.0, 0.50])     # TCP en reposo / navegación, sobre el frente
Z_TRASLADO = 0.58                        # TCP al cruzar con la papelera: su base queda ~7 cm sobre el borde de la tolva
# la papelera se lleva en arco alrededor de la base del brazo hasta quedar detrás, sobre la tolva, a R_VACIAR
# de la base, y ahí se invierte del todo girando la pinza alrededor del aro (eje x del chasis). Con la pinza
# en R_PINZA el Gen3 llega a -180° sin tocar el límite de joint_6 (verificado en la autoprueba).
R_VACIAR = 0.45                          # -> punto de vaciado en x = BRAZO_X - 0.45 = -0.13
INCLINAR = -math.pi
TOLVA = dict(x=(-0.528, 0.068), y=0.318, z=0.34)   # hueco interior de la tolva
# Pinza apuntando al piso, dedos cerrando en la dirección y del chasis: aprietan la pared de la papelera
# que mira al carro (un dedo por dentro y otro por fuera del aro)
R_PINZA = np.array([[0, 1, 0], [1, 0, 0], [0, 0, -1]], dtype=float)
# Escenario (oficina.py): papeleras repartidas por la oficina, cada una con basura de distintos tamaños y
# la ruta por los pasillos para llegar a ella. El robot pasa con la papelera a su izquierda (carga lateral,
# como un camión recolector), así nunca le pasa por encima.
TAREAS = oficina.TAREAS
BASURA = oficina.basura_inicial()
# punto de agarre en el marco de la papelera: centro del aro de la pared -y (la que mira al carro), con
# las almohadillas (40 mm) justo debajo del reborde
AGARRE = np.array([0.0, -0.105, 0.167])


def quat_z(a):
    """Cuaternión (w, x, y, z) de un giro a alrededor de la vertical."""
    return [math.cos(a / 2), 0.0, 0.0, math.sin(a / 2)]


def rot_y(th):
    c, s = math.cos(th), math.sin(th)
    return np.array([[c, 0, s], [0, 1, 0], [-s, 0, c]])


class IK:
    """IK numérica (mínimos cuadrados amortiguados) del Gen3 sobre el modelo MuJoCo del robot,
    con el chasis fijo en el origen: los objetivos se dan en el marco del chasis.
    La usan los dos simuladores (MuJoCo solo como librería de cinemática en el caso de Genesis)."""

    def __init__(self):
        self.m = mujoco.MjModel.from_xml_path(os.path.join(AQUI, "robot_rec.xml"))
        self.d = mujoco.MjData(self.m)
        self.d.qpos[:7] = [0, 0, 0, 1, 0, 0, 0]
        self.qa = [self.m.joint(n).qposadr[0] for n in BRAZO]
        self.va = [self.m.joint(n).dofadr[0] for n in BRAZO]
        self.lim = np.array([self.m.joint(n).range if self.m.joint(n).limited else (-1e9, 1e9) for n in BRAZO])
        self.sitio = self.m.site("tcp").id
        self.postura = GEN3_HOME
        self.avisar_como_error = False       # la autoprueba exige trayectorias continuas

    def tcp(self, q):
        self.d.qpos[self.qa] = q
        mujoco.mj_kinematics(self.m, self.d)
        return self.d.site_xpos[self.sitio].copy(), self.d.site_xmat[self.sitio].reshape(3, 3).copy()

    def resolver(self, p, semilla, R):
        q = np.array(semilla, dtype=float)
        jp, jr = np.zeros((3, self.m.nv)), np.zeros((3, self.m.nv))
        for _ in range(500):
            pos, Ra = self.tcp(q)
            e = np.concatenate([p - pos, 0.5 * sum(np.cross(Ra[:, i], R[:, i]) for i in range(3))])
            if np.linalg.norm(e) < 2e-4:                    # 0.2 mm / 0.01°
                return q
            mujoco.mj_comPos(self.m, self.d)
            mujoco.mj_jacSite(self.m, self.d, jp, jr, self.sitio)
            J = np.vstack([jp, jr])[:, self.va]
            Jp = J.T @ np.linalg.inv(J @ J.T + 1e-4 * np.eye(6))
            # tarea + redundancia (7 DOF): en el espacio nulo, tender a la postura de reposo (codo arriba)
            dq = Jp @ e + 0.05 * (np.eye(7) - Jp @ J) @ (self.postura - q)
            dq *= min(1.0, 0.2 / np.abs(dq).max())          # paso limitado: converge sin saltos
            q = np.clip(q + dq, *self.lim.T)
        return None

    def __call__(self, p, R=R_PINZA, semilla=None, avisar=True):
        # desde la configuración actual (movimiento continuo); si no converge, reintentar con la
        # cintura girada, avisando porque ese cambio de solución es un movimiento grande
        p = np.asarray(p, dtype=float)
        q = self.resolver(p, semilla, R)
        if q is not None:
            return q
        for g in (0, 1.57, -1.57, 3.14):
            q = self.resolver(p, self.postura + [g, 0, 0, 0, 0, 0, 0], R)
            if q is not None:
                assert not self.avisar_como_error, f"autoprueba: la IK salta de postura en {np.round(p, 3)}"
                if avisar:
                    print(f"   aviso: IK cambió de solución para {np.round(p, 3)} (movimiento brusco)")
                return q
        raise ValueError(f"IK sin solución para {np.round(p, 3)}")


def al_chasis(sim, p_mundo):
    pos, R = sim.chasis()
    return R.T @ (p_mundo - pos)


# ---------------- rutina (independiente del simulador) ----------------
def esperar(sim, T):
    for _ in range(int(T / sim.dt)):
        yield


def pinza(sim, desde, hasta, T=0.6):
    """Abrir/cerrar la pinza en rampa (como un servo real): cerrar de golpe a 60 N lanza los dedos
    tan rápido que atraviesan el asa en un paso de simulación."""
    n = int(T / sim.dt)
    for k in range(1, n + 1):
        sim.pinza(desde + (hasta - desde) * k / n)
        yield
    yield from esperar(sim, 0.3)


def ir(sim, ik, q, p, T, th=(0.0, 0.0), R0=R_PINZA, n=10):
    """Mover el TCP en línea recta hasta p (marco del chasis) girando la pinza de th[0] a th[1]
    alrededor de su eje y (el eje del asa). IK en n puntos intermedios, cada uno sembrado con el
    anterior, para que la configuración del brazo cambie de forma continua."""
    p0, p = ik.tcp(q)[0], np.asarray(p, dtype=float)
    return (yield from seguir(sim, ik, q, lambda s: (p0 + (p - p0) * s, R0 @ rot_y(th[0] + (th[1] - th[0]) * s)), T, n))


def arco(sim, ik, q, a, r, T, n=20):
    """Mover el TCP en arco horizontal alrededor de la base del brazo (a Z_TRASLADO), del ángulo a[0]
    al a[1] y del radio r[0] al r[1], con la pinza siempre en la misma orientación (asa a lo largo del carro)."""
    def pose(s):
        ang, rad = a[0] + (a[1] - a[0]) * s, r[0] + (r[1] - r[0]) * s
        return [BRAZO_X + rad * math.cos(ang), rad * math.sin(ang), Z_TRASLADO], R_PINZA
    return (yield from seguir(sim, ik, q, pose, T, n))


def seguir(sim, ik, q, pose, T, n):
    """Seguir la trayectoria pose(s), s∈[0,1] (perfil suave), resolviendo la IK en n puntos encadenados."""
    for k in range(1, n + 1):
        s = k / n
        q_sig = ik(*pose(s * s * (3 - 2 * s)), q)
        for i in range(1, int(T / n / sim.dt) + 1):
            sim.brazo(q + (q_sig - q) * i * n * sim.dt / T)
            yield
        q = q_sig
    return q


def agarre_actual(sim, i):
    """Punto de agarre (aro) de la papelera i en el marco del chasis, medido ahora."""
    p, R = sim.papelera(i)
    return al_chasis(sim, p + R @ AGARRE)


def vaciar_papelera(sim, ik, q, i):
    agarre = agarre_actual(sim, i)
    base = al_chasis(sim, sim.papelera(i)[0])                        # sitio original (centro de la base)
    print(f"   papelera {i + 1}: aro en {np.round(agarre, 3)} (marco del chasis) -> tomando")
    q = yield from ir(sim, ik, q, agarre + [0, 0, 0.10], 1.5)        # sobre el aro, dedos abiertos a ambos lados
    q = yield from ir(sim, ik, q, agarre, 1.0)                       # bajar: un dedo por dentro, otro por fuera
    yield from pinza(sim, DEDOS_ABIERTOS, DEDOS_CERRADOS)
    lado = [agarre[0], agarre[1], Z_TRASLADO]
    q = yield from ir(sim, ik, q, lado, 1.5)                         # subir en vertical

    # en arco alrededor de la base del brazo hasta quedar detrás, sobre la tolva
    a_lado = math.atan2(lado[1], lado[0] - BRAZO_X)
    a_atras = math.copysign(math.pi, a_lado)
    q = yield from arco(sim, ik, q, (a_lado, a_atras), (abs(lado[1]), R_VACIAR), 2.5)
    p_vaciar = [BRAZO_X - R_VACIAR, 0, Z_TRASLADO]

    print(f"   papelera {i + 1}: vaciando en la tolva")
    q = yield from ir(sim, ik, q, p_vaciar, 2.0, th=(0, INCLINAR), n=20)
    yield from esperar(sim, 1.2)
    q = yield from ir(sim, ik, q, p_vaciar, 3.0, th=(INCLINAR, 0), n=30)     # enderezar despacio
    yield from esperar(sim, 0.5)

    def soltada():
        """¿El aro ya no está entre los dedos? (en el marco de la pinza: x = cierre, y = a lo largo del aro,
        z = hacia las puntas). Deslizar a lo largo del aro no importa mientras quede contacto."""
        tcp, Rt = ik.tcp(q)
        x, y, z = rel = Rt.T @ (agarre_actual(sim, i) - tcp)
        if abs(x) > 0.02 or abs(y) > 0.09 or abs(z) > 0.04:
            print(f"   papelera {i + 1}: aro desplazado {np.round(rel * 1000).astype(int)} mm en la pinza")
            return True
        return False

    if soltada():
        print(f"   papelera {i + 1}: ¡se soltó de la pinza! se abandona y se sigue")
        yield from pinza(sim, DEDOS_CERRADOS, DEDOS_ABIERTOS)
        q = yield from arco(sim, ik, q, (a_atras, a_lado), (R_VACIAR, abs(lado[1])), 3.0)
        return (yield from ir(sim, ik, q, REPOSO, 2.0))

    print(f"   papelera {i + 1}: devolviéndola a su sitio")
    q = yield from arco(sim, ik, q, (a_atras, a_lado), (R_VACIAR, abs(lado[1])), 3.0)
    q = yield from ir(sim, ik, q, agarre + [0, 0, 0.10], 1.5)
    if soltada():
        print(f"   papelera {i + 1}: ¡se soltó en el regreso! se abandona y se sigue")
        yield from pinza(sim, DEDOS_CERRADOS, DEDOS_ABIERTOS)
        return (yield from ir(sim, ik, q, REPOSO, 2.0))
    # colgada de una pared la papelera queda inclinada (~16°) y el aro pudo deslizar en la pinza: llevar el
    # centro de la BASE sobre su sitio original y el aro a su altura original + 4 mm (si se la aprieta contra
    # el piso, al enderezarse empuja de lado las almohadillas y al abrir se arrastra); al aflojar se asienta
    yield from esperar(sim, 0.3)
    tcp = ik.tcp(q)[0]
    dxy = (base - al_chasis(sim, sim.papelera(i)[0]))[:2]
    dz = agarre[2] - agarre_actual(sim, i)[2] + 0.004
    q = yield from ir(sim, ik, q, [tcp[0] + dxy[0], tcp[1] + dxy[1], tcp[2] + dz], 1.5)
    yield from esperar(sim, 0.5)
    # soltar: el reborde descansa sobre las almohadillas; aflojar, bajar la pinza para despegarlas del
    # reborde y recién entonces abrir del todo (si no, al abrir una almohadilla arrastra la papelera)
    yield from pinza(sim, DEDOS_CERRADOS, DEDOS_AFLOJADOS)
    q = yield from ir(sim, ik, q, ik.tcp(q)[0] - [0, 0, 0.012], 0.6, n=3)
    yield from pinza(sim, DEDOS_AFLOJADOS, DEDOS_ABIERTOS)
    # al apoyarla el brazo cede algo de lado: recentrar la pinza abierta sobre el aro antes de subir, para
    # que ninguna almohadilla quede bajo el reborde
    tcp = ik.tcp(q)[0]
    q = yield from ir(sim, ik, q, [*agarre_actual(sim, i)[:2], tcp[2]], 0.6, n=3)
    tcp = ik.tcp(q)[0]
    q = yield from ir(sim, ik, q, [tcp[0], tcp[1], Z_TRASLADO], 1.5)  # subir en vertical, sacando el dedo interior
    q = yield from ir(sim, ik, q, REPOSO, 1.5)
    return q


def rumbo_actual(sim):
    R = sim.chasis()[1]
    return math.atan2(R[1, 0], R[0, 0])


def angulo(a):
    return (a + math.pi) % (2 * math.pi) - math.pi


def girar_a(sim, rumbo):
    """Girar en el sitio (ruedas en sentidos opuestos) hasta el rumbo pedido."""
    while abs(err := angulo(rumbo - rumbo_actual(sim))) > 0.015:
        g = float(np.clip(3.0 * err, -1.5, 1.5))
        g = math.copysign(max(abs(g), 0.25), g)                       # mínimo para vencer el rozamiento
        sim.ruedas(-g, g)
        yield
    sim.ruedas(0)
    yield from esperar(sim, 0.3)


class Bloqueado(Exception):
    pass


def ir_a(sim, punto, tol=0.10, t_max=60.0):
    """Ir a un punto de paso: si queda muy de lado, girar en el sitio; si no, avanzar corrigiendo el rumbo."""
    meta = np.array([*punto, 0.0])
    pasos = int(t_max / sim.dt)
    while math.hypot(*(p := al_chasis(sim, meta))[:2]) > tol:
        if (pasos := pasos - 1) < 0:
            sim.ruedas(0)
            raise Bloqueado(f"no llega a {np.round(punto, 2)} (¿obstáculo?)")
        ang = math.atan2(p[1], p[0])
        if abs(ang) > 0.35:
            g = float(np.clip(3.0 * ang, -1.5, 1.5))
            sim.ruedas(-g, g)
        else:
            g = float(np.clip(3.0 * ang, -1.0, 1.0))
            sim.ruedas(VEL_RUEDAS - g, VEL_RUEDAS + g)
        yield
    sim.ruedas(0)


def aproximar(sim, i, rumbo):
    """Recta de aproximación: avanzar con rumbo fijo, con la papelera a LADO_Y a la izquierda, hasta tenerla
    al costado de la base del brazo."""
    izq = np.array([-math.sin(rumbo), math.cos(rumbo), 0.0])
    fwd = np.array([math.cos(rumbo), math.sin(rumbo), 0.0])
    meta = sim.papelera(i)[0] - LADO_Y * izq                          # punto de la recta junto a la papelera
    meta[2] = 0.0
    while (p := al_chasis(sim, meta))[0] > BRAZO_X:
        # seguidor de línea: apuntar a un punto de la recta 0,8 m por delante de la proyección del carro
        pos = sim.chasis()[0]
        mira = meta + ((pos - meta) @ fwd + 0.8) * fwd
        m = al_chasis(sim, mira)
        g = float(np.clip(3.0 * math.atan2(m[1], m[0]), -1.0, 1.0))
        sim.ruedas(VEL_RUEDAS - g, VEL_RUEDAS + g)
        yield
    sim.ruedas(0)
    yield from esperar(sim, 1.0)


def rutina(sim, ik, q):
    sim.brazo(q); sim.pinza(DEDOS_ABIERTOS); sim.ruedas(0)
    yield from esperar(sim, 0.5)
    for i, t in enumerate(TAREAS):
        print(f"{i + 1}) Yendo a la papelera {i + 1} en {t['papelera']}...")
        izq = np.array([-math.sin(t["rumbo"]), math.cos(t["rumbo"])])
        fwd = np.array([math.cos(t["rumbo"]), math.sin(t["rumbo"])])
        try:
            for punto in t["ruta"]:                                    # pasillos
                yield from ir_a(sim, punto)
            yield from ir_a(sim, np.array(t["papelera"]) - LADO_Y * izq - 1.5 * fwd)   # inicio de la aproximación
        except Bloqueado as e:
            print(f"   ¡bloqueado! {e}; se salta esta papelera")
            sim.devueltas.append(float("inf"))
            continue
        yield from girar_a(sim, t["rumbo"])
        yield from aproximar(sim, i, t["rumbo"])
        inicio = sim.papelera(i)[0]
        q = yield from vaciar_papelera(sim, ik, q, i)
        sim.devueltas.append(np.linalg.norm((sim.papelera(i)[0] - inicio)[:2]))
    yield from esperar(sim, 1.0)


def resultado(sim):
    """(residuos en la tolva, papeleras devueltas de pie a menos de 5 cm de donde estaban)."""
    en_tolva = 0
    for j in range(len(BASURA)):
        x, y, z = al_chasis(sim, sim.basura(j))
        en_tolva += TOLVA["x"][0] < x < TOLVA["x"][1] and abs(y) < TOLVA["y"] and z < TOLVA["z"]
    devueltas = sum(d < 0.05 and sim.papelera(i)[1][2, 2] > 0.95 for i, d in enumerate(sim.devueltas))
    return en_tolva, devueltas


# ---------------- backends ----------------
class SimMuJoCo:
    def __init__(self, ventana, q0):
        spec = mujoco.MjSpec.from_file(os.path.join(AQUI, "escena.xml"))
        for i, t in enumerate(TAREAS):
            c = mujoco.MjSpec.from_file(os.path.join(AQUI, "papelera.xml"))
            marco = spec.worldbody.add_frame(pos=[*t["papelera"], 0], quat=quat_z(t["rumbo"]))
            marco.attach_body(c.body("papelera"), f"p{i}_", "")
        G = mujoco.mjtGeom
        for j, (tipo, med, masa, color, pos) in enumerate(BASURA):
            b = spec.worldbody.add_body(name=f"basura{j}", pos=pos)
            b.add_freejoint()
            if tipo == "bola":
                tipo_g, tam = G.mjGEOM_SPHERE, [med, 0, 0]
            elif tipo == "lata":
                tipo_g, tam = G.mjGEOM_CYLINDER, [med[0], med[1] / 2, 0]
            else:
                tipo_g, tam = G.mjGEOM_BOX, np.array(med) / 2
            b.add_geom(type=tipo_g, size=tam, mass=masa, rgba=[*color, 1], contype=3, conaffinity=3)
        self.m = spec.compile()
        self.d = mujoco.MjData(self.m)
        self.a_brazo = [self.m.actuator(n).id for n in BRAZO]
        self.a_dedos = [self.m.actuator(n).id for n in DEDOS]
        self.a_ruedas = [self.m.actuator(n).id for n in RUEDAS]
        self.d.qpos[[self.m.joint(n).qposadr[0] for n in BRAZO]] = q0      # arrancar en reposo
        x0, y0, rumbo0 = oficina.INICIO
        self.d.qpos[0:7] = [x0, y0, 0.193, *quat_z(rumbo0)]                # pose inicial en la oficina
        mujoco.mj_forward(self.m, self.d)
        self.dt = self.m.opt.timestep
        self.devueltas = []
        self.visor = None
        if ventana:
            import mujoco.viewer as visor_mj
            self.visor = visor_mj.launch_passive(self.m, self.d)

    def brazo(self, q):
        self.d.ctrl[self.a_brazo] = q

    def pinza(self, x):
        self.d.ctrl[self.a_dedos] = x

    def ruedas(self, izq, der=None):
        der = izq if der is None else der
        self.d.ctrl[self.a_ruedas] = [izq, der]

    def chasis(self):
        b = self.d.body("chasis")
        return b.xpos.copy(), b.xmat.reshape(3, 3).copy()

    def papelera(self, i):
        b = self.d.body(f"p{i}_papelera")
        return b.xpos.copy(), b.xmat.reshape(3, 3).copy()

    def basura(self, j):
        return self.d.body(f"basura{j}").xpos.copy()

    def step(self):
        mujoco.mj_step(self.m, self.d)
        if self.visor and self.d.time % (1 / 60) < self.dt:
            if not self.visor.is_running():
                sys.exit(0)
            with self.visor.lock():
                self.visor.cam.lookat[:] = self.d.body("chasis").xpos   # la cámara sigue al carro
            self.visor.sync()
            time.sleep(1 / 60)   # shortcut: ritmo aproximado a tiempo real, no compensa el cómputo

    def fin(self):
        while self.visor and self.visor.is_running():
            self.visor.sync(); time.sleep(0.05)


def quat_a_mat(q):
    R = np.zeros(9)
    mujoco.mju_quat2Mat(R, np.asarray(q, dtype=float).reshape(4))   # (w, x, y, z)
    return R.reshape(3, 3)


class SimGenesis:
    def __init__(self, ventana, gpu, q0):
        import genesis as gs
        gs.init(backend=gs.gpu if gpu else gs.cpu, logging_level="warning")
        self.dt = 0.002
        self.esc = gs.Scene(
            sim_options=gs.options.SimOptions(dt=self.dt),
            # mismos ajustes de agarre que escena.xml en MuJoCo (noslip, impratio, fricción torsional de las almohadillas)
            rigid_options=gs.options.RigidOptions(noslip_iterations=5, impratio=10, enable_torsional_friction=True),
            viewer_options=gs.options.ViewerOptions(camera_pos=(3.5, -3.0, 2.0), camera_lookat=(1.8, 0, 0.3)),
            show_viewer=ventana,
        )
        self.esc.add_entity(gs.morphs.Plane())
        self.esc.add_entity(gs.morphs.MJCF(file=os.path.join(AQUI, "oficina.xml")))
        x0, y0, rumbo0 = oficina.INICIO
        self.robot = self.esc.add_entity(gs.morphs.MJCF(file=os.path.join(AQUI, "robot_rec.xml"),
                                                        pos=(x0, y0, 0), euler=(0, 0, math.degrees(rumbo0))))
        self.papeleras = [self.esc.add_entity(gs.morphs.MJCF(file=os.path.join(AQUI, "papelera.xml"),
                                                             pos=(*t["papelera"], 0), euler=(0, 0, math.degrees(t["rumbo"]))))
                          for t in TAREAS]
        self.basuras = []
        for tipo, med, masa, color, pos in BASURA:
            if tipo == "bola":
                morph, vol = gs.morphs.Sphere(radius=med, pos=pos), 4 / 3 * math.pi * med ** 3
            elif tipo == "lata":
                morph, vol = gs.morphs.Cylinder(radius=med[0], height=med[1], pos=pos), math.pi * med[0] ** 2 * med[1]
            else:
                morph, vol = gs.morphs.Box(size=med, pos=pos), float(np.prod(med))
            self.basuras.append(self.esc.add_entity(morph, material=gs.materials.Rigid(rho=masa / vol),
                                                    surface=gs.surfaces.Default(color=color)))
        self.esc.build()
        idx = lambda nombres: [self.robot.get_joint(n).dofs_idx_local[0] for n in nombres]
        self.i_brazo, self.i_dedos, self.i_ruedas = idx(BRAZO), idx(DEDOS), idx(RUEDAS)
        # mismas ganancias / límites de fuerza que robot_rec.xml
        r = self.robot
        r.set_dofs_position(q0, self.i_brazo)
        r.set_dofs_kp([2000] * 4 + [500] * 3, self.i_brazo)
        r.set_dofs_kv([100] * 4 + [50] * 3, self.i_brazo)
        r.set_dofs_force_range([-105] * 4 + [-52] * 3, [105] * 4 + [52] * 3, self.i_brazo)
        r.set_dofs_kp([6000, 6000], self.i_dedos)
        r.set_dofs_kv([40, 40], self.i_dedos)
        r.set_dofs_force_range([-120, -120], [120, 120], self.i_dedos)
        r.set_dofs_kv([50, 50], self.i_ruedas)
        r.set_dofs_force_range([-40, -40], [40, 40], self.i_ruedas)
        self.devueltas = []

    def brazo(self, q):
        self.robot.control_dofs_position(np.asarray(q, dtype=float), self.i_brazo)

    def pinza(self, x):
        self.robot.control_dofs_position(np.array([x, x], dtype=float), self.i_dedos)

    def ruedas(self, izq, der=None):
        der = izq if der is None else der
        self.robot.control_dofs_velocity(np.array([izq, der], dtype=float), self.i_ruedas)

    @staticmethod
    def _pose(ent):
        return ent.get_pos().cpu().numpy().reshape(3), quat_a_mat(ent.get_quat().cpu().numpy())

    def chasis(self):
        return self._pose(self.robot.get_link("chasis"))

    def papelera(self, i):
        return self._pose(self.papeleras[i])

    def basura(self, j):
        return self.basuras[j].get_pos().cpu().numpy().reshape(3)

    def step(self):
        self.esc.step()

    def fin(self):
        pass


def hay_cuda():
    try:
        import torch
        return torch.cuda.is_available()
    except ImportError:
        return False


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--motor", choices=["auto", "mujoco", "genesis"], default="auto")
    ap.add_argument("--sin-ventana", action="store_true")
    a = ap.parse_args()

    oficina.generar_xml()
    ik = IK()
    q0 = ik(REPOSO, semilla=GEN3_HOME)
    ik.postura = q0
    autoprueba(ik, q0)

    gpu = hay_cuda()
    motor = a.motor if a.motor != "auto" else ("genesis" if gpu else "mujoco")
    print(f"GPU CUDA: {'sí' if gpu else 'no'}  ->  simulador: {motor}")
    sim = SimGenesis(not a.sin_ventana, gpu, q0) if motor == "genesis" else SimMuJoCo(not a.sin_ventana, q0)

    for _ in rutina(sim, ik, q0):
        sim.step()
    for i, dist in enumerate(sim.devueltas):
        print(f"   papelera {i + 1}: quedó a {dist * 100:.1f} cm de su sitio, "
              f"{'de pie' if sim.papelera(i)[1][2, 2] > 0.95 else 'volcada'}")
    en_tolva, devueltas = resultado(sim)
    total = len(BASURA)
    print(f"RESULTADO: {en_tolva}/{total} residuos en la tolva, {devueltas}/{len(TAREAS)} papeleras devueltas a su sitio")
    sim.fin()
    sys.exit(0 if en_tolva == total and devueltas == len(TAREAS) else 1)


class _SimSeca:
    """Simulador 'en seco' (solo cinemática) para la autoprueba: chasis en el origen; la papelera está en
    su sitio y, mientras la pinza está cerrada, se mueve rígidamente con ella."""
    dt = 0.05

    def __init__(self, ik, q, dy=0.0):
        self.ik, self.q, self.cerrada, self.dy = ik, q, False, dy

    def chasis(self):
        return np.zeros(3), np.eye(3)

    def papelera(self, i):
        if not self.cerrada:
            return np.array([BRAZO_X, LADO_Y + self.dy, -0.193]), np.eye(3)   # piso en z=-0.193 del chasis
        tcp, Rt = self.ik.tcp(self.q)
        Rp = Rt @ R_PINZA.T
        return tcp - Rp @ AGARRE, Rp

    def brazo(self, q):
        self.q = q

    def pinza(self, x):
        self.cerrada = x > DEDOS_CERRADOS / 2


def autoprueba(ik, q0):
    """Recorre toda la trayectoria del brazo de un ciclo (agarre, arco, volteo de 180°, regreso) solo con
    la cinemática y falla si la IK no encuentra solución continua en algún punto (cambio brusco de postura)."""
    ik.avisar_como_error = True
    with contextlib.redirect_stdout(io.StringIO()):                 # sin los mensajes de la rutina
        for dy in (0.0, -0.05, 0.05):                               # el carro nunca para exactamente donde debe
            for _ in vaciar_papelera(_SimSeca(ik, q0, dy), ik, q0, 0):
                pass
    ik.avisar_como_error = False


if __name__ == "__main__":
    main()
