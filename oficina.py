"""Escenario de oficina para el ROBOT-REC.

Genera `oficina.xml` (MJCF con paredes, ventanas, puerta, escritorios, sillas, archivadores, plantas...)
que cargan tanto MuJoCo (escena.xml lo incluye) como Genesis, y define dónde están las papeleras, qué
basura tiene cada una y por qué pasillos llega el robot a cada una.

Planta (m): sala de 10 x 8.5. Dos filas de escritorios (y=6.0 y y=3.6) dejan tres pasillos horizontales
(y≈1.6, 4.9 y 7.7) unidos por dos pasillos verticales (x≈0.9 y x≈9.15). El robot mide 1.1 x 0.72 m.

    python oficina.py      # solo regenera oficina.xml
"""
import math
import os

AQUI = os.path.dirname(os.path.abspath(__file__))
ANCHO, LARGO, ALTO_PARED = 10.0, 8.5, 2.6

# Pose inicial del robot: (x, y, rumbo)
INICIO = (0.9, 4.9, 0.0)

# Una tarea por papelera. El robot pasa con la papelera a su IZQUIERDA avanzando en dirección `rumbo`
# (la papelera se orienta igual, con la pared -y mirando al robot). `ruta` son los puntos de paso por los
# pasillos desde donde terminó la tarea anterior hasta la recta de aproximación.
TAREAS = [
    dict(papelera=(3.75, 5.45), rumbo=0.0, ruta=[]),                                # pasillo central, entre escritorios
    dict(papelera=(8.60, 2.90), rumbo=math.pi / 2, ruta=[(9.15, 4.9), (9.15, 1.6)]),  # junto al último escritorio
    dict(papelera=(6.25, 7.15), rumbo=math.pi, ruta=[(9.15, 7.7)]),                 # detrás de la fila norte
    dict(papelera=(4.00, 2.15), rumbo=0.0, ruta=[(0.9, 7.7), (0.9, 1.6)]),          # pasillo sur, junto a las sillas
]

# Basura de cada papelera: (tipo, medidas, masa kg, color). Tipos: "bola" (radio), "lata" (radio, alto),
# "caja" (largo, ancho, alto). Todo cabe en el fondo (15 cm) y queda por debajo del dedo interior de la pinza.
PAPEL, CARTON, ROJO, AZUL, VERDE, GRIS = (0.95, 0.95, 0.9), (0.72, 0.55, 0.35), (0.85, 0.15, 0.15), \
    (0.2, 0.4, 0.85), (0.25, 0.65, 0.3), (0.6, 0.6, 0.62)
BASURA = [
    [("bola", 0.035, 0.02, PAPEL), ("lata", (0.033, 0.11), 0.03, ROJO), ("caja", (0.06, 0.045, 0.03), 0.04, CARTON)],
    [("bola", 0.025, 0.01, PAPEL), ("bola", 0.04, 0.03, PAPEL), ("caja", (0.05, 0.05, 0.05), 0.08, AZUL),
     ("lata", (0.026, 0.09), 0.02, GRIS)],
    [("caja", (0.07, 0.05, 0.02), 0.03, CARTON), ("lata", (0.033, 0.11), 0.35, VERDE)],          # lata llena
    [("bola", 0.03, 0.02, PAPEL), ("caja", (0.04, 0.03, 0.03), 0.02, ROJO), ("bola", 0.02, 0.01, PAPEL),
     ("caja", (0.065, 0.04, 0.04), 0.12, GRIS)],
]
# huecos en el fondo de la papelera (marco de la papelera); el lado -y es el del dedo interior de la pinza
HUECOS = [(-0.045, 0.035), (0.045, 0.035), (-0.045, -0.035), (0.045, -0.035)]


def basura_inicial():
    """[(tipo, medidas, masa, color, (x, y, z) en el mundo)] de toda la basura, dentro de su papelera."""
    out = []
    for t, items in zip(TAREAS, BASURA):
        (px, py), c, s = t["papelera"], math.cos(t["rumbo"]), math.sin(t["rumbo"])
        for (tipo, med, masa, color), (hx, hy) in zip(items, HUECOS):
            alto = 2 * med if tipo == "bola" else med[1] if tipo == "lata" else med[2]
            out.append((tipo, med, masa, color, (px + c * hx - s * hy, py + s * hx + c * hy, 0.008 + alto / 2 + 0.003)))
    return out


# ---------------- generación de oficina.xml ----------------
def _caja(nombre, centro, medidas, material, colision=True):
    (x, y, z), (lx, ly, lz) = centro, medidas
    col = "" if colision else ' contype="0" conaffinity="0"'
    return (f'      <geom name="{nombre}" type="box" pos="{x:.3f} {y:.3f} {z:.3f}" '
            f'size="{lx / 2:.3f} {ly / 2:.3f} {lz / 2:.3f}" material="of_{material}"{col}/>')


def _cil(nombre, centro, r, alto, material, colision=True):
    x, y, z = centro
    col = "" if colision else ' contype="0" conaffinity="0"'
    return (f'      <geom name="{nombre}" type="cylinder" pos="{x:.3f} {y:.3f} {z:.3f}" '
            f'size="{r:.3f} {alto / 2:.3f}" material="of_{material}"{col}/>')


def _escritorio(n, x, y, silla_dy):
    g = [_caja(f"{n}_tablero", (x, y, 0.735), (1.4, 0.7, 0.03), "madera")]
    g += [_caja(f"{n}_pata{i}", (x + sx * 0.66, y + sy * 0.31, 0.36), (0.04, 0.04, 0.72), "metal")
          for i, (sx, sy) in enumerate(((1, 1), (1, -1), (-1, 1), (-1, -1)))]
    g.append(_caja(f"{n}_cajonera", (x + 0.42, y, 0.32), (0.42, 0.55, 0.6), "gris_claro"))
    lado = math.copysign(1, silla_dy)                 # +1: la silla está al norte del escritorio
    g.append(_caja(f"{n}_monitor", (x - 0.1, y - lado * 0.22, 1.0), (0.55, 0.03, 0.33), "negro", False))
    g.append(_caja(f"{n}_pie", (x - 0.1, y - lado * 0.22, 0.8), (0.06, 0.06, 0.1), "negro", False))
    g.append(_caja(f"{n}_teclado", (x - 0.1, y + lado * 0.15, 0.755), (0.45, 0.15, 0.015), "gris_oscuro", False))
    # silla (frente al escritorio, del lado del pasillo donde no circula el robot)
    sx, sy = x - 0.1, y + silla_dy
    g += [_cil(f"{n}_silla_base", (sx, sy, 0.03), 0.3, 0.04, "negro"),
          _cil(f"{n}_silla_poste", (sx, sy, 0.24), 0.03, 0.4, "metal"),
          _caja(f"{n}_silla_asiento", (sx, sy, 0.46), (0.48, 0.46, 0.07), "tela"),
          _caja(f"{n}_silla_respaldo", (sx, sy + math.copysign(0.24, silla_dy), 0.75), (0.46, 0.05, 0.55), "tela")]
    return g


def _planta(n, x, y):
    return [_cil(f"{n}_maceta", (x, y, 0.2), 0.2, 0.4, "maceta"),
            f'      <geom name="{n}_hojas" type="sphere" pos="{x} {y} 0.75" size="0.35" material="of_hojas"/>',
            f'      <geom name="{n}_hojas2" type="sphere" pos="{x + 0.1} {y - 0.08} 1.05" size="0.22" material="of_hojas"/>']


def generar_xml():
    W, L, H, e = ANCHO, LARGO, ALTO_PARED, 0.1
    g = [  # paredes
        _caja("pared_sur", (W / 2, -e / 2, H / 2), (W + 2 * e, e, H), "pared"),
        _caja("pared_norte", (W / 2, L + e / 2, H / 2), (W + 2 * e, e, H), "pared"),
        _caja("pared_oeste", (-e / 2, L / 2, H / 2), (e, L, H), "pared"),
        _caja("pared_este", (W + e / 2, L / 2, H / 2), (e, L, H), "pared"),
        _caja("zocalo_sur", (W / 2, 0.01, 0.05), (W, 0.02, 0.1), "gris_oscuro", False),
        _caja("zocalo_norte", (W / 2, L - 0.01, 0.05), (W, 0.02, 0.1), "gris_oscuro", False),
    ]
    g += [_caja(f"ventana{i}", (x, L - 0.005, 1.5), (1.6, 0.02, 1.1), "vidrio", False) for i, x in enumerate((2.0, 5.0, 8.0))]
    g += [_caja("puerta", (W - 0.005, 6.5, 1.05), (0.02, 0.95, 2.1), "puerta", False),
          _caja("pizarra", (0.005, 4.9, 1.45), (0.02, 1.8, 0.9), "blanco", False)]
    for i, x in enumerate((2.5, 5.0, 7.5)):
        g += _escritorio(f"escA{i}", x, 6.0, 0.6)     # fila norte, sillas al norte
        g += _escritorio(f"escB{i}", x, 3.6, -0.6)    # fila sur, sillas al sur
    g += [_caja(f"archivador{i}", (x, 0.25, 0.6), (0.9, 0.45, 1.2), "gris_claro") for i, x in enumerate((5.6, 6.6, 7.6))]
    g += [_caja("estante", (2.6, 0.2, 0.9), (1.2, 0.35, 1.8), "madera")]
    # plantas fuera de los pasillos y de las esquinas donde gira el robot
    g += _planta("planta1", 0.4, 0.4) + _planta("planta2", 4.4, 0.35) + _planta("planta3", 9.6, 0.4)

    xml = f'''<!-- Generado por oficina.py: no editar a mano. Oficina de {W} x {L} m para el ROBOT-REC. -->
<mujoco model="oficina">
  <asset>
    <material name="of_pared" rgba="0.88 0.87 0.83 1"/>
    <material name="of_madera" rgba="0.62 0.45 0.3 1"/>
    <material name="of_metal" rgba="0.55 0.56 0.58 1"/>
    <material name="of_gris_claro" rgba="0.75 0.76 0.78 1"/>
    <material name="of_gris_oscuro" rgba="0.25 0.26 0.28 1"/>
    <material name="of_negro" rgba="0.08 0.08 0.09 1"/>
    <material name="of_tela" rgba="0.2 0.27 0.45 1"/>
    <material name="of_vidrio" rgba="0.6 0.8 0.95 0.6"/>
    <material name="of_puerta" rgba="0.45 0.3 0.2 1"/>
    <material name="of_blanco" rgba="0.97 0.97 0.97 1"/>
    <material name="of_maceta" rgba="0.6 0.35 0.25 1"/>
    <material name="of_hojas" rgba="0.2 0.55 0.25 1"/>
  </asset>
  <worldbody>
    <light pos="{W / 4} {L / 2} 2.5" dir="0 0 -1" diffuse="0.5 0.5 0.5"/>
    <light pos="{3 * W / 4} {L / 2} 2.5" dir="0 0 -1" diffuse="0.5 0.5 0.5"/>
    <body name="oficina">
{chr(10).join(g)}
    </body>
  </worldbody>
</mujoco>
'''
    with open(os.path.join(AQUI, "oficina.xml"), "w", encoding="utf-8") as f:
        f.write(xml)


if __name__ == "__main__":
    generar_xml()
    print("oficina.xml generado")
