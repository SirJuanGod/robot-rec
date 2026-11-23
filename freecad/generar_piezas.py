# Genera las piezas del ROBOT-REC (base diferencial + tolva + pinza) en FreeCAD y exporta un STL por pieza.
# El brazo es un Kinova Gen3 (mallas oficiales en meshes/gen3, de MuJoCo Menagerie).
# Uso:  "C:\Program Files\FreeCAD 1.1\bin\freecadcmd.exe" freecad\generar_piezas.py
# Unidades: mm. Cada pieza está en el marco local de su cuerpo en robot_rec.xml.
# Marco del chasis: z=0 a media altura del cuerpo (cuerpo de z=-90 a +90, cubierta en +90).
import os
import FreeCAD as App
import Part
import Mesh

V = App.Vector
EJE_Y = V(0, 1, 0)
HERE = os.path.dirname(os.path.abspath(__file__))
OUT = os.path.join(HERE, "..", "meshes")
os.makedirs(OUT, exist_ok=True)

CH_L, CH_W, CH_H = 1100.0, 720.0, 180.0  # cuerpo de la base
CHAFLAN = 90.0                           # esquinas achaflanadas
RUEDA_R, RUEDA_A, RUEDA_Y = 120.0, 60.0, 315.0   # ruedas motrices, en el centro (x=0)
PASO = (300.0, 270.0)                    # paso de rueda: largo en x, desde |y|=270 hacia afuera
LOCAS = [(sx * 450.0, sy * 270.0) for sx in (1, -1) for sy in (1, -1)]   # ruedas locas
TOLVA = dict(x0=-540.0, x1=80.0, w=660.0, h=250.0, pared=12.0)            # sobre la cubierta
BRAZO_X = 320.0
TOP = CH_H / 2


def box(lx, ly, lz, cx=0, cy=0, cz=0):
    return Part.makeBox(lx, ly, lz, V(cx - lx / 2, cy - ly / 2, cz - lz / 2))


def cyl(r, h, base=V(0, 0, 0), axis=V(0, 0, 1)):
    return Part.makeCylinder(r, h, base, axis)


def cyl_y(r, largo, cx=0, cy=0, cz=0):
    return cyl(r, largo, V(cx, cy - largo / 2, cz), EJE_Y)


def cyl_x(r, largo, cx=0, cy=0, cz=0):
    return cyl(r, largo, V(cx - largo / 2, cy, cz), V(1, 0, 0))


def verticales(forma):
    return [e for e in forma.Edges if abs(e.Vertexes[0].Z - e.Vertexes[-1].Z) > 1]


def redondear(forma, r, solo_verticales=False):
    return forma.makeFillet(r, verticales(forma) if solo_verticales else forma.Edges)


def achaflanado(lx, ly, lz, c, cz=0):
    b = box(lx, ly, lz, cz=cz)
    return b.makeChamfer(c, verticales(b))


def unir(*formas):
    return formas[0].fuse(list(formas[1:])) if len(formas) > 1 else formas[0]


def pasos_de_rueda():
    return unir(*[box(PASO[0], 120, CH_H - 25 + 20, cy=s * (PASO[1] + 60), cz=-12.5 - 10) for s in (1, -1)])


# ---------------- base diferencial ----------------
def chasis():
    cuerpo = achaflanado(CH_L, CH_W, CH_H, CHAFLAN)
    return cuerpo.cut(pasos_de_rueda())


def negro():
    """Banda de caucho perimetral (parachoques), ToF y placas de las ruedas locas."""
    banda = achaflanado(CH_L + 24, CH_W + 24, 45, CHAFLAN + 5, cz=-50).cut(
        achaflanado(CH_L - 2, CH_W - 2, 60, CHAFLAN, cz=-50)).cut(pasos_de_rueda())
    tof = [cyl_x(12, 10, cx=s * (CH_L / 2 + 14), cy=y, cz=-50) for s in (1, -1) for y in (-200, 0, 200)]
    placas = [box(80, 80, 6, cx=x, cy=y, cz=-TOP - 3) for x, y in LOCAS]
    return unir(banda, *tof, *placas)


def detalles():
    """Tapas de los compartimentos (baterías / electrónica) en la cubierta delantera."""
    return unir(*[redondear(box(140, 110, 4, cx=430, cy=s * 230, cz=TOP + 2), 10, solo_verticales=True) for s in (1, -1)])


def luces():
    return unir(*[box(6, 90, 18, cx=CH_L / 2 + 3, cy=s * 210, cz=40) for s in (1, -1)])


def placa_brazo():
    return redondear(box(200, 200, 12, cx=BRAZO_X, cz=TOP + 6), 4)


def paro_base():
    return box(70, 70, 50, cx=150, cy=-260, cz=TOP + 25)


def paro_seta():
    return unir(cyl(14, 15, V(150, -260, TOP + 50)), cyl(26, 14, V(150, -260, TOP + 62)))


def tolva():
    t = TOLVA
    largo, cx = t["x1"] - t["x0"], (t["x0"] + t["x1"]) / 2
    ext = redondear(box(largo, t["w"], t["h"], cx=cx, cz=TOP + t["h"] / 2), 30, solo_verticales=True)
    hueco = box(largo - 2 * t["pared"], t["w"] - 2 * t["pared"], t["h"] + 10, cx=cx, cz=TOP + t["h"] / 2 + 5)
    return ext.cut(hueco)


def lidar():
    z0 = TOP + TOLVA["h"]
    poste = cyl(16, 140, V(TOLVA["x0"] - 16, 0, z0 - 40))
    soporte = box(40, 60, 20, cx=TOLVA["x0"] - 6, cz=z0)
    sensor = unir(cyl(38, 30, V(TOLVA["x0"] - 16, 0, z0 + 100)), cyl(30, 18, V(TOLVA["x0"] - 16, 0, z0 + 130)))
    return unir(poste, soporte, sensor)


def rueda():
    return redondear(cyl_y(RUEDA_R, RUEDA_A), 14)


def llanta():
    disco = cyl_y(82, RUEDA_A + 4)
    cubo = cyl_y(28, RUEDA_A + 14)
    pernos = [cyl_y(6, RUEDA_A + 10, cx=50 * c, cz=50 * s)
              for c, s in ((1, 0), (0.309, 0.951), (-0.809, 0.588), (-0.809, -0.588), (0.309, -0.951))]
    return unir(disco, cubo, *pernos)


def horquilla():
    """Horquilla de la rueda loca; origen en el eje de giro (bajo la placa), avance de 25 mm."""
    plato = cyl(32, 8, V(0, 0, -8))
    lados = [box(40, 4, 52, cx=-20, cy=s * 17, cz=-34) for s in (1, -1)]
    return unir(plato, *lados)


def rueda_loca():
    return redondear(cyl_y(45, 26), 5)


# ---------------- papelera de malla metálica (origen: centro de la base) ----------------
# Troncopiramidal: base 170 mm, boca 210 mm, alto 200 mm. Arriba un aro rígido vertical (40 mm, 10 mm de
# espesor) con un reborde enrollado (22 mm) en el borde: la pinza aprieta el aro y el reborde queda encima.
PAPELERA = dict(base=170.0, boca=210.0, z_aro=140.0, alto=200.0, aro=10.0, reborde=22.0)


def cuadro(lado, z):
    h = lado / 2
    return Part.makePolygon([V(-h, -h, z), V(h, -h, z), V(h, h, z), V(-h, h, z), V(-h, -h, z)])


def anillo(ext, esp, z0, alto):
    return box(ext, ext, alto, cz=z0 + alto / 2).cut(box(ext - 2 * esp, ext - 2 * esp, alto + 2, cz=z0 + alto / 2))


def papelera_malla():
    """Paredes de malla (en la simulación se dibujan semitransparentes)."""
    c = PAPELERA
    k = (c["boca"] - c["base"]) / c["z_aro"]                         # ensanche por mm de altura
    ext = Part.makeLoft([cuadro(c["base"], 0), cuadro(c["boca"], c["z_aro"])], True)
    inte = Part.makeLoft([cuadro(c["base"] - 4, 2), cuadro(c["boca"] - 4 + k, c["z_aro"] + 1)], True)
    return ext.cut(inte)


def papelera_aro():
    """Base sólida, aro superior y reborde (opacos)."""
    c = PAPELERA
    base = redondear(box(c["base"] + 6, c["base"] + 6, 8, cz=4), 2)
    aro = anillo(c["boca"] + c["aro"], c["aro"], c["z_aro"], c["alto"] - 10 - c["z_aro"])
    reborde = anillo(c["boca"] + c["reborde"], c["reborde"], c["alto"] - 10, 10)
    return unir(base, aro, reborde)


# ---------------- pinza (se acopla a la brida del Gen3) ----------------
def pinza():
    return cyl(38, 12).fuse(redondear(box(130, 56, 30, cz=27), 6))


def dedo():
    return redondear(box(12, 40, 70, cz=35), 3)                       # origen en la cara de la pinza


def almohadilla():
    return box(12, 36, 40, cx=-12, cz=48)                             # caucho de 12 mm, mira al centro (-X)


PIEZAS = {
    "chasis": chasis(), "chasis_negro": negro(), "detalles": detalles(), "luces": luces(),
    "placa_brazo": placa_brazo(), "paro_base": paro_base(), "paro_seta": paro_seta(),
    "tolva": tolva(), "lidar": lidar(), "rueda": rueda(), "llanta": llanta(),
    "horquilla": horquilla(), "rueda_loca": rueda_loca(),
    "pinza": pinza(), "dedo": dedo(), "almohadilla": almohadilla(),
    "papelera_malla": papelera_malla(), "papelera_aro": papelera_aro(),
}

for f in os.listdir(OUT):            # borrar STL de versiones anteriores (meshes/gen3 no se toca)
    if f.endswith(".stl"):
        os.remove(os.path.join(OUT, f))

doc = App.newDocument("ROBOT_REC")
for nombre, forma in PIEZAS.items():
    assert forma.isValid() and forma.Volume > 0, nombre
    obj = doc.addObject("Part::Feature", nombre)
    obj.Shape = forma
    malla = Mesh.Mesh(forma.tessellate(0.5))
    malla.write(os.path.join(OUT, nombre + ".stl"))
    print(f"{nombre}.stl  volumen={forma.Volume / 1e6:.3f} L  caras={malla.CountFacets}")

doc.recompute()
doc.saveAs(os.path.join(HERE, "robot_rec_piezas.FCStd"))
t = TOLVA
print(f"Capacidad de la tolva: {(t['x1'] - t['x0'] - 2 * t['pared']) * (t['w'] - 2 * t['pared']) * t['h'] / 1e6:.0f} L")
print("OK ->", os.path.abspath(OUT))
