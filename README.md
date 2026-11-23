# ROBOT-REC: base diferencial recolectora + Kinova Gen3 en una oficina (FreeCAD + MuJoCo / Genesis)

Basado en `preset.pdf`. El robot recorre una **oficina** (generada por código) y vacía en su tolva las **papeleras de malla** repartidas por ella. Cada papelera tiene basura de distintas formas, tamaños y masas.

En cada papelera:

1. Llega por los pasillos.
2. Se alinea y pasa con la papelera a su izquierda (carga lateral, como un camión recolector), deteniéndose a su lado.
3. El Gen3 pinza el aro superior de la pared que mira al carro: un dedo por dentro y otro por fuera.
4. La lleva en arco hasta encima de la tolva y la invierte para vaciarla.
5. La devuelve vacía a su sitio.

![oficina](captura_rutina_oficina.png)

## El escenario (`oficina.py` → `oficina.xml`)

- **Sala** de 10 × 8,5 m con ventanas, puerta, pizarra, zócalos y piso de baldosas.
- **Muebles:**
  - 6 escritorios en dos filas, cada uno con monitor, teclado, cajonera y silla.
  - Archivadores, un estante y plantas.
- **Pasillos:** las dos filas de escritorios dejan tres pasillos horizontales (y ≈ 1,6, 4,9 y 7,7 m) unidos por dos verticales (x ≈ 0,9 y 9,15 m).
- **4 papeleras:** en el pasillo central entre escritorios, junto al último escritorio, detrás de la fila norte y en el pasillo sur.
- **Basura variada** (13 piezas en total):
  - Bolas de papel de 2 a 4 cm de radio.
  - Latas de 9 a 11 cm, una de ellas llena (350 g).
  - Cajitas de 4 a 7 cm.

Todo se define arriba en `oficina.py`:
- `TAREAS`: dónde está cada papelera, con qué rumbo se aproxima el robot y los puntos de paso por los pasillos.
- `BASURA`: qué hay en cada papelera.
- `INICIO`: dónde arranca el robot.

`oficina.xml` se regenera solo al ejecutar `simular.py` (no editarlo a mano). Al mover una papelera, deja libres los pasillos (el robot mide 1,1 × 0,72 m y, al girar en el sitio, barre un círculo de ~0,66 m de radio).

## El robot

- **Base de tracción diferencial** de 110×72 cm y 35 kg:
  - **2 ruedas motrices de 24 cm en el centro** y **4 ruedas locas giratorias** en las esquinas.
  - Banda-parachoques con sensores ToF, faros, paro de emergencia y LiDAR 360°.
- **Tolva** de ~95 L sobre la cubierta trasera.
- **Brazo Kinova Gen3** de 7 DOF. Las mallas, inercias y ganancias vienen del modelo oficial de [MuJoCo Menagerie](https://github.com/google-deepmind/mujoco_menagerie/tree/main/kinova_gen3) (licencia BSD-3, ver `meshes/gen3/LICENSE`).
- **Pinza paralela**:
  - Dedos acoplados (piñón-cremallera), 120 N por dedo (del orden de una Robotiq 2F-85).
  - Almohadillas de caucho de 12 mm y palma que hace de tope.
  - Cierra en rampa de 0,6 s.
- **Papeleras** (`papelera.xml`) de malla metálica, troncopiramidales (base 17 cm, boca 21 cm, alto 20 cm):
  - Aro superior rígido de 1 cm de espesor, que es donde se agarran.
  - Reborde de 2,2 cm que queda encima de las almohadillas.

| Archivo | Qué es |
|---|---|
| `oficina.py` | Escenario: genera `oficina.xml` y define papeleras, basura y rutas |
| `freecad/generar_piezas.py` | Modela base, ruedas, ruedas locas, tolva, pinza y papelera en FreeCAD y exporta `meshes/*.stl` + `freecad/robot_rec_piezas.FCStd` |
| `meshes/gen3/` | Mallas STL del Kinova Gen3 (Menagerie) |
| `robot_rec.xml` | Ensamblaje completo del robot (MJCF) |
| `papelera.xml` | Papelera de malla (MJCF); `simular.py` la inserta una vez por cada tarea |
| `escena.xml` | Escena MuJoCo: piso + oficina + robot |
| `simular.py` | Navegación, rutina del brazo, IK del Gen3 y los dos backends (MuJoCo / Genesis) |

## 1. Generar los STL (FreeCAD 1.1)

```bash
"C:\Program Files\FreeCAD 1.1\bin\freecadcmd.exe" freecad\generar_piezas.py
```

## 2. Instalar

```bash
python -m venv .venv
.venv\Scripts\pip install mujoco numpy
```

Solo si tienes GPU NVIDIA con CUDA (para Genesis):

```bash
.venv\Scripts\pip install torch --index-url https://download.pytorch.org/whl/cu124
.venv\Scripts\pip install genesis-world
```

## 3. Ejecutar

```bash
.venv\Scripts\python simular.py
```

Usa **Genesis** si `torch.cuda.is_available()` y **MuJoCo** en caso contrario.
- `--motor mujoco|genesis` fuerza uno de los dos.
- `--sin-ventana` corre sin visor. Al final informa, para cada papelera, a qué distancia de su sitio quedó y si quedó de pie, y cuántos residuos terminaron en la tolva. Sale con código 1 si falta algún residuo o alguna papelera quedó a más de 5 cm o volcada.

Antes de simular, una **autoprueba** recorre solo con la cinemática la trayectoria del brazo en un ciclo (agarre, arco, volteo de 180° y regreso), con el carro parado a ±5 cm de donde debería. Falla si la IK tendría que saltar de postura.

## Notas de diseño (lo que se aprendió ajustando la simulación)

- **Navegación:** usa puntos de paso por los pasillos y giro en el sitio cuando el siguiente punto queda de lado. La aproximación final es un seguidor de línea que empieza 1,5 m antes de la papelera. Si el robot no llega a un punto en 60 s (un obstáculo), lo avisa y salta esa papelera.
- **IK:** es numérica (mínimos cuadrados amortiguados), aprovecha la redundancia de los 7 DOF y usa el modelo MuJoCo como librería de cinemática, también en Genesis. Cada movimiento se resuelve en puntos encadenados (recta, arco o giro de muñeca), así el brazo nunca salta de configuración.
- **Vaciado:** la papelera se voltea 180° alrededor del aro, a lo largo del carro. Con la pinza orientada como al recoger, el Gen3 llega a 180° sin tocar el límite de `joint_6`.
- **Agarre por el aro:** colgada de una sola pared, la papelera hace ~1 N·m de torque sobre la pinza, por eso hacen falta 120 N. El reborde debe sobresalir 6 mm a cada lado; con menos, el aro se escurre entre las almohadillas.
- **Soltar la papelera:** colgada queda inclinada ~16°. Para dejarla, se lleva el **centro de la base** sobre su sitio con el aro 4 mm sobre su altura original (sin apretarla contra el piso). Después la pinza se afloja, baja 1,2 cm para despegar las almohadillas del reborde, abre, **se recentra sobre el aro** y recién entonces sube.
- **Colisiones de la papelera:** las cajas de las paredes inclinadas no deben asomar bajo la base; si asoman, la papelera cojea.
- **Física:** en los dos simuladores se usan cono elíptico (MuJoCo), `impratio` alto, `noslip` y fricción torsional en las almohadillas, para que lo agarrado no resbale lentamente.
