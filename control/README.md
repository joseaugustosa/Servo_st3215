# Controlo gráfico — Braço 4 DOF + Garra (ST3215)

Software em Python para:
- **Cinemática inversa (IK)** e direta (FK) — juntas **a1, a2, a3, a4** + garra
- **Valores editáveis** na GUI (L1–L4, IDs, offsets, sinais)
- **Ligação Serial e/ou Wi‑Fi** (ou ambos)
- Vista 3D (stick figure) pronta para modelo 3D real

A vista 3D atual é um modelo esquemático. Quando tiveres o modelo 3D do braço, substitui a desenho em `app.py` (`_redraw`) pelo loader do teu mesh (ex.: STL/GLB).

## Arranque rápido

```bash
cd control
pip install -r requirements.txt
python app.py
```

## Configuração

Edita `config.yaml`:

| Campo | Significado |
|--------|-------------|
| `arm.L1..L4` | Comprimentos dos elos (mm) — editáveis na GUI |
| `arm.servo_ids` | IDs: a1, a2, a3, a4, gripper |
| `arm.joint_sign` / `joint_offset_deg` | Calibração mecânica |
| `connection.mode` | `simulation` \| `serial` \| `wifi` \| `both` |

## Modos da GUI

1. **Cartesiano (IK)** — X, Y, Z + pitch → calcula a1..a4
2. **Articular a1..a4** — cada motor + garra
3. **Parâmetros** — edita L1–L4, IDs, offsets; Aplicar / Guardar
4. **Ligação** — Serial, Wi‑Fi, ou ambos

## Ligação ao hardware

- **simulation** — só visualização
- **serial** — cabo USB; linha `ARM id,pos,spd,acc;...`
- **wifi** — `POST /api/arm` no ESP32
- **both** — envia para Serial e Wi‑Fi ao mesmo tempo

Conversão de ângulo → posição ST3215 (0–4095, centro 2047) em `arm_model.py`.

## Estrutura

```
control/
  app.py           # GUI + vista 3D
  kinematics.py    # IK / FK
  arm_model.py     # estado, limites, conversão servo
  comm.py          # simulação / HTTP / serial
  config.yaml      # geometria e ligação
  requirements.txt
```

## Próximo passo (modelo 3D)

Quando introduzires o 3D do braço:
1. Coloca o ficheiro em `control/assets/` (ex.: `arm.stl`)
2. Troca `_redraw()` para carregar e transformar o mesh com os ângulos das juntas
3. Mantém IK/FK e `comm.py` iguais — a simulação passa a usar a geometria real
