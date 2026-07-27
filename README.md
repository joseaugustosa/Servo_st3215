# Servo_st3215

Workspace para servos ST3215 (Waveshare / Feetech SMS_STS) e braço robótico.

## Conteúdo

| Pasta | Descrição |
|--------|-----------|
| `SCServo/` | Biblioteca Arduino oficial (SMS/STS) |
| `ServoDriverST/` | Firmware ESP32 (Wi‑Fi, web UI, controlo de servos) |
| `control/` | **Software PC** — IK 4DOF + garra, GUI 3D |

## Controlo gráfico do braço

```bash
cd control
pip install -r requirements.txt
python app.py
```

- 4 motores (**a1, a2, a3, a4**) + garra
- Cinemática inversa; parâmetros editáveis; Serial e/ou Wi‑Fi
- Vista 3D em simulação — preparada para modelo 3D real

Detalhes em [`control/README.md`](control/README.md).
