"""Comunicação: simulação, Serial, Wi‑Fi (HTTP), ou ambos em simultâneo."""

from __future__ import annotations

from abc import ABC, abstractmethod
from typing import List, Optional
import json
from pathlib import Path

import yaml


class CommBackend(ABC):
    @abstractmethod
    def connect(self) -> None: ...

    @abstractmethod
    def disconnect(self) -> None: ...

    @abstractmethod
    def send_positions(self, commands: List[dict]) -> None: ...

    @property
    @abstractmethod
    def connected(self) -> bool: ...

    @property
    def label(self) -> str:
        return type(self).__name__


class SimulationBackend(CommBackend):
    def __init__(self):
        self._connected = True
        self.last_commands: List[dict] = []

    def connect(self) -> None:
        self._connected = True

    def disconnect(self) -> None:
        self._connected = False

    def send_positions(self, commands: List[dict]) -> None:
        self.last_commands = list(commands)

    @property
    def connected(self) -> bool:
        return self._connected

    @property
    def label(self) -> str:
        return "Simulação"


class WifiBackend(CommBackend):
    """POST /api/arm  JSON: {servos:[{id,pos,speed,acc},...]}"""

    def __init__(self, base_url: str):
        self.base_url = base_url.rstrip("/")
        self._connected = False

    def connect(self) -> None:
        import urllib.request

        try:
            urllib.request.urlopen(self.base_url + "/", timeout=2)
            self._connected = True
        except Exception as exc:
            self._connected = False
            raise ConnectionError(f"Wi‑Fi/HTTP falhou ({self.base_url}): {exc}") from exc

    def disconnect(self) -> None:
        self._connected = False

    def send_positions(self, commands: List[dict]) -> None:
        if not self._connected:
            raise ConnectionError("Wi‑Fi não ligado")
        import urllib.request

        payload = json.dumps(
            {
                "servos": [
                    {"id": c["id"], "pos": c["pos"], "speed": c["speed"], "acc": c["acc"]}
                    for c in commands
                ]
            }
        ).encode("utf-8")
        req = urllib.request.Request(
            self.base_url + "/api/arm",
            data=payload,
            headers={"Content-Type": "application/json"},
            method="POST",
        )
        urllib.request.urlopen(req, timeout=2)

    @property
    def connected(self) -> bool:
        return self._connected

    @property
    def label(self) -> str:
        return f"Wi‑Fi ({self.base_url})"


class SerialBackend(CommBackend):
    """Linha ASCII: ARM id,pos,spd,acc;id2,...\\n"""

    def __init__(self, port: str, baud: int = 115200):
        self.port = port
        self.baud = baud
        self._ser = None

    def connect(self) -> None:
        import serial

        self._ser = serial.Serial(self.port, self.baud, timeout=1)
        self._ser.reset_input_buffer()

    def disconnect(self) -> None:
        if self._ser and self._ser.is_open:
            self._ser.close()
        self._ser = None

    def send_positions(self, commands: List[dict]) -> None:
        if not self._ser or not self._ser.is_open:
            raise ConnectionError("Serial não ligada")
        parts = [f"{c['id']},{c['pos']},{c['speed']},{c['acc']}" for c in commands]
        self._ser.write(("ARM " + ";".join(parts) + "\n").encode("ascii"))

    @property
    def connected(self) -> bool:
        return bool(self._ser and self._ser.is_open)

    @property
    def label(self) -> str:
        return f"Serial ({self.port}@{self.baud})"


class MultiBackend(CommBackend):
    """Envia para todos os backends ativos (ex.: Serial + Wi‑Fi)."""

    def __init__(self, backends: List[CommBackend]):
        self.backends = backends

    def connect(self) -> None:
        errors = []
        for b in self.backends:
            try:
                b.connect()
            except Exception as exc:
                errors.append(f"{b.label}: {exc}")
        if not any(b.connected for b in self.backends):
            raise ConnectionError("Nenhuma ligação ativa: " + " | ".join(errors))

    def disconnect(self) -> None:
        for b in self.backends:
            try:
                b.disconnect()
            except Exception:
                pass

    def send_positions(self, commands: List[dict]) -> None:
        errors = []
        sent = 0
        for b in self.backends:
            if not b.connected:
                continue
            try:
                b.send_positions(commands)
                sent += 1
            except Exception as exc:
                errors.append(f"{b.label}: {exc}")
        if sent == 0:
            raise ConnectionError("Falha no envio: " + (" | ".join(errors) or "sem backends"))

    @property
    def connected(self) -> bool:
        return any(b.connected for b in self.backends)

    @property
    def label(self) -> str:
        parts = [b.label for b in self.backends if b.connected]
        return " + ".join(parts) if parts else "Desligado"


def list_serial_ports() -> List[str]:
    try:
        from serial.tools import list_ports

        return [p.device for p in list_ports.comports()]
    except Exception:
        return []


def create_backend(
    mode: str,
    wifi_url: str = "http://192.168.4.1",
    serial_port: str = "COM3",
    serial_baud: int = 115200,
) -> CommBackend:
    mode = (mode or "simulation").lower()
    if mode == "simulation":
        return SimulationBackend()
    if mode == "serial":
        return SerialBackend(serial_port, serial_baud)
    if mode in ("wifi", "http"):
        return WifiBackend(wifi_url)
    if mode == "both":
        return MultiBackend(
            [SerialBackend(serial_port, serial_baud), WifiBackend(wifi_url)]
        )
    return SimulationBackend()


def create_backend_from_config(config_path: Optional[str | Path] = None) -> CommBackend:
    if config_path is None:
        config_path = Path(__file__).with_name("config.yaml")
    with open(config_path, "r", encoding="utf-8") as f:
        cfg = yaml.safe_load(f)
    conn = cfg["connection"]
    return create_backend(
        mode=conn.get("mode", "simulation"),
        wifi_url=conn.get("wifi_url", conn.get("http_base_url", "http://192.168.4.1")),
        serial_port=conn.get("serial_port", "COM3"),
        serial_baud=int(conn.get("serial_baud", 115200)),
    )
