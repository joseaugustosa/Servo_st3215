from pathlib import Path

p = Path(__file__).resolve().parents[1] / "ServoDriverST" / "ARM_PAGE.h"
t = p.read_text(encoding="utf-8")
start = t.index("R\"rawliteral(") + len("R\"rawliteral(")
end = t.index(")rawliteral\"")
html = t[start:end]
out = Path(__file__).resolve().parent / "assets" / "arm_web_preview.html"
out.write_text(html, encoding="utf-8")
print(f"Wrote {len(html)} bytes -> {out}")
