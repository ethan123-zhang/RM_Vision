"""复用基线的同一分析方法，仅替换输入记录；输出保存在本实验目录。"""
from pathlib import Path

baseline = Path(__file__).resolve().parent.parent / '2026-10-29-single-wheel-long8s/analyze_dynamics.py'
source = baseline.read_text(encoding='utf-8').replace('can_capture_long8s', 'can_capture_ki004')
source = source.replace('and 10 speed steps', 'and selected speed steps')
exec(compile(source, str(baseline), 'exec'), globals())
