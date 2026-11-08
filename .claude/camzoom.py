# Usage: python .claude/camzoom.py <engine-file> <distance factor> [target dy]
# Scales the default camera distance of an engine module and optionally moves the target up/down.
import re, sys
path, k = sys.argv[1], float(sys.argv[2])
dy = float(sys.argv[3]) if len(sys.argv) > 3 else 0.0
s = open(path, encoding='utf-8').read()
pat = re.compile(r"camera:\s*\{\s*position:\s*\[([^\]]+)\],\s*target:\s*\[([^\]]+)\]\s*\}")
m = pat.search(s)
p = [float(v) for v in m.group(1).split(',')]
t = [float(v) for v in m.group(2).split(',')]
t[1] += dy
np_ = [round(t[i] + (p[i] - t[i]) * k, 2) for i in range(3)]
np_[1] = round(np_[1] + dy, 2)
new = f"camera: {{ position: [{', '.join(map(str, np_))}], target: [{', '.join(str(round(v, 2)) for v in t)}] }}"
s = s[:m.start()] + new + s[m.end():]
open(path, 'w', encoding='utf-8').write(s)
print(path, '->', new)
