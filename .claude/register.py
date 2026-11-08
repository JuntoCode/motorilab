# Usage: python .claude/register.py <catalogId>=<file> ...  — adds load() to catalog entries.
import re, sys
p = 'js/catalog.js'
s = open(p, encoding='utf-8').read()
for arg in sys.argv[1:]:
    i, f = arg.split('=')
    pat = re.compile(r"(\{ id: '%s', short: '[^']*', vehicle: '[^']*') \}" % i)
    assert pat.search(s), i
    s = pat.sub(lambda m: m.group(1) + ", load: () => import('./engines/%s.js') }" % f, s)
open(p, 'w', encoding='utf-8').write(s)
print(s.count('load: ()'), 'engines ready')
