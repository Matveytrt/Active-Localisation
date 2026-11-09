"""Verify that offline consolidation preserved real observations and constraints."""
import argparse
import hashlib
import json
from pathlib import Path
import sqlite3

parser = argparse.ArgumentParser()
parser.add_argument('--online', type=Path, required=True)
parser.add_argument('--global-db', type=Path, required=True)
args = parser.parse_args()


def digest(path, query):
    con = sqlite3.connect(f'file:{path}?mode=ro', uri=True)
    value = hashlib.sha256()
    count = 0
    for row in con.execute(query):
        for item in row:
            raw = item if isinstance(item, bytes) else repr(item).encode()
            value.update(len(raw).to_bytes(8, 'little'))
            value.update(raw)
        count += 1
    con.close()
    return {'rows': count, 'sha256': value.hexdigest()}


queries = {'sensor_data': 'SELECT * FROM Data ORDER BY id',
           'constraints': 'SELECT * FROM Link ORDER BY rowid',
           'raw_poses': 'SELECT id,pose FROM Node ORDER BY id'}
result = {'status': 'passed', 'online': str(args.online), 'optimized': str(args.global_db), 'tables': {}}
for name, query in queries.items():
    before, after = digest(args.online, query), digest(args.global_db, query)
    assert before == after, (name, before, after)
    result['tables'][name] = before
args.global_db.with_suffix('.data-audit.json').write_text(json.dumps(result, indent=2))
print(json.dumps(result, indent=2))
