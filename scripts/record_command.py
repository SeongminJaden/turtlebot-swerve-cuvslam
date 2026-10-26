"""Record an explicit command, output and exit status for reproducible lessons."""
import argparse
import datetime
import hashlib
import json
from pathlib import Path
import subprocess
import sys

p = argparse.ArgumentParser()
p.add_argument('--output', type=Path, required=True)
p.add_argument('command', nargs=argparse.REMAINDER)
a = p.parse_args()
command = a.command[1:] if a.command[:1] == ['--'] else a.command
if not command:
    p.error('Provide a command after --')
a.output.mkdir(parents=True, exist_ok=True)
started = datetime.datetime.now(datetime.timezone.utc).isoformat()
result = subprocess.run(command, stdout=subprocess.PIPE, stderr=subprocess.STDOUT, text=True)
print(result.stdout, end='')
log = a.output/'output.log'
log.write_text(result.stdout)
report = {'started_utc': started, 'cwd': str(Path.cwd()), 'command': command,
          'exit_code': result.returncode, 'output_sha256': hashlib.sha256(log.read_bytes()).hexdigest(),
          'git_commit': subprocess.check_output(['git','rev-parse','HEAD'],text=True).strip(),
          'git_status': subprocess.check_output(['git','status','--short'],text=True)}
(a.output/'manifest.json').write_text(json.dumps(report, ensure_ascii=False, indent=2))
sys.exit(result.returncode)
