#!/usr/bin/env python3
"""Negative controls for the shell verifier. No game data/compiler/browser used."""
from pathlib import Path
import os
import shutil
import subprocess
import tempfile
import unittest

SCRIPT = Path(__file__).with_name('public_verify.sh')


class PublicVerifyTests(unittest.TestCase):
    def test_native_helpers_reject_symlink_into_checkout(self):
        root = SCRIPT.parent.parent.resolve()
        with tempfile.TemporaryDirectory(prefix='i76-native-paths-') as tmp:
            parent = Path(tmp)
            link = parent / 'checkout-link'
            link.symlink_to(root / 'tools', target_is_directory=True)
            assets = parent / 'synthetic-assets'
            assets.mkdir()
            (assets / 'nitro.zfs').touch()
            (assets / 'nitro.zix').touch()
            env = dict(os.environ, OUT=str(link), NITRO_APP=str(assets), CC='/nonexistent-compiler')
            for script, args in [('build_probe.sh', ['--list']), ('frame_gate.sh', [str(link)])]:
                run = subprocess.run(['bash', str(SCRIPT.with_name(script)), *args], env=env,
                                     capture_output=True, text=True, timeout=10)
                self.assertEqual(run.returncode, 2, run.stdout + run.stderr)
                self.assertIn('outside', run.stderr)

    def test_early_unit_failure_propagates_in_both_modes(self):
        for mode in ['--units', '--all']:
            with self.subTest(mode=mode), tempfile.TemporaryDirectory(prefix='i76-verifier-') as tmp:
                parent = Path(tmp)
                root = parent / 'source'
                (root / 'tools').mkdir(parents=True)
                shutil.copyfile(SCRIPT, root / 'tools/public_verify.sh')
                builder = root / 'tools/build_probe.sh'
                builder.write_text('''#!/usr/bin/env bash
set -eu
code=0
[[ "$1" != raster_test ]] || code=17
printf '#!/bin/sh\\nexit %s\\n' "$code" > "$OUT/$1"
chmod +x "$OUT/$1"
''')
                builder.chmod(0o755)
                assets = parent / 'synthetic-assets'
                assets.mkdir()
                # Prerequisite placeholders only, never decoded or distributed.
                (assets / 'nitro.zfs').write_bytes(b'')
                (assets / 'nitro.zix').write_bytes(b'')
                env = dict(os.environ, I76_VERIFY_OUT=str(parent / 'output'), NITRO_APP=str(assets))
                run = subprocess.run(['bash', str(root / 'tools/public_verify.sh'), mode],
                                     env=env, capture_output=True, text=True, timeout=10)
                self.assertEqual(run.returncode, 17, run.stdout + run.stderr)
                self.assertNotIn('== build-pixel-history ==', run.stdout)


if __name__ == '__main__':
    unittest.main()
