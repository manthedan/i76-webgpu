#!/usr/bin/env python3
"""Asset-free negative controls for the release inventory, not a secret scanner."""
from pathlib import Path
import os
import shutil
import subprocess
import sys
import tempfile
import unittest

CHECKER = Path(__file__).with_name('check_source.py')


class InventoryTests(unittest.TestCase):
    def setUp(self):
        self.tmp = tempfile.TemporaryDirectory(prefix='i76-inventory-')
        self.addCleanup(self.tmp.cleanup)
        self.root = Path(self.tmp.name)
        (self.root / 'tools').mkdir()
        shutil.copyfile(CHECKER, self.root / 'tools/check_source.py')
        self.names = ['PUBLIC_FILES.txt', 'readme.txt', 'tools/check_source.py']
        (self.root / 'readme.txt').write_text('Synthetic source fixture.\n')
        self.manifest(self.names)

    def manifest(self, names):
        (self.root / 'PUBLIC_FILES.txt').write_text(''.join(n + '\n' for n in names))

    def run_check(self, *flags):
        return subprocess.run([sys.executable, str(self.root / 'tools/check_source.py'), *flags],
                              capture_output=True, text=True, timeout=10)

    def test_valid_source(self):
        self.assertEqual(self.run_check().returncode, 0)

    def test_missing_and_extra(self):
        (self.root / 'readme.txt').unlink()
        self.assertNotEqual(self.run_check().returncode, 0)
        (self.root / 'readme.txt').write_text('ok')
        (self.root / 'unlisted.txt').write_text('not approved')
        self.assertNotEqual(self.run_check('--working-tree').returncode, 0)

    def test_duplicate_and_unsafe_paths(self):
        for extra in ['readme.txt', '../outside', '/absolute', 'a/../b', './readme.txt']:
            with self.subTest(extra=extra):
                self.manifest(sorted(self.names + [extra]))
                self.assertNotEqual(self.run_check().returncode, 0)

    def test_binary_and_invalid_utf8(self):
        for b in [b'\x00binary', b'\xff']:
            with self.subTest(b=b):
                (self.root / 'readme.txt').write_bytes(b)
                self.assertNotEqual(self.run_check().returncode, 0)

    def test_source_symlink_rejected_in_both_modes(self):
        (self.root / 'readme.txt').unlink()
        (self.root / 'readme.txt').symlink_to('PUBLIC_FILES.txt')
        for flags in [(), ('--working-tree',)]:
            self.assertNotEqual(self.run_check(*flags).returncode, 0)

    def test_dependencies_are_not_a_release_export(self):
        path = self.root / 'node_modules/example/bin'
        path.parent.mkdir(parents=True)
        path.write_bytes(b'\x00synthetic dependency')
        self.assertEqual(self.run_check('--working-tree').returncode, 0)
        self.assertNotEqual(self.run_check().returncode, 0)
        self.manifest(sorted(self.names + ['node_modules/example/bin']))
        self.assertNotEqual(self.run_check('--working-tree').returncode, 0)

    def test_wasm_is_only_allowed_in_working_tree(self):
        p = self.root / 'web/dist/i76web.wasm'
        p.parent.mkdir(parents=True)
        p.write_bytes(b'\x00asm')
        self.assertEqual(self.run_check('--working-tree').returncode, 0)
        self.assertNotEqual(self.run_check().returncode, 0)
        (p.parent / 'unexpected.dat').write_bytes(b'not approved')
        self.assertNotEqual(self.run_check('--working-tree').returncode, 0)

    @unittest.skipUnless(hasattr(os, 'mkfifo'), 'requires POSIX FIFO')
    def test_nonregular_entry_rejected_without_reading(self):
        os.mkfifo(self.root / 'pipe')
        self.assertNotEqual(self.run_check().returncode, 0)


if __name__ == '__main__':
    unittest.main()
