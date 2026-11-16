# Copyright 2026 Tony Le, Gabriel Novas
#
# Use of this source code is governed by an MIT-style
# license that can be found in the LICENSE file or at
# https://opensource.org/licenses/MIT.

"""
Rules every source file of these repositories follows; each package checks its own tree.

- No source file is longer than MAX_LINES lines.
- Source files contain ASCII characters only (English text, no emojis).
"""

import os

MAX_LINES = 200
SOURCE_EXTENSIONS = (
    '.py', '.cpp', '.hpp', '.h', '.xacro', '.yaml', '.yml', '.xml', '.cmake', '.md', '.txt',
    '.json', '.srdf', '.repos', '.tcss')
# Not written by hand, or not text: meshes, CAD, RViz layouts, the vendored Modbus library and
# build output.
SKIPPED_DIRECTORIES = (
    '.git', 'meshes', 'meshes_original', 'cad', 'rviz', 'Modbus', 'build', 'install', 'log',
    '__pycache__', 'doc')


def source_files(root):
    """Yield the hand-written text files below root."""
    for directory, subdirectories, names in os.walk(root):
        subdirectories[:] = [name for name in subdirectories if name not in SKIPPED_DIRECTORIES]
        for name in sorted(names):
            if name.endswith(SOURCE_EXTENSIONS):
                yield os.path.join(directory, name)


def violations(root):
    """Return one message per broken rule in the tree below root (empty when all is well)."""
    found = []
    for path in source_files(root):
        with open(path, 'rb') as stream:
            lines = stream.read().split(b'\n')
        relative = os.path.relpath(path, root)
        if lines and lines[-1] == b'':
            lines = lines[:-1]
        if len(lines) > MAX_LINES:
            found.append(f'{relative}: {len(lines)} lines (limit {MAX_LINES})')
        for number, line in enumerate(lines, start=1):
            if any(byte > 127 for byte in line):
                found.append(f'{relative}:{number}: non-ASCII character')
                break
    return found
