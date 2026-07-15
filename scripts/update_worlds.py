#!/usr/bin/env python3
"""Align Webots world files with Norby sensor configuration."""

from pathlib import Path
import re

ROOT = Path(__file__).resolve().parents[1]

NEW_EXTENSION_SLOT = """  extensionSlot [
    Accelerometer {
      name "accelerometer(1)"
    }
    Gyro {
      name "gyro(1)"
    }
    InertialUnit {
    }
    GPS {
    }
    Camera {
      name "camera"
      translation 0 0.01 0.04
      width 640
      height 480
    }
    RobotisLds01 {
    }
  ]"""

EXTENSION_SLOT_PATTERN = re.compile(
    r'  extensionSlot \[\s*'
    r'(?:Camera \{.*?\}\s*)?'
    r'RobotisLds01 \{\s*\}\s*'
    r'\]',
    re.DOTALL,
)


def _align_extension_slot(text: str) -> str:
    if 'GPS {' in text and 'name "camera"' in text:
        return text
    return EXTENSION_SLOT_PATTERN.sub(NEW_EXTENSION_SLOT, text, count=1)


def _pin_externproto(text: str) -> str:
    return (
        text.replace('R2023b', 'R2025a')
        .replace('/develop/projects/', '/R2025a/projects/')
    )


def main() -> None:
    candidates = list(ROOT.rglob('*.wbt'))
    for path in candidates:
        original = path.read_text(encoding='utf-8')
        updated = _pin_externproto(_align_extension_slot(original))
        if updated != original:
            path.write_text(updated, encoding='utf-8')
            print(f'updated {path.relative_to(ROOT)}')


if __name__ == '__main__':
    main()
