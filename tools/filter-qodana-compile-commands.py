#!/usr/bin/env python3
from __future__ import annotations

import argparse
import json
import os
from pathlib import Path


INCLUDE_ROOTS = ("Algorithms", "Components", "Devices", "Interface", "Services")


def included_relative_path(path_text: str) -> tuple[Path, str] | None:
    normalized = path_text.replace("\\", "/")

    for root in INCLUDE_ROOTS:
        prefix = f"{root}/"
        if normalized.startswith(prefix):
            return Path(normalized), ""

        marker = f"/{root}/"
        marker_index = normalized.find(marker)
        if marker_index != -1:
            return Path(normalized[marker_index + 1 :]), normalized[:marker_index]

    return None


def replace_project_root(text: str, original_roots: set[str], project_dir: Path) -> str:
    rewritten = text
    project_dir_text = project_dir.as_posix()

    for original_root in sorted(original_roots, key=len, reverse=True):
        if original_root:
            rewritten = rewritten.replace(original_root, project_dir_text)

    return rewritten


def filter_compile_commands(compile_commands_path: Path, project_dir: Path) -> tuple[int, int]:
    compile_commands_path = compile_commands_path.resolve()
    project_dir = project_dir.resolve()
    backup_path = compile_commands_path.with_name("compile_commands.full.json")

    with compile_commands_path.open(encoding="utf-8") as file:
        compile_commands = json.load(file)

    original_roots: set[str] = set()
    filtered = []

    for entry in compile_commands:
        file_path = entry.get("file")
        if not isinstance(file_path, str):
            continue

        relative_with_root = included_relative_path(file_path)
        if relative_with_root is None:
            continue

        relative_path, original_root = relative_with_root
        original_roots.add(original_root)

        filtered_entry = dict(entry)
        filtered_entry["directory"] = compile_commands_path.parent.as_posix()
        filtered_entry["file"] = (project_dir / relative_path).as_posix()

        if "command" in filtered_entry and isinstance(filtered_entry["command"], str):
            filtered_entry["command"] = replace_project_root(
                filtered_entry["command"],
                original_roots,
                project_dir,
            )

        if "arguments" in filtered_entry and isinstance(filtered_entry["arguments"], list):
            filtered_entry["arguments"] = [
                replace_project_root(argument, original_roots, project_dir)
                if isinstance(argument, str)
                else argument
                for argument in filtered_entry["arguments"]
            ]

        if "output" in filtered_entry and isinstance(filtered_entry["output"], str):
            filtered_entry["output"] = replace_project_root(
                filtered_entry["output"],
                original_roots,
                project_dir,
            )

        filtered.append(filtered_entry)

    if not filtered:
        raise SystemExit(f"{compile_commands_path} would be empty")

    if backup_path.exists():
        backup_path.unlink()

    compile_commands_path.rename(backup_path)
    with compile_commands_path.open("w", encoding="utf-8") as file:
        json.dump(filtered, file, indent=2)
        file.write("\n")

    return len(filtered), len(compile_commands)


def main() -> None:
    parser = argparse.ArgumentParser(description="Filter compile_commands.json for Qodana C/C++ analysis.")
    parser.add_argument(
        "compile_commands",
        nargs="?",
        type=Path,
        help="Path to the compile_commands.json file to filter in place. Defaults to build/$QODANA_CMAKE_PRESET/compile_commands.json.",
    )
    parser.add_argument(
        "--project-dir",
        type=Path,
        default=Path(os.environ.get("QODANA_PROJECT_DIR", Path.cwd())),
        help="Project directory as seen by Qodana. Defaults to QODANA_PROJECT_DIR or the current working directory.",
    )
    args = parser.parse_args()

    compile_commands = args.compile_commands
    if compile_commands is None:
        cmake_preset = os.environ.get("QODANA_CMAKE_PRESET")
        if not cmake_preset:
            raise SystemExit("QODANA_CMAKE_PRESET is not set")
        compile_commands = Path("build") / cmake_preset / "compile_commands.json"

    filtered_count, total_count = filter_compile_commands(compile_commands, args.project_dir)
    print(f"Filtered compile commands: {filtered_count} / {total_count}")


if __name__ == "__main__":
    main()
