"""Build the exact SDK sources for host simulation; no hardware connection."""
from pathlib import Path
import os
import shutil
import subprocess

ROOT = Path(__file__).resolve().parents[1]


def build() -> Path:
    compiler = shutil.which("gcc") or shutil.which("clang")
    if not compiler:
        raise RuntimeError("需要 GCC/Clang；Windows 可使用 MinGW GCC，或使用预构建分发包")
    out = ROOT / "src/pid_agent/workbench/native"
    out.mkdir(parents=True, exist_ok=True)
    target = out / ("pt_sdk.dll" if os.name == "nt" else "libpt_sdk.so")
    sources = [ROOT / f"sdk/src/{n}.c" for n in ("pt_pid", "pt_protocol", "pt_runtime")]
    command = [compiler, "-std=c99", "-O2", "-Wall", "-Wextra", "-Werror", "-pedantic", "-shared"]
    command += ["-static-libgcc"] if os.name == "nt" else ["-fPIC"]
    # MinGW's linker can misdecode absolute Unicode paths; relative ASCII paths
    # keep the user's Chinese workspace name out of the compiler arguments.
    command += ["-I", "sdk/include", *(p.relative_to(ROOT).as_posix() for p in sources),
                "-lm", "-o", target.relative_to(ROOT).as_posix()]
    subprocess.run(command, check=True, cwd=ROOT)
    return target


if __name__ == "__main__":
    print(build())
