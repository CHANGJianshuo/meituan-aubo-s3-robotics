#!/usr/bin/env python3
"""检查原有 318 项与本包所有 GTest 的真实执行身份；不修改历史核验脚本。"""
import argparse
from collections import Counter
from pathlib import Path
import re
import subprocess
import xml.etree.ElementTree as ET


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--build-base", type=Path, default=Path("build"))
    args = parser.parse_args()
    root = Path(__file__).resolve().parents[3]
    pattern = re.compile(r"\bTEST(?:_F)?\s*\(\s*(\w+)\s*,\s*(\w+)\s*\)")
    base = "1d940f7c5f12561fed6641457c0b4b630a282b05"
    git = lambda *a: subprocess.check_output(["git", *a], cwd=root, text=True)
    baseline = set()
    for f in git("ls-tree", "-r", "--name-only", base, "src").splitlines():
        if re.fullmatch(r"src/mtc_\w+_cpp/test/test_\w+\.cpp", f):
            baseline.update(f"{s}.{c}" for s, c in pattern.findall(git("show", f"{base}:{f}")))
    assert len(baseline) == 318, len(baseline)
    current = set()
    for f in (root / "src").glob("mtc_*_cpp/test/test_*.cpp"):
        current.update(f"{s}.{c}" for s, c in pattern.findall(f.read_text()))
    build = args.build_base if args.build_base.is_absolute() else root / args.build_base
    actual = Counter()
    packages = Counter()
    xmls = list(build.glob("mtc_*_cpp/test_results/**/*.gtest.xml"))
    for f in xmls:
        for case in ET.parse(f).getroot().iter("testcase"):
            key = f"{case.attrib['classname'].split('.')[-1]}.{case.attrib['name']}"
            assert case.find("failure") is None and case.find("error") is None, key
            assert case.find("skipped") is None and case.get("status") == "run", key
            assert case.get("result") == "completed", key
            actual[key] += 1
            packages[f.relative_to(build).parts[0]] += 1
    assert baseline <= actual.keys(), sorted(baseline - actual.keys())
    assert current == actual.keys(), (sorted(current - actual.keys()), sorted(actual.keys() - current))
    assert all(v == 1 for v in actual.values()), actual
    assert len(xmls) == 20, len(xmls)  # 原16个 + 本包4个注册可执行文件。
    print(f"PASS: 原有318项身份全部保留并执行；本轮新增{len(current-baseline)}项")
    print(f"PASS: {len(actual)}项独立GTest / {len(xmls)}个可执行文件，FAIL 0 / SKIP 0")
    for pkg, count in sorted(packages.items()):
        print(f"  {pkg}: {count} PASS")


if __name__ == "__main__":
    main()
