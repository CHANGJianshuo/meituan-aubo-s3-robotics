#!/usr/bin/env python3
"""核对 Ownership Audit 基线478项身份与全部 C++（含 Planner）GTest 实际执行结果。"""
import argparse
from collections import Counter
from pathlib import Path
import re
import subprocess
import xml.etree.ElementTree as ET


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--build-base', type=Path, default=Path('build'))
    args = parser.parse_args()
    root = Path(__file__).resolve().parents[3]
    build = args.build_base if args.build_base.is_absolute() else root / args.build_base
    git = lambda *a: subprocess.check_output(['git', *a], cwd=root, text=True)
    baseline = set()
    pattern = re.compile(r'\bTEST(?:_F)?\s*\(\s*(\w+)\s*,\s*(\w+)\s*\)')
    base = '681bb64bfb8cd482d1e78950cef4cfe395b21bff'
    for name in git('ls-tree', '-r', '--name-only', base, 'src').splitlines():
        if re.fullmatch(r'src/(?:mtc_\w+_cpp|mtc_moveit_planner)/test/test_\w+\.cpp', name):
            source = git('show', f'{base}:{name}')
            baseline.update(f'{s}.{c}' for s, c in pattern.findall(source)
                            if (s, c) != ('TrajectoryValidator', 'Name'))
            baseline.update('TrajectoryValidator.' + c for c in
                            re.findall(r'^INVALID_TEST\((\w+),', source, re.M))
    assert len(baseline) == 478, len(baseline)
    actual = Counter()
    inventory = set()
    by_package = Counter()
    packages = list((root / 'src').glob('mtc_*_cpp')) + [root / 'src/mtc_moveit_planner']
    xmls = [p for pkg in packages for p in (build / pkg.name / 'test_results').rglob('*.gtest.xml')]
    expected_executables = 0
    declared = set()
    for pkg in packages:
        cmake = pkg / 'CMakeLists.txt'
        if not cmake.exists():
            continue
        for source_file in (pkg / 'test').glob('test_*.cpp'):
            source = source_file.read_text()
            declared.update(f'{s}.{c}' for s, c in pattern.findall(source)
                            if (s, c) != ('TrajectoryValidator', 'Name'))
            declared.update('TrajectoryValidator.' + c for c in
                            re.findall(r'^INVALID_TEST\((\w+),', source, re.M))
        names = re.findall(r'ament_add_gtest\(\s*(test_\w+)', cmake.read_text())
        expected_executables += len(names)
        for name in names:
            # Listing registers tests but never runs fixtures or enables a backend.
            output = subprocess.check_output([str(build / pkg.name / name), '--gtest_list_tests'],
                                             text=True, timeout=15)
            suite = None
            for line in output.splitlines():
                if re.fullmatch(r'\w+\.', line):
                    suite = line[:-1]
                elif suite and re.fullmatch(r'  \w+', line):
                    key = f'{suite}.{line.strip()}'
                    assert key not in inventory, f'duplicate registration {key}'
                    inventory.add(key)
    for filename in xmls:
        for case in ET.parse(filename).getroot().iter('testcase'):
            key = f"{case.attrib['classname'].split('.')[-1]}.{case.attrib['name']}"
            assert case.find('failure') is None and case.find('error') is None, key
            assert case.find('skipped') is None and case.get('status') == 'run', key
            assert case.get('result') == 'completed', key
            actual[key] += 1
            by_package[filename.relative_to(build).parts[0]] += 1
    assert inventory == declared, (sorted(declared - inventory), sorted(inventory - declared))
    assert len(xmls) == expected_executables, (len(xmls), expected_executables)
    assert baseline <= actual.keys(), sorted(baseline - actual.keys())
    assert inventory == actual.keys(), (sorted(inventory - actual.keys()), sorted(actual.keys() - inventory))
    assert all(v == 1 for v in actual.values()), actual
    print(f'PASS: 原478项身份全部保留并执行；本轮新增{len(inventory - baseline)}项')
    print(f'PASS: {len(actual)}项独立GTest / {len(xmls)}个可执行文件；FAIL 0 / SKIP 0')
    for package, count in sorted(by_package.items()):
        print(f'  {package}: {count} PASS')


if __name__ == '__main__':
    main()
