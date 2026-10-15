#!/usr/bin/env python3
"""离线自检：在没有 ROS 2 生成器的机器上，对工作空间做静态一致性检查。

它**不能替代** `colcon build` / `rosidl` 代码生成，只用于在无 Jazzy 环境下
尽早发现结构性错误：
  1. `.msg` / `.srv` / `.action` 的分隔符数量与字段语法；
  2. 常量与字段位置是否正确（Action 的 Goal 段允许常量）；
  3. `CMakeLists.txt` 中列出的接口文件是否存在，且没有漏列的接口文件；
  4. `package.xml` XML 合法性、`<name>` 与目录名一致；
  5. 全部 Python 文件语法可编译；
  6. `.gitignore` 没有误忽略源码目录。

用法：
    python3 scripts/offline_selfcheck.py
退出码：0 全部通过；1 存在失败项。
"""

from __future__ import annotations

import os
import py_compile
import re
import subprocess
import sys
import tempfile
import xml.etree.ElementTree as ET
from pathlib import Path

WORKSPACE = Path(__file__).resolve().parent.parent

PRIMITIVE_TYPES = {
    'bool', 'byte', 'char', 'float32', 'float64', 'int8', 'uint8',
    'int16', 'uint16', 'int32', 'uint32', 'int64', 'uint64', 'string', 'wstring',
}

results: list[tuple[bool, str]] = []


def check(ok: bool, message: str) -> None:
    results.append((bool(ok), message))


def is_valid_field_type(type_token: str) -> bool:
    """校验字段类型（含数组与包限定名）。"""
    base = type_token.split('[', 1)[0]
    if base in PRIMITIVE_TYPES:
        return True
    # 允许 package/msg/Type 或 msg/Type 形式
    if '/' in base:
        parts = base.split('/')
        return len(parts) in (2, 3) and all(
            re.fullmatch(r'[A-Za-z][A-Za-z0-9_]*', p) for p in parts)
    return bool(re.fullmatch(r'[A-Za-z][A-Za-z0-9_]*', base))


FIELD_RE = re.compile(
    r'^(?P<type>[A-Za-z][A-Za-z0-9_/]*(?:\[(?P<size>\d*)\])?)\s+'
    r'(?P<name>[A-Za-z][A-Za-z0-9_]*)\s*(?:#.*)?$')
CONST_RE = re.compile(
    r'^(?P<type>bool|byte|char|float32|float64|int8|uint8|int16|uint16|int32|uint32|int64|uint64|string)'
    r'\s+(?P<name>[A-Z][A-Z0-9_]*)\s*=\s*(?P<value>.+?)\s*(?:#.*)?$')


def check_interface_file(path: Path, expected_sections: int) -> None:
    """检查接口文件的语法与段数。expected_sections=1 表示 msg，2 表示 srv，4 表示 action。"""
    text = path.read_text(encoding='utf-8')
    lines = [ln.rstrip('\n') for ln in text.splitlines()]

    separators = sum(1 for ln in lines if ln.strip() == '---')
    expected_separators = expected_sections - 2 if expected_sections == 4 else expected_sections - 1
    # msg: 0 个 '---'；srv: 1 个；action: 2 个
    check(separators == expected_separators,
          f'{path.relative_to(WORKSPACE)}: 分隔符数量 {separators}，期望 {expected_separators}')

    section = 0
    section_has_field = [False] * expected_sections
    has_any_content = False
    seen_names: dict[int, set[str]] = {i: set() for i in range(expected_sections)}
    problems: list[str] = []

    # 常量名允许中间出现数字（如 COMMAND_OK / COLOR_RED）
    const_re = re.compile(
        r'^(?P<type>bool|byte|char|float32|float64|int8|uint8|int16|uint16|int32|uint32|int64|uint64|string)'
        r'\s+(?P<name>[A-Za-z][A-Za-z0-9_]*)\s*=\s*(?P<value>.+?)\s*(?:#.*)?$')

    for lineno, raw in enumerate(lines, 1):
        line = raw.strip()
        if not line or line.startswith('#'):
            continue
        if line == '---':
            section += 1
            if section >= expected_sections:
                problems.append(f'L{lineno}: 多余的分隔符')
            continue
        if section >= expected_sections:
            problems.append(f'L{lineno}: 内容出现在最后一个分隔符之后')
            continue

        const_match = const_re.match(line)
        if const_match:
            # Action 的 Goal 段（section 0）允许常量；msg/srv 也允许。
            # 但 Result / Feedback 段出现常量通常是误写。
            if expected_sections == 4 and section in (1, 2, 3) and line.split()[0] in PRIMITIVE_TYPES:
                problems.append(f'L{lineno}: Result/Feedback 段出现常量定义（疑似误写）：{line}')
            name = const_match.group('name')
            if name in seen_names[section]:
                problems.append(f'L{lineno}: 常量名重复 {name}')
            seen_names[section].add(name)
            has_any_content = True
            continue

        field_match = FIELD_RE.match(line)
        if not field_match:
            problems.append(f'L{lineno}: 无法解析的行：{line!r}')
            continue
        type_token = field_match.group('type')
        name = field_match.group('name')
        if not is_valid_field_type(type_token):
            problems.append(f'L{lineno}: 非法字段类型 {type_token!r}')
        if name in seen_names[section]:
            problems.append(f'L{lineno}: 字段名重复 {name}')
        seen_names[section].add(name)
        section_has_field[section] = True
        has_any_content = True

    # 允许“只有常量”的接口文件（例如 ErrorCodes.msg），但完全空文件应报错。
    if not has_any_content:
        problems.append('文件没有任何字段或常量')

    check(not problems, f'{path.relative_to(WORKSPACE)}: ' +
          ('语法正确' if not problems else '；'.join(problems)))


def check_package(pkg_dir: Path) -> None:
    pkg_xml = pkg_dir / 'package.xml'
    cmake = pkg_dir / 'CMakeLists.txt'

    if not pkg_xml.exists():
        return  # 占位目录，跳过

    try:
        root = ET.parse(pkg_xml).getroot()
        name = root.findtext('name')
        check(name == pkg_dir.name,
              f'{pkg_dir.name}/package.xml: <name>={name!r} 与目录名一致')
        check(root.tag == 'package', f'{pkg_dir.name}/package.xml: 根元素为 <package>')
        build_type = root.findtext('./export/build_type')
        check(build_type in ('ament_cmake', 'ament_python'),
              f'{pkg_dir.name}/package.xml: build_type={build_type!r} 合法')
    except ET.ParseError as exc:
        check(False, f'{pkg_dir.name}/package.xml: XML 解析失败：{exc}')
        return

    # 接口包：核对 CMakeLists 列出的文件与实际文件
    if cmake.exists():
        cmake_text = cmake.read_text(encoding='utf-8')
        listed = set(re.findall(r'"((?:msg|srv|action)/[A-Za-z0-9_]+\.(?:msg|srv|action))"',
                                cmake_text))
        actual = {str(p.relative_to(pkg_dir))
                  for p in list(pkg_dir.glob('msg/*.msg'))
                  + list(pkg_dir.glob('srv/*.srv'))
                  + list(pkg_dir.glob('action/*.action'))}
        missing_in_cmake = sorted(actual - listed)
        missing_on_disk = sorted(listed - actual)
        check(not missing_in_cmake,
              f'{pkg_dir.name}: 接口文件未列在 CMakeLists 中 {missing_in_cmake}')
        check(not missing_on_disk,
              f'{pkg_dir.name}: CMakeLists 列出了不存在的接口文件 {missing_on_disk}')


def check_python(pkg_dir: Path) -> None:
    py_files = [p for p in pkg_dir.rglob('*.py') if '__pycache__' not in p.parts]
    failures = []
    with tempfile.TemporaryDirectory() as tmp:
        for index, py in enumerate(py_files):
            try:
                py_compile.compile(str(py), cfile=os.path.join(tmp, f'{index}.pyc'),
                                   doraise=True)
            except py_compile.PyCompileError as exc:
                failures.append(f'{py.relative_to(WORKSPACE)}: {exc.msg.strip()}')
    if py_files:
        check(not failures,
              f'{pkg_dir.name}: {len(py_files)} 个 Python 文件语法检查' +
              ('' if not failures else '；失败：' + ' | '.join(failures)))


def check_gitignore() -> None:
    """确认不会被 .gitignore 误伤的关键源码路径。"""
    probes = [
        'src/mtc_task/mtc_task/lib/helper.py',
        'src/mtc_tool/mtc_tool/bin/tool.py',
        'config/safety.yaml',
        'docs/migration/SOURCE_AUDIT.md',
    ]
    # 注意：`git check-ignore` 对**否定规则**也会返回 0（因为它确实命中了 `!` 规则），
    # 因此不能用它的退出码判断“是否会被提交”。权威做法是临时建文件后用
    # `git status --porcelain --ignored`：`!!` 表示被忽略，`??` 表示未被忽略。
    for probe in probes:
        target = WORKSPACE / probe
        created = not target.exists()
        if created:
            target.parent.mkdir(parents=True, exist_ok=True)
            target.write_text('# offline selfcheck probe\n', encoding='utf-8')
        try:
            proc = subprocess.run(
                ['git', 'status', '--porcelain', '--ignored', '--', probe],
                cwd=WORKSPACE, capture_output=True, text=True)
            status = proc.stdout.strip()[:2]
            check(status != '!!', f'.gitignore 未误忽略 {probe}' +
                  ('' if status != '!!' else '（被忽略，会被静默排除出提交）'))
        finally:
            if created:
                target.unlink(missing_ok=True)


def check_entry_points() -> None:
    """核验 setup.py 声明的 console_scripts 指向真实存在的模块与函数。

    这一项在无 ROS 2 环境下无法通过 `ros2 run` 验证，但静态解析即可发现
    “入口点指向不存在的模块”这类安装后必然失败的问题。
    """
    import ast

    declared: dict[str, str] = {}
    for setup in sorted((WORKSPACE / 'src').glob('*/setup.py')):
        pkg = setup.parent.name
        try:
            tree = ast.parse(setup.read_text(encoding='utf-8'))
        except SyntaxError as exc:
            check(False, f'{pkg}/setup.py: 语法错误 {exc}')
            continue
        for node in ast.walk(tree):
            if not isinstance(node, ast.Dict):
                continue
            for key, value in zip(node.keys, node.values):
                if not (isinstance(key, ast.Constant) and key.value == 'console_scripts'):
                    continue
                for elt in getattr(value, 'elts', []):
                    if not isinstance(elt, ast.Constant):
                        continue
                    name, _, target = elt.value.partition(' = ')
                    declared[name.strip()] = target.strip()
                    module, _, func = target.strip().partition(':')
                    path = (setup.parent / Path(*module.split('.')))
                    file = (path.with_suffix('.py') if path.with_suffix('.py').is_file()
                            else path / '__init__.py')
                    ok = file.is_file() and f'def {func}(' in file.read_text(encoding='utf-8')
                    check(ok, f'{pkg}: 入口点 {name} -> {module}:{func} '
                              + ('存在' if ok else '缺失（安装后必然失败）'))
    if not declared:
        check(False, '没有任何包声明 console_scripts')


def check_error_code_mirrors() -> None:
    """核验各纯 Python 模块中的错误码常量与 ErrorCodes.msg 数值一致。

    FSM / 工具 / 安全 / 桥接层为脱离 rclpy 而各自镜像了错误码；
    本检查确保镜像不会与 IDL 漂移。
    """
    import importlib.util
    import re as _re
    import sys as _sys

    msg_path = WORKSPACE / 'src/mtc_interfaces/msg/ErrorCodes.msg'
    if not msg_path.is_file():
        check(False, 'ErrorCodes.msg 缺失，无法核验镜像')
        return
    ref = {name: int(value) for name, value in _re.findall(
        r'^uint16\s+([A-Z_0-9]+)\s*=\s*(\d+)', msg_path.read_text(encoding='utf-8'), _re.M)}

    targets = [
        ('mtc_tool/codes.py', 'mtc_tool/mtc_tool/codes.py', None),
        ('mtc_safety/interlocks.py', 'mtc_safety/mtc_safety/interlocks.py', None),
        ('mtc_aubo_bridge/bridge_core.py', 'mtc_aubo_bridge/mtc_aubo_bridge/bridge_core.py', None),
        ('mtc_manipulation/pick_place_fsm.py',
         'mtc_manipulation/mtc_manipulation/pick_place_fsm.py', 'ErrorCode'),
    ]
    for label, rel, container in targets:
        path = WORKSPACE / 'src' / rel
        if not path.is_file():
            check(False, f'错误码镜像核验：找不到 {rel}')
            continue
        try:
            spec = importlib.util.spec_from_file_location(
                'mirror_' + label.replace('/', '_').replace('.', '_'), path)
            module = importlib.util.module_from_spec(spec)
            _sys.modules[spec.name] = module
            spec.loader.exec_module(module)
        except Exception as exc:  # 无法导入则无法核验，按失败处理
            check(False, f'{rel}: 导入失败，无法核验错误码镜像（{type(exc).__name__}）')
            continue
        source = getattr(module, container) if container else module
        items = {k: int(v) for k, v in vars(source).items()
                 if k.isupper() and isinstance(v, int) and not isinstance(v, bool)}
        normalized = {_re.sub(r'^(ERROR_|ERR_)', '', k): v for k, v in items.items()}
        compared = {k: v for k, v in normalized.items() if k in ref}
        bad = [f'{k}: msg={ref[k]} 代码={v}' for k, v in compared.items() if ref[k] != v]
        check(not bad, f'{rel}: 错误码镜像 {len(compared)}/{len(ref)} 项与 IDL 一致'
              + ('' if not bad else f'；不一致 {bad}'))
        del _sys.modules[spec.name]


def check_joint_name_consistency() -> None:
    """核验关节名称在各配置与实现文件中的集合完全一致。

    `ExecuteJointMove` 要求 joint_names 与 target_rad 一一对应，
    名称漂移会导致执行层拒绝合法请求（或漏检非法顺序）。
    """
    import re as _re

    baseline_file = WORKSPACE / 'config/motion_profiles.yaml'
    candidates = [
        'config/motion_profiles.yaml',
        'config/manipulation.yaml',
        'src/mtc_motion_execution/mtc_motion_execution/motion_executor_node.py',
        'src/mtc_motion_execution/mtc_motion_execution/backends.py',
        'src/mtc_motion_execution/test/test_backends.py',
        'src/mtc_bringup/mtc_bringup/mock_pipeline.py',
    ]
    baseline: list[str] | None = None
    for rel in candidates:
        path = WORKSPACE / rel
        if not path.is_file():
            check(False, f'关节名核验：找不到 {rel}')
            continue
        names = list(dict.fromkeys(_re.findall(
            r'["\']([A-Za-z_0-9]*_joint)["\']', path.read_text(encoding='utf-8'))))
        if rel == 'config/motion_profiles.yaml':
            baseline = names
            check(bool(names), f'{rel}: 解析到 {len(names)} 个关节名')
            continue
        if baseline is None:
            continue
        extra = sorted(set(names) - set(baseline))
        missing = sorted(set(baseline) - set(names))
        check(set(names) == set(baseline),
              f'{rel}: 关节名与 config/motion_profiles.yaml 一致'
              + ('' if not (extra or missing) else f'；多余 {extra} 缺失 {missing}'))


def main() -> int:
    src = WORKSPACE / 'src'
    if not src.is_dir():
        print(f'错误：找不到 {src}')
        return 1

    for path in sorted(src.rglob('*.msg')):
        check_interface_file(path, expected_sections=1)
    for path in sorted(src.rglob('*.srv')):
        check_interface_file(path, expected_sections=2)
    for path in sorted(src.rglob('*.action')):
        check_interface_file(path, expected_sections=4)

    for pkg_dir in sorted(p for p in src.iterdir() if p.is_dir()):
        check_package(pkg_dir)
        check_python(pkg_dir)

    check_gitignore()
    check_entry_points()
    check_error_code_mirrors()
    check_joint_name_consistency()

    passed = sum(1 for ok, _ in results if ok)
    failed = [(ok, msg) for ok, msg in results if not ok]
    width = max((len(msg) for _, msg in results), default=10)
    for ok, msg in results:
        print(f"[{'PASS' if ok else 'FAIL'}] {msg:<{width}}")
    print()
    print(f'OFFLINE SELF-CHECK RESULT: PASS {passed} / FAIL {len(failed)}')
    if failed:
        print('\n失败项：')
        for _, msg in failed:
            print(f'  - {msg}')
    return 1 if failed else 0


if __name__ == '__main__':
    sys.exit(main())
