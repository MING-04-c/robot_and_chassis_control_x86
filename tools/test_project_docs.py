"""Check the central documentation and self-contained Kylin document export."""
import hashlib
import importlib.util
import json
from pathlib import Path
import re
import zipfile


ROOT = Path(__file__).resolve().parents[1]
DOCS = ROOT/'ros_ws/docs'


def test_current_guide_local_links_exist():
    current = [ROOT/'README.md', DOCS/'README.md', DOCS/'CODE_MAP_CN.md',
               DOCS/'archive/README.md', DOCS/'tasks/ROBOT_TASK_GUIDE.md',
               DOCS/'tasks/PAINT_DETECTION.md', DOCS/'simulation/PATH_PREVIEW.md',
               *sorted((DOCS/'tcp').glob('*.md'))]
    missing = []
    for document in current:
        assert document.is_file(), document
        for target in re.findall(r'\]\(([^)]+)\)', document.read_text()):
            if re.match(r'^[a-z]+:', target) or target.startswith('#'):
                continue
            target = target.split('#', 1)[0]
            if not (document.parent/target).exists():
                missing.append(f'{document.relative_to(ROOT)} -> {target}')
    assert not missing, '\n'.join(missing)


def test_export_uses_central_docs_not_source_redirects(tmp_path, monkeypatch):
    spec = importlib.util.spec_from_file_location('export_docs_test', ROOT/'tools/export_industrial_pc.py')
    module = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(module)
    monkeypatch.setattr(module, '__file__', str(tmp_path/'tools/export_industrial_pc.py'))
    device = tmp_path/'ros_ws/src/eli_cs_robot_serial/device/industrial_pc'
    sources = {
        device/'host_tools/config.json': '{"robot_host":"example"}',
        tmp_path/'ros_ws/src/geminipro_camer_calibration/scripts/capture_transfer.py': '# fixture',
    }
    for name in ('industrial_pc_protocol.py', 'industrial_pc_client.py',
                 'industrial_pc_hk_receiver.py', 'industrial_pc_heartbeat.py'):
        sources[device/'src'/name] = '# fixture'
    names = ('README.md', 'QUICKSTART.md', 'TCP通讯协议_麒麟工控机.md', 'OLINE流程与协议.md')
    for name in names:
        sources[tmp_path/'ros_ws/docs/tcp'/name] = (DOCS/'tcp'/name).read_text()
    for path, text in sources.items():
        path.parent.mkdir(parents=True, exist_ok=True)
        path.write_text(text)
    output = tmp_path/'industrial_pc_kylin_tcp'
    output.mkdir()
    (output/'config.json').write_text('{"robot_host":"site-specific"}')
    module.main()
    assert json.loads((output/'config.json').read_text())['robot_host'] == 'site-specific'
    for name in names:
        assert (output/name).read_bytes() == (DOCS/'tcp'/name).read_bytes()
        for target in re.findall(r'\]\(([^)]+)\)', (output/name).read_text()):
            if not re.match(r'^[a-z]+:', target) and not target.startswith('#'):
                assert (output/target.split('#', 1)[0]).exists(), (name, target)
    manifest = json.loads((output/'SOURCE_FILES_SHA256.json').read_text())
    assert all(hashlib.sha256((output/name).read_bytes()).hexdigest() == digest
               for name, digest in manifest.items())
    with zipfile.ZipFile(tmp_path/'industrial_pc_kylin_tcp.zip') as archive:
        assert archive.testzip() is None
        for name in names:
            assert archive.read('industrial_pc_kylin_tcp/'+name) == (DOCS/'tcp'/name).read_bytes()


def test_no_redirect_placeholder_documents_or_workspace_loose_guides():
    assert not (ROOT/'ros_ws/README_INTEGRATED_CN.md').exists()
    assert not (ROOT/'ros_ws/机械臂与底盘ROS控制命令.md').exists()
    placeholders = []
    for directory in (ROOT/'ros_ws/src', DOCS):
        for path in directory.rglob('*.md'):
            if path.read_text(errors='replace').startswith('# 文档已集中'):
                placeholders.append(str(path.relative_to(ROOT)))
    assert not placeholders, '\n'.join(placeholders)
