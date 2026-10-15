"""Public hardware transport must use explicit site settings, never a baked-in NUC."""
import io
from types import SimpleNamespace
from unittest.mock import patch

import pytest
from mtc_webui.hardware_backend import HardwareTransport


def transport(host=None, alias=None):
    settings = dict(remote_python='/sdk/python', remote_bridge='/deploy/bridge.py',
                    remote_package='/deploy/package', manifest_sha256='fixture')
    if host is not None:
        settings['ssh_host'] = host
    if alias is not None:
        settings['ssh_host_key_alias'] = alias
    return HardwareTransport(SimpleNamespace(settings=settings),
                             dict(username='fixture', password='not-real'))


@pytest.mark.parametrize('host', [None, '', '-unsafe', 'operator@host extra'])
def test_missing_or_invalid_site_refused_before_subprocess(host):
    with patch('mtc_webui.hardware_backend.subprocess.Popen') as popen:
        with pytest.raises(ValueError, match='ssh_host'):
            transport(host)._spawn('state')
        popen.assert_not_called()


@pytest.mark.parametrize('alias', [None, 'commissioned-host'])
def test_transport_uses_site_host_and_optional_key_alias(alias):
    child = SimpleNamespace(stdin=io.StringIO(), poll=lambda: 0)
    with patch('mtc_webui.hardware_backend.subprocess.Popen', return_value=child) as popen:
        assert transport('operator@nuc.example.invalid', alias)._spawn('state') is child
        argv = popen.call_args.args[0]
        assert argv[-2] == 'operator@nuc.example.invalid'
        assert 'StrictHostKeyChecking=yes' in argv
        assert [x for x in argv if x.startswith('HostKeyAlias=')] == ([] if alias is None else ['HostKeyAlias='+alias])
        assert 'not-real' not in ' '.join(argv)
