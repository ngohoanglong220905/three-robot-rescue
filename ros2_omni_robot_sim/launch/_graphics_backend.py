"""Choose a process environment for RViz and Gazebo on hybrid GPU machines."""

from pathlib import Path
import subprocess


def _intel_dri_prime():
    for render_node in sorted(Path('/sys/class/drm').glob('renderD*')):
        device = render_node / 'device'
        try:
            if (device / 'vendor').read_text().strip().lower() == '0x8086':
                address = device.resolve().name.replace(':', '_').replace('.', '_')
                return f'pci-{address}'
        except OSError:
            continue
    return None


def graphics_environment(requested):
    """Return the backend, child environment overrides, and diagnostic text."""
    rendering = requested
    intel_device = _intel_dri_prime()
    diagnostic = None
    if rendering == 'auto':
        rendering = 'native'
        try:
            driver = subprocess.run(
                ['nvidia-smi'], stdout=subprocess.PIPE, stderr=subprocess.STDOUT,
                text=True, timeout=2.0, check=False,
            )
        except (OSError, subprocess.TimeoutExpired):
            pass
        else:
            if 'driver/library version mismatch' in driver.stdout.lower():
                rendering = 'intel' if intel_device else 'software'
                diagnostic = (
                    'NVIDIA driver/library version mismatch. Restart the computer '
                    'to synchronize the NVIDIA kernel module and libraries.'
                )

    if rendering == 'native':
        return rendering, {}, diagnostic
    if rendering not in ('intel', 'software'):
        raise ValueError(f'Unsupported graphics backend: {requested}')
    if rendering == 'intel' and not intel_device:
        raise RuntimeError('No Intel render device found; select software rendering instead.')

    environment = {
        'QT_QPA_PLATFORM': 'xcb',
        'QT_XCB_GL_INTEGRATION': 'xcb_glx',
        '__GLX_VENDOR_LIBRARY_NAME': 'mesa',
        'LIBGL_ALWAYS_SOFTWARE': 'true' if rendering == 'software' else 'false',
    }
    if rendering == 'intel':
        environment['DRI_PRIME'] = intel_device
    # Gazebo's offscreen sensor renderer can also use EGL. Keep it on Mesa
    # instead of loading the incompatible NVIDIA vendor library.
    mesa_egl = Path('/usr/share/glvnd/egl_vendor.d/50_mesa.json')
    if mesa_egl.is_file():
        environment['__EGL_VENDOR_LIBRARY_FILENAMES'] = str(mesa_egl)
    return rendering, environment, diagnostic
