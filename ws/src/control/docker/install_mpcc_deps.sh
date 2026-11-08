#!/usr/bin/env bash
# Run as root in the ROS Humble image. Keep MPCC numerical deps separate from TUM/ROS.
set -euo pipefail

acados_dir=/opt/acados
mpcc_venv=/opt/mpcc_venv
acados_revision=48e223e85f0408ebfd1d8c6d6fb0589e9c41b3aa # v0.5.1

apt-get update
apt-get install -y --no-install-recommends build-essential cmake git python3-venv
if [ ! -d "$acados_dir" ]; then
    git clone --branch v0.5.1 --depth 1 https://github.com/acados/acados.git "$acados_dir"
fi
test "$(git -C "$acados_dir" rev-parse HEAD)" = "$acados_revision"
git -C "$acados_dir" submodule update --init --depth 1 external/blasfeo external/hpipm
cmake -S "$acados_dir" -B "$acados_dir/build" \
    -DCMAKE_BUILD_TYPE=Release -DBLASFEO_TARGET=GENERIC -DHPIPM_TARGET=GENERIC
cmake --build "$acados_dir/build" --target install --parallel 4

# System site packages expose ROS's apt dependencies; pip writes only to this venv.
python3 -m venv --system-site-packages "$mpcc_venv"
"$mpcc_venv/bin/pip" install 'numpy==1.24.4' 'scipy==1.10.1' 'casadi==3.7.2' 'cython==3.3.1' 'future-fstrings==1.2.0'
"$mpcc_venv/bin/pip" install -e "$acados_dir/interfaces/acados_template"
ACADOS_SOURCE_DIR="$acados_dir" "$mpcc_venv/bin/python3" - <<'PY'
from pathlib import Path
from urllib.request import urlopen
import shutil
from acados_template.utils import TERA_VERSION, get_architecture_amd64_arm64, get_tera_exec_path
# Download acados's matching renderer now instead of prompting during ROS startup.
renderer = Path(get_tera_exec_path())
if not renderer.exists():
    architecture = get_architecture_amd64_arm64()
    url = (f'https://github.com/acados/tera_renderer/releases/download/v{TERA_VERSION}/'
           f't_renderer-v{TERA_VERSION}-linux-{architecture}')
    renderer.parent.mkdir(parents=True, exist_ok=True)
    with urlopen(url) as response, renderer.open('wb') as output:
        shutil.copyfileobj(response, output)
    renderer.chmod(0o755)
PY
