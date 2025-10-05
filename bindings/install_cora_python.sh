#!/usr/bin/env bash
# Simple helper to make the built CORA Python module importable in your current Python environment.
# Default behavior: writes a .pth file pointing to the build/lib directory into site-packages.
# Usage:
#   ./bindings/install_cora_python.sh [path-to-build-lib] [python-exe]
# Examples:
#   ./bindings/install_cora_python.sh                 # uses ../build/lib and python3
#   ./bindings/install_cora_python.sh /tmp/build/lib  # custom build/lib
#   ./bindings/install_cora_python.sh '' python       # use default build/lib but with `python`

set -euo pipefail

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
DEFAULT_BUILD_LIB="$(cd "${SCRIPT_DIR}/.." && pwd)/build/lib"

BUILD_LIB_DIR="${1:-$DEFAULT_BUILD_LIB}"
PYEXE="${2:-python3}"

if [[ ! -d "${BUILD_LIB_DIR}" ]]; then
  echo "Error: build/lib directory not found at: ${BUILD_LIB_DIR}" >&2
  echo "Please build with CMake first (cmake .. && make) or pass the correct path." >&2
  exit 1
fi

# Find the site-packages directory for the chosen Python
SITE_PACKAGES_DIR="$(${PYEXE} -c 'import sysconfig; print(sysconfig.get_paths()["purelib"])')"

if [[ -z "${SITE_PACKAGES_DIR}" || ! -d "${SITE_PACKAGES_DIR}" ]]; then
  echo "Error: could not determine site-packages for ${PYEXE}." >&2
  exit 1
fi

PTH_FILE="${SITE_PACKAGES_DIR}/cora_local.pth"

echo "Installing CORA Python path via .pth file:"
echo "  BUILD_LIB_DIR:     ${BUILD_LIB_DIR}"
echo "  PYTHON EXECUTABLE: ${PYEXE}"
echo "  SITE-PACKAGES:     ${SITE_PACKAGES_DIR}"
echo "  PTH FILE:          ${PTH_FILE}"

echo "${BUILD_LIB_DIR}" > "${PTH_FILE}"
echo "Wrote ${PTH_FILE} with path: ${BUILD_LIB_DIR}"

# Copy type stub for editor / type-checker convenience
STUB_SRC="${SCRIPT_DIR}/cora.pyi"
if [[ -f "${STUB_SRC}" ]]; then
  echo "Copying stub file to site-packages: ${STUB_SRC} -> ${SITE_PACKAGES_DIR}"
  cp "${STUB_SRC}" "${SITE_PACKAGES_DIR}/cora.pyi"
fi

echo "Verifying import in a fresh ${PYEXE}..."
${PYEXE} - <<'PYCODE'
import importlib, sys
try:
    m = importlib.import_module('cora')
    print('Import cora OK. Module file:', getattr(m, '__file__', '<built-in>'))
except Exception as e:
    print('FAILED to import cora:', e)
    sys.exit(1)
PYCODE

echo "Done. You can now 'import cora' from ${PYEXE}."
