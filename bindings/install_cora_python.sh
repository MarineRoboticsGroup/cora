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

# Determine python executable to use:
# 1) If caller provided a non-empty 2nd arg, use that.
# 2) Else if environment variable PYEXE is set and non-empty, use that.
# 3) Otherwise default to 'python3'.
if [[ -n "${2:-}" ]]; then
  PYEXE="${2}"
elif [[ -n "${PYEXE:-}" ]]; then
  # respect exported PYEXE in the shell if present
  PYEXE="${PYEXE}"
else
  PYEXE="python3"
fi

# Verify the python executable exists on PATH
if ! command -v "${PYEXE}" >/dev/null 2>&1; then
  echo "Error: python executable '${PYEXE}' not found in PATH. Try installing Python or pass a valid python executable." >&2
  exit 1
fi

if [[ ! -d "${BUILD_LIB_DIR}" ]]; then
  echo "Error: build/lib directory not found at: ${BUILD_LIB_DIR}" >&2
  echo "Please build with CMake first (cmake .. && make) or pass the correct path." >&2
  exit 1
fi

# Find the site-packages directory for the chosen Python. Try several
# methods to be robust across system Python, venvs, conda, and older
# distributions.
SITE_PACKAGES_DIR=""

try_cmd() {
  # Run a python snippet and return trimmed stdout (silence stderr)
  "${PYEXE}" -c "$1" 2>/dev/null || true
}

# Candidate methods (prefer sysconfig.get_paths()['purelib'])
c1="$(try_cmd 'import sysconfig; print(sysconfig.get_paths().get("purelib",""))')"
c2="$(try_cmd 'import sysconfig; print(sysconfig.get_path("purelib") or "")')"
c3="$(try_cmd 'from distutils.sysconfig import get_python_lib; print(get_python_lib())')"
c4="$(try_cmd 'import site; print(site.getsitepackages()[0] if hasattr(site, "getsitepackages") else "")')"
c5="$(try_cmd 'import site; print(site.getusersitepackages() if hasattr(site, "getusersitepackages") else "")')"

for cand in "$c1" "$c2" "$c3" "$c4" "$c5"; do
  # strip CRs and surrounding whitespace
  cand="$(echo "$cand" | tr -d '\r' | sed -e 's/^\s\+//;s/\s\+$//')"
  if [[ -n "$cand" && -d "$cand" ]]; then
    SITE_PACKAGES_DIR="$cand"
    break
  fi
done

# Last resort: try a conventional prefix-based guess (e.g. /usr or venv prefix)
if [[ -z "$SITE_PACKAGES_DIR" ]]; then
  py_prefix="$(try_cmd 'import sys; print(sys.prefix)')"
  py_ver="$(try_cmd 'import sys; print("%s.%s" % (sys.version_info[0], sys.version_info[1]))')"
  py_prefix="$(echo "$py_prefix" | tr -d '\r')"
  py_ver="$(echo "$py_ver" | tr -d '\r')"
  if [[ -n "$py_prefix" && -n "$py_ver" ]]; then
    guess="$py_prefix/lib/python$py_ver/site-packages"
    if [[ -d "$guess" ]]; then
      SITE_PACKAGES_DIR="$guess"
    fi
  fi
fi

if [[ -z "${SITE_PACKAGES_DIR}" || ! -d "${SITE_PACKAGES_DIR}" ]]; then
  echo "Error: could not determine site-packages for ${PYEXE}. Tried multiple detection methods." >&2
  echo "You can pass an explicit python executable as the 2nd arg to this script, or set PYEXE in the environment." >&2
  echo "Debug outputs (may help):" >&2
  echo "  sysconfig.get_paths()['purelib']: $c1" >&2
  echo "  sysconfig.get_path('purelib'): $c2" >&2
  echo "  distutils.get_python_lib(): $c3" >&2
  echo "  site.getsitepackages()[0]: $c4" >&2
  echo "  site.getusersitepackages(): $c5" >&2
  echo "  sys.prefix guess: $py_prefix" >&2
  echo "  python major.minor: $py_ver" >&2
  exit 1
fi

PTH_FILE="${SITE_PACKAGES_DIR}/cora_local.pth"

echo "Installing CORA Python path via .pth file:"
echo "  BUILD_LIB_DIR:     ${BUILD_LIB_DIR}"
echo "  PYTHON EXECUTABLE: ${PYEXE}"
echo "  SITE-PACKAGES:     ${SITE_PACKAGES_DIR}"
echo "  PTH FILE:          ${PTH_FILE}"

# Try to write the .pth file. If we don't have permission to write to the
# system site-packages, prefer the per-user site-packages; as a last resort
# offer to use sudo to write into the system location.
write_pth() {
  local target_pth="$1"
  echo "${BUILD_LIB_DIR}" > "${target_pth}" 2>/dev/null && return 0 || return 1
}

if write_pth "${PTH_FILE}"; then
  echo "Wrote ${PTH_FILE} with path: ${BUILD_LIB_DIR}"
else
  # try user site-packages
  USER_SITE="$(${PYEXE} -c 'import site,sys; print(site.getusersitepackages())' 2>/dev/null || true)"
  USER_SITE="$(echo "${USER_SITE}" | tr -d '\r')"
  if [[ -n "${USER_SITE}" && -d "${USER_SITE}" ]]; then
    USER_PTH="${USER_SITE}/cora_local.pth"
    if write_pth "${USER_PTH}"; then
      PTH_FILE="${USER_PTH}"
      SITE_PACKAGES_DIR="${USER_SITE}"
      echo "No permission to write system site-packages; wrote ${PTH_FILE} in user site-packages instead."
    else
      echo "No permission to write to user site-packages (${USER_SITE}) either." >&2
    fi
  fi

  # if still not written, try sudo if available
  if [[ ! -f "${PTH_FILE}" ]]; then
    if command -v sudo >/dev/null 2>&1; then
      echo "Attempting to write ${PTH_FILE} using sudo (you may be prompted for your password)..."
      echo "${BUILD_LIB_DIR}" | sudo tee "${PTH_FILE}" >/dev/null
      if [[ -f "${PTH_FILE}" ]]; then
        echo "Wrote ${PTH_FILE} with sudo."
      else
        echo "Failed to write ${PTH_FILE} even with sudo." >&2
        exit 1
      fi
    else
      echo "Permission denied writing ${PTH_FILE}, and sudo is not available." >&2
      echo "You can either run this script with sudo or set up a user .pth manually in your user site-packages." >&2
      exit 1
    fi
  fi
fi

# Copy type stub for editor / type-checker convenience into the actual site-packages used
STUB_SRC="${SCRIPT_DIR}/cora.pyi"
if [[ -f "${STUB_SRC}" ]]; then
  echo "Copying stub file to site-packages: ${STUB_SRC} -> ${SITE_PACKAGES_DIR}"
  if cp "${STUB_SRC}" "${SITE_PACKAGES_DIR}/cora.pyi" 2>/dev/null; then
    : # success
  else
    # try sudo copy as fallback
    if command -v sudo >/dev/null 2>&1; then
      echo "Copying stub requires elevation; trying sudo..."
      sudo cp "${STUB_SRC}" "${SITE_PACKAGES_DIR}/cora.pyi"
    else
      echo "Warning: could not copy stub into ${SITE_PACKAGES_DIR} and sudo is not available." >&2
    fi
  fi
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
