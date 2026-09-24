#!/usr/bin/env bash

set -euo pipefail

readonly SCRIPT_DIR="$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")" && pwd)"
readonly PROJECT_DIR="$(cd -- "${SCRIPT_DIR}/.." && pwd)"
readonly THIRD_PARTY_DIR="${PROJECT_DIR}/third_party"
readonly PINOCCHIO_DIR="${THIRD_PARTY_DIR}/pinocchio"
readonly PINOCCHIO_PREFIX="${PINOCCHIO_DIR}/install"
readonly MICROMAMBA_DIR="${THIRD_PARTY_DIR}/.tools/micromamba"
readonly MICROMAMBA="${MICROMAMBA_DIR}/micromamba"
readonly PRIVATE_MAMBA_ROOT="${THIRD_PARTY_DIR}/.micromamba"
readonly PINOCCHIO_VERSION="4.1.0"

usage() {
    cat <<EOF
Usage: $0 <command>

Commands:
  install   Install Pinocchio ${PINOCCHIO_VERSION} into third_party/pinocchio/install.
  status    Show whether the expected private Pinocchio is installed.
  version   Print the expected and installed Pinocchio versions.
  remove    Remove the private Pinocchio installation.
  purge     Remove Pinocchio, the private micromamba binary and its package cache.
EOF
}

platform_name() {
    case "$(uname -m)" in
        x86_64)
            printf '%s\n' "linux-64"
            ;;
        aarch64|arm64)
            printf '%s\n' "linux-aarch64"
            ;;
        *)
            echo "Unsupported architecture: $(uname -m)" >&2
            return 1
            ;;
    esac
}

installed_version() {
    local metadata
    metadata="$(find "${PINOCCHIO_PREFIX}/conda-meta"         -maxdepth 1 -type f -name 'pinocchio-*.json' -print -quit 2>/dev/null || true)"

    if [[ -z "${metadata}" ]]; then
        return 1
    fi

    sed -n 's/.*"version":[[:space:]]*"\([^"]*\)".*/\1/p' "${metadata}" | head -n 1
}

verify_installation() {
    local version
    version="$(installed_version || true)"

    if [[ "${version}" != "${PINOCCHIO_VERSION}" ]]; then
        echo "Expected Pinocchio ${PINOCCHIO_VERSION}, found '${version:-none}'." >&2
        return 1
    fi

    if ! find "${PINOCCHIO_PREFIX}" -type f         -path '*/cmake/pinocchio/pinocchioConfig.cmake' -print -quit |
        grep -q .; then
        echo "Pinocchio CMake package was not found under ${PINOCCHIO_PREFIX}." >&2
        return 1
    fi

    if ! compgen -G "${PINOCCHIO_PREFIX}/lib/libpinocchio_parsers.so*" >/dev/null; then
        echo "Pinocchio URDF parser library was not found." >&2
        return 1
    fi
}

bootstrap_micromamba() {
    if [[ -x "${MICROMAMBA}" ]]; then
        return
    fi

    command -v curl >/dev/null 2>&1 || {
        echo "curl is required to bootstrap the private micromamba executable." >&2
        exit 1
    }
    command -v tar >/dev/null 2>&1 || {
        echo "tar with bzip2 support is required to bootstrap micromamba." >&2
        exit 1
    }

    local platform
    local temporary_dir
    platform="$(platform_name)"
    temporary_dir="$(mktemp -d)"

    echo "Downloading project-private micromamba for ${platform}..."
    if ! curl -fsSL "https://micro.mamba.pm/api/micromamba/${platform}/latest" |
        tar -xj -C "${temporary_dir}" bin/micromamba; then
        rm -rf "${temporary_dir}"
        echo "Failed to download or extract micromamba." >&2
        exit 1
    fi

    mkdir -p "${MICROMAMBA_DIR}"
    install -m 0755 "${temporary_dir}/bin/micromamba" "${MICROMAMBA}"
    rm -rf "${temporary_dir}"
}

install_pinocchio() {
    if verify_installation >/dev/null 2>&1; then
        echo "Pinocchio ${PINOCCHIO_VERSION} is already installed."
        echo "Prefix: ${PINOCCHIO_PREFIX}"
        return
    fi

    bootstrap_micromamba

    rm -rf "${PINOCCHIO_PREFIX}"
    mkdir -p "${PINOCCHIO_DIR}" "${PRIVATE_MAMBA_ROOT}"

    echo "Installing Pinocchio ${PINOCCHIO_VERSION} into:"
    echo "  ${PINOCCHIO_PREFIX}"

    MAMBA_ROOT_PREFIX="${PRIVATE_MAMBA_ROOT}" "${MICROMAMBA}" create         --yes         --prefix "${PINOCCHIO_PREFIX}"         --override-channels         --channel conda-forge         "pinocchio=${PINOCCHIO_VERSION}"

    verify_installation
    echo "Private Pinocchio installation is ready."
}

show_status() {
    if verify_installation >/dev/null 2>&1; then
        echo "Pinocchio status: ready"
        echo "Version: $(installed_version)"
        echo "Prefix:  ${PINOCCHIO_PREFIX}"
        return 0
    fi

    echo "Pinocchio status: not installed"
    echo "Expected: ${PINOCCHIO_VERSION}"
    echo "Prefix:   ${PINOCCHIO_PREFIX}"
    return 1
}

show_version() {
    echo "Expected:  ${PINOCCHIO_VERSION}"
    echo "Installed: $(installed_version || echo none)"
}

remove_pinocchio() {
    rm -rf "${PINOCCHIO_PREFIX}"
    echo "Removed ${PINOCCHIO_PREFIX}"
}

purge_pinocchio() {
    rm -rf "${PINOCCHIO_DIR}" "${MICROMAMBA_DIR}" "${PRIVATE_MAMBA_ROOT}"
    echo "Removed all project-private Pinocchio and micromamba data."
}

case "${1:-}" in
    install)
        install_pinocchio
        ;;
    status)
        show_status
        ;;
    version)
        show_version
        ;;
    remove)
        remove_pinocchio
        ;;
    purge)
        purge_pinocchio
        ;;
    -h|--help|help|"")
        usage
        ;;
    *)
        echo "Unknown command: $1" >&2
        usage >&2
        exit 2
        ;;
esac