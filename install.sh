#!/usr/bin/env bash
# Register (or unregister) cwAsio4RtAudio as a cwASIO driver under /etc/cwASIO.
# The registration itself is done by the driver's registerDriver()/unregisterDriver(),
# called through the cwAsioRegister helper built alongside the driver.
# Run this as a normal user; sudo is used for the parts that need root.

set -euo pipefail

SCRIPT_DIR=$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)
REGISTRY=/etc/cwASIO
MAX_NAME_LEN=31     # ASIO limits driver names to 31 characters plus NUL

name=cwAsio4RtAudio
driver=
description=
config=user
uninstall=false
force=false

usage() {
    cat <<EOF
Usage: $(basename "$0") [options] [NAME]

Registers the cwAsio4RtAudio driver in $REGISTRY/NAME, so that cwASIO
applications can find it. NAME defaults to "cwAsio4RtAudio". Register the
driver several times under different names to get multiple instances.

Options:
  -d, --driver PATH       Path to cwAsio4RtAudioDriver.so. By default the
                          build directory next to this script is searched.
  -D, --description TEXT  Description shown to users. Without this option an
                          existing description is kept, otherwise it
                          defaults to "NAME: cwASIO for RtAudio devices".
  -c, --config WHERE      Where to put a copy of the example config file:
                          "user" (~/.cwASIO/NAME, default), "system"
                          ($REGISTRY/NAME) or "none". An existing config file
                          is kept unless --force is given.
  -f, --force             Overwrite an existing config file.
  -u, --uninstall         Remove the registration $REGISTRY/NAME instead.
                          The user config in ~/.cwASIO/NAME is left alone.
                          With --driver, only an entry for that driver is
                          removed.
  -h, --help              Show this help.
EOF
}

die() {
    echo "$(basename "$0"): $*" >&2
    exit 1
}

# run a command as root, using sudo unless we are root already
as_root() {
    if [[ $EUID -eq 0 ]]; then
        "$@"
    else
        sudo "$@"
    fi
}

args=()
while [[ $# -gt 0 ]]; do
    case $1 in
        -d|--driver)        driver=${2:?"$1 needs an argument"}; shift 2 ;;
        -D|--description)   description=${2?"$1 needs an argument"}; shift 2 ;;
        -c|--config)        config=${2:?"$1 needs an argument"}; shift 2 ;;
        -f|--force)         force=true; shift ;;
        -u|--uninstall)     uninstall=true; shift ;;
        -h|--help)          usage; exit 0 ;;
        --)                 shift; args+=("$@"); break ;;
        -*)                 usage >&2; exit 2 ;;
        *)                  args+=("$1"); shift ;;
    esac
done
[[ ${#args[@]} -le 1 ]] || { usage >&2; exit 2; }
[[ ${#args[@]} -eq 1 ]] && name=${args[0]}

[[ -n $name && $name != *"/"* && $name != "." && $name != ".." ]] || die "invalid driver name '$name'"
[[ ${#name} -le $MAX_NAME_LEN ]] || die "driver name '$name' is longer than $MAX_NAME_LEN characters"
case $config in user|system|none) ;; *) die "invalid --config value '$config'" ;; esac

entry=$REGISTRY/$name

# find the cwAsioRegister helper, preferably from the same build as the driver
find_helper() {
    local helper=$(dirname "$1")/cwAsioRegister
    [[ -x $helper ]] || helper=$(find "$SCRIPT_DIR/build" -name cwAsioRegister -type f -perm -u+x -print -quit 2>/dev/null)
    [[ -n $helper ]] || helper=$(command -v cwAsioRegister) || die "cwAsioRegister helper not found, build the project first"
    echo "$helper"
}

if $uninstall; then
    [[ -d $entry ]] || die "no driver registered under '$name'"
    if [[ -z $driver ]]; then
        # unregisterDriver() must be called from the driver that owns the entry. Only load it if
        # it is ours, as we'd run arbitrary code as root otherwise.
        driver=$(cat "$entry/driver" 2>/dev/null || true)
        [[ $(basename "$driver") == cwAsio4RtAudioDriver.so ]] \
            || die "'$name' is registered for a different driver (${driver:-none}), not removing it"
        [[ -f $driver ]] || die "registered driver $driver no longer exists, remove $entry by hand"
    fi
    helper=$(find_helper "$driver")
    as_root "$helper" -u "$driver" "$name"
    echo "Unregistered '$name'."
    exit 0
fi

# find the driver shared object
if [[ -z $driver ]]; then
    mapfile -t found < <(find "$SCRIPT_DIR/build" -name cwAsio4RtAudioDriver.so -type f 2>/dev/null)
    case ${#found[@]} in
        0) die "cwAsio4RtAudioDriver.so not found in $SCRIPT_DIR/build, build it first or use --driver" ;;
        1) driver=${found[0]} ;;
        *) die "several builds found, choose one with --driver:$(printf '\n  %s' "${found[@]}")" ;;
    esac
fi
[[ -f $driver ]] || die "driver '$driver' does not exist"
driver=$(realpath "$driver")

as_root mkdir -p "$REGISTRY"     # registerDriver() only creates the driver's own directory
helper=$(find_helper "$driver")
as_root "$helper" "$driver" "$name"
# keep an existing description unless a new one was given
if [[ -n $description || ! -e $entry/description ]]; then
    printf '%s' "${description:-$name: cwASIO for RtAudio devices}" | as_root tee "$entry/description" >/dev/null
fi
echo "Registered '$name' -> $driver"

case $config in
    user)   config_dir=$HOME/.cwASIO/$name ;;
    system) config_dir=$entry ;;
    none)   exit 0 ;;
esac
if [[ -e $config_dir/config ]] && ! $force; then
    echo "Keeping existing config file $config_dir/config (use --force to overwrite)."
elif [[ $config == system ]]; then
    as_root install -m 644 "$SCRIPT_DIR/config" "$config_dir/config"
    echo "Installed config file $config_dir/config"
else
    install -D -m 644 "$SCRIPT_DIR/config" "$config_dir/config"
    echo "Installed config file $config_dir/config"
fi
echo "Edit the config file to select the audio devices you want to use."
