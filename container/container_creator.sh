#!/bin/bash
set -eu

usage() {
    echo "create_container.sh [--help] [--name container_name] [--cmd container_command]"
    echo "                    [--mount-path path_to_write] [--dockerfile path] [--rebuild] [--robot]"
    echo ""
    echo "  --name        suffix for the container name (default: dev)"
    echo "  --cmd         command to run inside the container (default: bash)"
    echo "  --mount-path  host folder mounted read-write (default: \$HOME/phd_projects)"
    echo "  --dockerfile  path to the Dockerfile to build from (default: ./Dockerfile)"
    echo "  --rebuild     force rebuild of the image even if it already exists"
    echo "  --robot       talk to the real robot over eth0 instead of localhost-only simulation"
    exit "${1:-1}";
}

o_rebuild=0
o_robot=0
while [ $# -gt 0 ]; do
    case "$1" in
        -h|--help)    usage 0;;
        --name)       o_name=${2?argument missing for --name}; shift 2;;
        --cmd)        o_cmd=${2?argument missing for --cmd}; shift 2;;
        --mount-path) o_mpath=${2?argument missing for --mount-path}; shift 2;;
        --dockerfile) o_dfile=${2?argument missing for --dockerfile}; shift 2;;
        --rebuild)    o_rebuild=1; shift;;
        --robot)      o_robot=1; shift;;
        -*)
            echo "unknown option '$1'" >&2; usage;;
        *)  shift;;
    esac
done

# ----- image / container names (all local, no registry) -----------------------
image="ros2-humble-mirte:dev"
dockerfile=${o_dfile:-"./Dockerfile"}

# ----- build the image if it is missing (or if --rebuild was passed) ----------
if [ "$o_rebuild" -eq 1 ] || ! docker image inspect "$image" >/dev/null 2>&1; then
    if [ ! -f "$dockerfile" ]; then
        echo "Dockerfile '$dockerfile' not found." >&2
        echo "Pass its location with --dockerfile, or run this script from the folder containing it." >&2
        exit 1
    fi
    echo "Building image '$image' from '$dockerfile'..."
    docker build -t "$image" -f "$dockerfile" "$(dirname "$dockerfile")"
else
    echo "Image '$image' already exists (use --rebuild to force a rebuild)."
fi

# ----- resolve the real user, even under sudo ---------------------------------
user=${SUDO_USER:-$USER}
home=$(eval echo "~$user")

# writable folder (defaults to ~/phd_projects), created if missing
path=${o_mpath:-"$home/phd_projects"}
mkdir -p "$path"

container="ros2-tiago-${o_name:-dev}"
if docker container inspect "$container" >/dev/null 2>&1; then
    echo "container '$container' already exists" >&2
    echo "You can remove it with 'docker rm $container'" >&2
    exit 1
fi

# ----- run options ------------------------------------------------------------
opts=(
    # whole home visible read-only, phd_projects writable on top of it
    --volume "$home:$home:ro"
    --volume "$path:$path:rw"
    --workdir "$home"

    # X11 / Gazebo GUI forwarding
    --env DISPLAY
    --env QT_X11_NO_MITSHM=1
    --volume /tmp/.X11-unix:/tmp/.X11-unix:rw

    --net host

    # If you want files created in phd_projects to be owned by YOU on the host
    # instead of root, uncomment the next line. Note: the althack image runs as
    # root and some ROS tooling expects a writable home, so test before relying
    # on this.
    # --user "$(id -u "$user"):$(id -g "$user")"
)

# ----- ROS 2 networking: simulation (localhost only) or real robot (eth0) ----
if [ "$o_robot" -eq 1 ]; then
    fastdds_xml="$path/fastdds_eth0.xml"
    if [ ! -f "$fastdds_xml" ]; then
        echo "'$fastdds_xml' not found; create it before using --robot." >&2
        exit 1
    fi
    opts+=(
        --ipc host
        --env ROS_LOCALHOST_ONLY=0
        --env ROS_DOMAIN_ID=0
        --env RMW_IMPLEMENTATION=rmw_fastrtps_cpp
        --env FASTRTPS_DEFAULT_PROFILES_FILE="$fastdds_xml"
    )
    echo "Robot mode: ROS 2 traffic restricted to eth0 via $fastdds_xml"
else
    opts+=(--env ROS_LOCALHOST_ONLY=1)
fi

# ----- GPU acceleration (needs the NVIDIA Container Toolkit on the host) -------
if [ -n "$(command -v nvidia-container-runtime)" ]; then
    opts+=(--gpus all)
    echo "Using GPU acceleration"
else
    echo "No GPU acceleration available (install nvidia-container-toolkit to enable)"
fi

# ----- create the container ---------------------------------------------------
read -r -a cmd <<< "${o_cmd:-bash}"
docker create "${opts[@]}" -it --name "$container" "$image" "${cmd[@]}" > /dev/null

echo ""
echo "Container '$container' created."
echo "Before starting it (once per login), allow X access:  xhost +local:docker"
echo "Then start it with:                                   docker start -ai $container"
