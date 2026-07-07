#!/usr/bin/env bash
set -e

if [ "$#" -lt 6 ]; then
    echo "usage: sitl_run.sh sitl_bin debugger model world src_path build_path"
    exit 1
fi

if [[ -n "$DONT_RUN" ]]; then
    echo "Not running simulation (DONT_RUN is set)."
    exit 0
fi

sitl_bin="$1"
debugger="$2"
model="$3"
world="$4"
src_path="$5"
build_path="$6"

echo "SITL ARGS"
echo "sitl_bin: $sitl_bin"
echo "debugger: $debugger"
echo "model: $model"
echo "world: $world"
echo "src_path: $src_path"
echo "build_path: $build_path"

rootfs="$build_path/rootfs"
mkdir -p "$rootfs"

# Determine airframe ID from airframe file name: {id}_{model}
# e.g. 4001_gz_x500 -> id=4001, model=gz_x500
AIRFRAME_ID=""
if [ -d "${src_path}/ROMFS/px4fmu_common/init.d-posix/airframes" ]; then
    AIRFRAME_ID=$(ls "${src_path}/ROMFS/px4fmu_common/init.d-posix/airframes" 2>/dev/null | grep "_${model}$" | grep -oE "^[0-9]+" | head -1)
fi

if [ -z "$AIRFRAME_ID" ]; then
    echo "ERROR: Could not find airframe ID for model: $model"
    echo "Looked in: ${src_path}/ROMFS/px4fmu_common/init.d-posix/airframes"
    exit 1
fi

echo "Airframe ID: $AIRFRAME_ID"

# Simulator environment
export PX4_SIMULATOR=gz
export PX4_GZ_WORLD=${world:-default}
export PX4_SIM_MODEL=${model}
export PX4_SYS_AUTOSTART=$AIRFRAME_ID

# Set SDF model path for Gazebo
export GAZEBO_MODEL_PATH="${src_path}/Tools/simulation/gz/models:${GAZEBO_MODEL_PATH}"

# Kill any previous instances
pkill -x gz || true
pkill -x px4 || true

SIM_PID=0

if [ -x "$(command -v gz)" ]; then
    # Determine world path
    if [ "$world" == "none" ] || [ -z "$world" ]; then
        if [ -f "${src_path}/Tools/simulation/gz/worlds/${model}.sdf" ]; then
            world_path="${src_path}/Tools/simulation/gz/worlds/${model}.sdf"
        else
            world_path="${src_path}/Tools/simulation/gz/worlds/default.sdf"
        fi
    else
        if [ -f "${src_path}/Tools/simulation/gz/worlds/${world}.sdf" ]; then
            world_path="${src_path}/Tools/simulation/gz/worlds/${world}.sdf"
        else
            world_path="$world"
        fi
    fi

    echo "Starting gz sim with world: $world_path"

    # Start gz sim (headless by default)
    gz sim -r "$world_path" &
    SIM_PID=$!

    # Wait for gz sim to be ready
    sleep 3

    # Spawn the model
    model_name="${model}"
    sdf_file="${src_path}/Tools/simulation/gz/models/${model}/${model}.sdf"

    if [ ! -f "$sdf_file" ]; then
        echo "ERROR: Model SDF not found: $sdf_file"
        kill -9 $SIM_PID 2>/dev/null || true
        exit 1
    fi

    echo "Spawning model $model_name from $sdf_file"

    retries=10
    while [ $retries -gt 0 ]; do
        if gz model -s -f "$sdf_file" -n "$model_name" 2>/dev/null; then
            echo "Model spawned successfully"
            break
        fi
        retries=$((retries - 1))
        echo "Waiting for gz sim... ($retries retries left)"
        sleep 1
    done

    if [ $retries -eq 0 ]; then
        echo "WARNING: Could not spawn model, proceeding anyway..."
    fi
else
    echo "ERROR: gz command not found. Please install Gazebo."
    exit 1
fi

# Build px4 command
if [[ -n "$NO_PXH" ]]; then
    no_pxh="-d"
else
    no_pxh=""
fi

if [[ -n "$VERBOSE_SIM" ]]; then
    verbose="--verbose"
else
    verbose=""
fi

pushd "$rootfs" >/dev/null
set +e

sitl_command="\"$sitl_bin\" $no_pxh $verbose \"$build_path\"/etc"
echo "SITL COMMAND: $sitl_command"

if [ "$debugger" == "lldb" ]; then
    eval lldb -- $sitl_command
elif [ "$debugger" == "gdb" ]; then
    eval gdb --args $sitl_command
elif [ "$debugger" == "valgrind" ]; then
    eval valgrind --track-origins=yes --leak-check=full -v $sitl_command
else
    eval $sitl_command
fi

popd >/dev/null
kill -9 $SIM_PID 2>/dev/null || true
echo "Simulation ended."
