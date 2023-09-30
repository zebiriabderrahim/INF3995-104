#!/bin/sh

if [ -n "$DESTDIR" ] ; then
    case $DESTDIR in
        /*) # ok
            ;;
        *)
            /bin/echo "DESTDIR argument must be absolute... "
            /bin/echo "otherwise python's distutils will bork things."
            exit 1
    esac
fi

echo_and_run() { echo "+ $@" ; "$@" ; }

echo_and_run cd "/home/nvidia/INF3995-104/embedded/agilex_ws/src/src/camera_info_manager_py"

# ensure that Python install destination exists
echo_and_run mkdir -p "$DESTDIR/home/nvidia/INF3995-104/embedded/agilex_ws/install/lib/python3/dist-packages"

# Note that PYTHONPATH is pulled from the environment to support installing
# into one location when some dependencies were installed in another
# location, #123.
echo_and_run /usr/bin/env \
    PYTHONPATH="/home/nvidia/INF3995-104/embedded/agilex_ws/install/lib/python3/dist-packages:/mnt/home/nvidia/INF3995-104/embedded/agilex_ws/build/lib/python3/dist-packages:$PYTHONPATH" \
    CATKIN_BINARY_DIR="/mnt/home/nvidia/INF3995-104/embedded/agilex_ws/build" \
    "/usr/bin/python3" \
    "/home/nvidia/INF3995-104/embedded/agilex_ws/src/src/camera_info_manager_py/setup.py" \
     \
    build --build-base "/mnt/home/nvidia/INF3995-104/embedded/agilex_ws/build/src/camera_info_manager_py" \
    install \
    --root="${DESTDIR-/}" \
    --install-layout=deb --prefix="/home/nvidia/INF3995-104/embedded/agilex_ws/install" --install-scripts="/home/nvidia/INF3995-104/embedded/agilex_ws/install/bin"
