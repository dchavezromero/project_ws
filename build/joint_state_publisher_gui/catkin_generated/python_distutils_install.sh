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

echo_and_run cd "/home/dennis/project_ws/src/joint_state_publisher/joint_state_publisher_gui"

# ensure that Python install destination exists
echo_and_run mkdir -p "$DESTDIR/home/dennis/project_ws/install/lib/python2.7/dist-packages"

# Note that PYTHONPATH is pulled from the environment to support installing
# into one location when some dependencies were installed in another
# location, #123.
echo_and_run /usr/bin/env \
    PYTHONPATH="/home/dennis/project_ws/install/lib/python2.7/dist-packages:/home/dennis/project_ws/build/joint_state_publisher_gui/lib/python2.7/dist-packages:$PYTHONPATH" \
    CATKIN_BINARY_DIR="/home/dennis/project_ws/build/joint_state_publisher_gui" \
    "/usr/bin/python2" \
    "/home/dennis/project_ws/src/joint_state_publisher/joint_state_publisher_gui/setup.py" \
    egg_info --egg-base /home/dennis/project_ws/build/joint_state_publisher_gui \
    build --build-base "/home/dennis/project_ws/build/joint_state_publisher_gui" \
    install \
    --root="${DESTDIR-/}" \
    --install-layout=deb --prefix="/home/dennis/project_ws/install" --install-scripts="/home/dennis/project_ws/install/bin"
