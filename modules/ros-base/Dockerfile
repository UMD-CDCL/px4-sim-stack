# No dependency downloads here. Prepare local parents with ./px4sim deps.
# Each package is an independent branch; assembly copies only installed files.
ARG DS_TAG=7.1
FROM px4simstack/ros-deps:${DS_TAG} AS deps

FROM px4simstack/yolo-deps:${DS_TAG} AS parser
COPY --from=yolo_src / /src
RUN --network=none set -eux; \
    cmp /src/cuda-version /opt/ds-yolo/cuda-requested; \
    make -C /src "CUDA_VER=$(cat /opt/ds-yolo/cuda-version)" -j4; \
    mkdir -p /out; \
    cp /src/libnvdsinfer_custom_impl_Yolo.so /out/

FROM deps AS mavros
ARG MAVROS_PATCH=1
RUN --network=none \
    --mount=type=bind,from=mavros_patch_src,target=/tmp/chimera-deploy/remote/mavros_patch \
    --mount=type=bind,from=mavros_src,target=/tmp/chimera-deploy/submodules/mavros \
    --mount=type=bind,from=angles_src,target=/tmp/chimera-deploy/submodules/angles \
    # mavros_patch/apply.sh copies geographic_msgs from the geographic_info
    # submodule path, so mirror that layout in the isolated build mount.
    --mount=type=bind,from=geographic_msgs_src,target=/tmp/chimera-deploy/submodules/geographic_info/geographic_msgs \
    set -eux; \
    mkdir -p /opt/mavros/install; \
    if [ "${MAVROS_PATCH}" = 1 ] && [ "${ROS_DISTRO}" = humble ]; then \
        MAKEFLAGS=-j4 MAVROS_PATCH_WS=/opt/mavros \
            bash /tmp/chimera-deploy/remote/mavros_patch/apply.sh; \
    fi

FROM deps AS px4_msgs
ARG TARGETARCH
ARG DS_TAG
ARG ROS_BUILD_JOBS=4
WORKDIR /home/user/ros2_ws
COPY --from=px4_msgs_src / src/px4_msgs/
RUN --network=none \
    --mount=type=cache,id=ros-ccache-px4-msgs-${DS_TAG}-${TARGETARCH},target=/root/.cache/ccache,sharing=locked \
    . /opt/ros/${ROS_DISTRO}/setup.sh \
    && MAKEFLAGS="-j${ROS_BUILD_JOBS}" colcon build --executor sequential \
       --packages-select px4_msgs \
       --cmake-args -DCMAKE_BUILD_TYPE=Release -DBUILD_TESTING=OFF \
                    -DCMAKE_C_COMPILER_LAUNCHER=ccache -DCMAKE_CXX_COMPILER_LAUNCHER=ccache

FROM deps AS cdcl_umd_msgs
ARG TARGETARCH
ARG DS_TAG
ARG ROS_BUILD_JOBS=4
WORKDIR /home/user/ros2_ws
COPY --from=cdcl_umd_msgs_src / src/cdcl_umd_msgs/
RUN --network=none \
    --mount=type=cache,id=ros-ccache-cdcl-msgs-${DS_TAG}-${TARGETARCH},target=/root/.cache/ccache,sharing=locked \
    . /opt/ros/${ROS_DISTRO}/setup.sh \
    && MAKEFLAGS="-j${ROS_BUILD_JOBS}" colcon build --executor sequential \
       --packages-select cdcl_umd_msgs \
       --cmake-args -DCMAKE_BUILD_TYPE=Release -DBUILD_TESTING=OFF \
                    -DCMAKE_C_COMPILER_LAUNCHER=ccache -DCMAKE_CXX_COMPILER_LAUNCHER=ccache

FROM deps AS mavinsight
ARG TARGETARCH
ARG DS_TAG
ARG ROS_BUILD_JOBS=4
WORKDIR /home/user/ros2_ws
COPY --from=mavinsight_src / src/mavinsight/
RUN --network=none \
    --mount=type=cache,id=ros-ccache-mavinsight-${DS_TAG}-${TARGETARCH},target=/root/.cache/ccache,sharing=locked \
    . /opt/ros/${ROS_DISTRO}/setup.sh \
    && MAKEFLAGS="-j${ROS_BUILD_JOBS}" colcon build --executor sequential \
       --packages-select mavinsight \
       --cmake-args -DCMAKE_BUILD_TYPE=Release -DBUILD_TESTING=OFF \
                    -DCMAKE_C_COMPILER_LAUNCHER=ccache -DCMAKE_CXX_COMPILER_LAUNCHER=ccache

FROM deps AS tracking_test
ARG TARGETARCH
ARG DS_TAG
ARG ROS_BUILD_JOBS=4
WORKDIR /home/user/ros2_ws
COPY --from=tracking_test_src / src/tracking_test/
RUN --network=none \
    . /opt/ros/${ROS_DISTRO}/setup.sh \
    && colcon build --executor sequential \
       --packages-select tracking_test

FROM deps AS umd_uas
ARG TARGETARCH
ARG DS_TAG
ARG ROS_BUILD_JOBS=4
WORKDIR /home/user/ros2_ws
COPY --from=umd_uas_src / src/umd_uas/
RUN --network=none \
    --mount=type=cache,id=ros-ccache-umd-uas-${DS_TAG}-${TARGETARCH},target=/root/.cache/ccache,sharing=locked \
    . /opt/ros/${ROS_DISTRO}/setup.sh \
    && MAKEFLAGS="-j${ROS_BUILD_JOBS}" colcon build --executor sequential \
       --packages-select umd_uas \
       --cmake-args -DCMAKE_BUILD_TYPE=Release -DBUILD_TESTING=OFF \
                    -DCMAKE_C_COMPILER_LAUNCHER=ccache -DCMAKE_CXX_COMPILER_LAUNCHER=ccache

FROM deps AS runtime
ARG HOST_UID=1000
ARG HOST_GID=1000
ARG USERNAME=user
# Each prefix is isolated. The root colcon setup utilities discover the union
# of installed package markers and order runtime dependencies when sourced.
COPY --link --from=px4_msgs /home/user/ros2_ws/install/ /home/user/ros2_ws/install/
COPY --link --from=mavros /opt/mavros/install/ /opt/mavros/install/
COPY --link --from=parser /out/libnvdsinfer_custom_impl_Yolo.so /opt/ds-yolo/libnvdsinfer_custom_impl_Yolo.so
COPY --link --from=cdcl_umd_msgs /home/user/ros2_ws/install/ /home/user/ros2_ws/install/
COPY --link --from=mavinsight /home/user/ros2_ws/install/ /home/user/ros2_ws/install/
COPY --link --from=tracking_test /home/user/ros2_ws/install/ /home/user/ros2_ws/install/
COPY --link --from=umd_uas /home/user/ros2_ws/install/ /home/user/ros2_ws/install/
COPY --link site-params.sh ros-env.sh /usr/local/bin/

# COPY --link can recreate destination path components as root. Restore the
# runtime user's ownership of its home so ROS and other tools can create
# per-user state such as ~/.ros, ~/.cache, and ~/.config.
RUN chown "${HOST_UID}:${HOST_GID}" /home/user

USER ${USERNAME}
WORKDIR /home/user/ros2_ws
