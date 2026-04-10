FROM osrf/ros:noetic-desktop-full-focal

ENV DEBIAN_FRONTEND=noninteractive
ENV TZ=America/Phoenix
ENV LANG=C.UTF-8
ENV LC_ALL=C.UTF-8
ENV ROS_DISTRO=noetic
ENV PORTABLE_ROS_USER=portable
ENV PORTABLE_ROS_HOME=/home/portable
ENV HOME=/home/portable

SHELL ["/bin/bash", "-lc"]

RUN apt-get update && apt-get install -y --no-install-recommends \
    tzdata \
    rsync \
    expect \
    mesa-utils \
    x11-apps \
    libgl1 \
    libgl1-mesa-dri \
    libglu1-mesa \
    libglfw3 \
    libglew2.1 \
    libassimp5 \
    libomp5 \
    libgoogle-glog0v5 \
    libgflags2.2 \
    libatlas3-base \
    libsuitesparse-dev \
    python3 \
    python3-pip \
    python3-numpy \
    python3-opencv \
    python3-matplotlib \
    python3-catkin-tools \
    python3-rosdep \
    python3-rosinstall \
    python3-rosinstall-generator \
    python3-wstool \
    ros-noetic-cv-bridge \
    ros-noetic-pcl-ros \
    ros-noetic-image-transport \
    ros-noetic-tf \
    ros-noetic-tf2-ros \
    ros-noetic-rosbag \
    ros-noetic-rviz \
  && ln -fs /usr/share/zoneinfo/${TZ} /etc/localtime \
  && dpkg-reconfigure -f noninteractive tzdata \
  && rm -rf /var/lib/apt/lists/*

RUN id -u "${PORTABLE_ROS_USER}" >/dev/null 2>&1 || useradd -m -d "${PORTABLE_ROS_HOME}" -s /bin/bash "${PORTABLE_ROS_USER}"

WORKDIR /workspace

COPY rt/calib ${PORTABLE_ROS_HOME}/ws_calib
COPY rt/livox ${PORTABLE_ROS_HOME}/ws_livox
COPY rt/iri ${PORTABLE_ROS_HOME}/iridescence
COPY rt/lib ${PORTABLE_ROS_HOME}/lib
COPY rt/usr/lib /usr/local/lib
COPY rt/usr/share /usr/local/share
COPY overrides /opt/portable_ros_stack/overrides
COPY docker/ros_entrypoint.sh /ros_entrypoint.sh

RUN chmod +x /ros_entrypoint.sh \
  && mkdir -p /home/leif \
  && ln -sfn "${PORTABLE_ROS_HOME}/ws_livox" /home/leif/ws_livox \
  && ln -sfn "${PORTABLE_ROS_HOME}/ws_calib" /home/leif/ws_calib \
  && if [[ -d /opt/portable_ros_stack/overrides/ws_calib/scripts ]]; then rsync -a /opt/portable_ros_stack/overrides/ws_calib/scripts/ "${PORTABLE_ROS_HOME}/ws_calib/scripts/"; fi \
  && if [[ -d /opt/portable_ros_stack/overrides/ws_livox/scripts ]]; then rsync -a /opt/portable_ros_stack/overrides/ws_livox/scripts/ "${PORTABLE_ROS_HOME}/ws_livox/scripts/"; fi \
  && if [[ -f /usr/local/lib/libgtsam.so.4.2.0 ]]; then ln -sf libgtsam.so.4.2.0 /usr/local/lib/libgtsam.so.4 && ln -sf libgtsam.so.4 /usr/local/lib/libgtsam.so; fi \
  && if [[ -f /usr/local/lib/libgtsam_unstable.so.4.2.0 ]]; then ln -sf libgtsam_unstable.so.4.2.0 /usr/local/lib/libgtsam_unstable.so.4 && ln -sf libgtsam_unstable.so.4 /usr/local/lib/libgtsam_unstable.so; fi \
  && if [[ -f /usr/local/lib/libiridescence.so.1.0.1 ]]; then ln -sf libiridescence.so.1.0.1 /usr/local/lib/libiridescence.so.1 && ln -sf libiridescence.so.1 /usr/local/lib/libiridescence.so; fi \
  && if [[ -f "${PORTABLE_ROS_HOME}/iridescence/build/libiridescence.so.1.0.1" ]]; then ln -sf libiridescence.so.1.0.1 "${PORTABLE_ROS_HOME}/iridescence/build/libiridescence.so.1" && ln -sf libiridescence.so.1 "${PORTABLE_ROS_HOME}/iridescence/build/libiridescence.so"; fi \
  && if compgen -G "${PORTABLE_ROS_HOME}/ws_calib/scripts/*.sh" > /dev/null; then chmod +x "${PORTABLE_ROS_HOME}"/ws_calib/scripts/*.sh; fi \
  && if compgen -G "${PORTABLE_ROS_HOME}/ws_livox/scripts/*.sh" > /dev/null; then chmod +x "${PORTABLE_ROS_HOME}"/ws_livox/scripts/*.sh; fi \
  && ldconfig \
  && mkdir -p "${PORTABLE_ROS_HOME}/.config/rosbag_preprocessing" \
  && chown -R "${PORTABLE_ROS_USER}:${PORTABLE_ROS_USER}" "${PORTABLE_ROS_HOME}"

USER portable

ENTRYPOINT ["/ros_entrypoint.sh"]
CMD ["bash"]
