# Ubuntu 20.04 and ROS Noetic
# Also Duckietown ROS Messages: https://github.com/duckietown/dt-ros-commons

ARG ARCH
ARG ON_JETSON
ARG DT_IMAGE_VERSION

FROM duckietown/dt-ros-commons:daffy-${ARCH}

ARG ARCH
ARG ON_JETSON

ENV ON_JETSON=${ON_JETSON}

# imgui is at beginning of Dockerfile to avoid re-running the slow commmand for other changes
# Install dependencies separately to avoid installing unwanted clashing dependencies
RUN sudo apt-get update && sudo apt-get install -y python3-pip && \
   python3 -m pip install --no-deps imgui_datascience && \
   python3 -m pip install imgui matplotlib pygame pyopengl xxhash

# Dependencies installation
COPY assets/requirements/* /tmp/
RUN apt-get update \
    && apt-get install -y \
    $(awk -F: '/^[^#]/ { print $1 }' /tmp/requirements-apt.txt | uniq) \
    && apt-get clean && rm -rf /var/lib/apt/lists/*
RUN python3 -m pip install -r /tmp/requirements-pip.txt
RUN rm /tmp/requirements*

# Install lightweight IDE SublimeText
RUN wget -qO - https://download.sublimetext.com/sublimehq-pub.gpg | gpg --dearmor | sudo tee /etc/apt/trusted.gpg.d/sublimehq-archive.gpg > /dev/null && \
    echo "deb https://download.sublimetext.com/ apt/stable/" | sudo tee /etc/apt/sources.list.d/sublime-text.list && \
    sudo apt-get update && sudo apt-get install -y sublime-text

# Build from source latest noetic-devel branch of rqt_plot
# this is needed to fix a bug in rqt_plot that prevents it from running using the qwtplot backend
# bug tracked in https://github.com/ros-visualization/rqt_plot/pull/70
RUN sudo apt-get remove -y ros-noetic-rqt-plot \
    && cd ${CATKIN_WS_DIR}/src \
    && git clone -b noetic-devel https://github.com/ros-visualization/rqt_plot \
    && sed -i.old '1s;^;import os\nos.environ["QT_API"] = "pyqt5"\n;' \
    /code/catkin_ws/src/rqt_plot/src/rqt_plot/data_plot/__init__.py && \
    . /opt/ros/${ROS_DISTRO}/setup.sh && \
    . ${CATKIN_WS_DIR}/devel/setup.bash  && \
    cd ${CATKIN_WS_DIR} && catkin build rqt_plot

# copy and install the gym-duckietown package
COPY assets/gym-duckietown/gym-duckietown /code/gym-duckietown
RUN cd /code/gym-duckietown && python3 setup.py sdist && \
    tar tfz dist/* && \
    python3 -m pip install dist/*

# Copy the RQT and RViz example config files
COPY assets/config/Duckietown_RQT.perspective assets/config/Duckietown_RVIZ.rviz ${CATKIN_WS_DIR}/src/

# Install noVNC and websockify
RUN mkdir -p /opt/novnc/utils/websockify && \
    wget -qO- https://github.com/novnc/noVNC/archive/refs/tags/v1.3.0.tar.gz | tar xz --strip-components=1 -C /opt/novnc && \
    wget -qO- https://github.com/novnc/websockify/archive/refs/tags/v0.9.0.tar.gz | tar xz --strip-components=1 -C /opt/novnc/utils/websockify

# Copy the custom novnc main page
COPY assets/vnc/custom-novnc.html /opt/novnc/vnc.html

# Symbolic link python3->python (for novnc-proxy) 
RUN ln -s /usr/bin/python3 /usr/bin/python
 
RUN /bin/bash -c 'set -ex && \
ARCH_LABEL=`uname -m` && \
mkdir /usr/lib/dri && \
ln -s /usr/lib/${ARCH_LABEL}-linux-gnu/dri/swrast_dri.so /usr/lib/dri/swrast_dri.so && \
ln -sf /usr/lib/${ARCH_LABEL}-linux-gnu/libdrm.so.2.4.0 /usr/lib/${ARCH_LABEL}-linux-gnu/libdrm.so.2'

# https://forums.developer.nvidia.com/t/jetpack-4-3-mesa-loader-failed-to-open-swrast-while-in-xrdp-session/111199/42
# Anyway, it will not symlink it correctly (even with -f flag). It seems the symlink has to be performed inside the running container.
# RUN rm /usr/lib/aarch64-linux-gnu/libdrm.so.2 && ln -sf /usr/lib/aarch64-linux-gnu/libdrm.so.2.4.0 /usr/lib/aarch64-linux-gnu/libdrm.so.2

# Set environment variables
ENV USER=root
ENV PASSWORD=quackquack

# Set up the VNC server
RUN mkdir -p /root/.vnc && \
    x11vnc -storepasswd $PASSWORD /root/.vnc/passwd

# Create the xstartup file
COPY assets/vnc/vnc-xstartup.sh /root/.vnc/xstartup
RUN  chmod +x /root/.vnc/xstartup

# Set up supervisor to manage VNC and noVNC
RUN mkdir -p /etc/supervisor/conf.d
COPY assets/vnc/supervisord.conf /etc/supervisor/conf.d/supervisord.conf

# (apt opencv and not pip opencv is required, to have better video codec support)   
RUN python3 -m pip uninstall -y opencv-python 

# Default terminal starting point
WORKDIR /code/catkin_ws
RUN echo "source /opt/ros/noetic/setup.bash" >> ~/.bashrc
RUN echo "source /code/catkin_ws/devel/setup.bash" >> ~/.bashrc

# --- apply some xfce4 desktop settings

# Disable blank screen by pre-configuring
# Define an ARG for the directory path
ARG CONFIG_DIR="/root/.config/xfce4/xfconf/xfce-perchannel-xml"
# Create the necessary directory structure
RUN mkdir -p ${CONFIG_DIR}
# Copy the pre-configured xfce4-power-manager.xml into the container
COPY assets/vnc/xfce4-power-manager.xml ${CONFIG_DIR}/xfce4-power-manager.xml
RUN chmod 644 ${CONFIG_DIR}/xfce4-power-manager.xml

# desktop icons
# desktop shortcuts for user_code folder and Sublime Text
COPY assets/vnc/desktop-shortcuts/* /root/Desktop/
RUN chmod +x /root/Desktop/user_code.desktop /root/Desktop/sublime_text.desktop
# hide home and Trash
COPY assets/vnc/xfce4-desktop.xml ${CONFIG_DIR}/xfce4-desktop.xml

# --- xfce4 desktop settings end

ARG DT_IMAGE_VERSION
ENV DT_IMAGE_VERSION=${DT_IMAGE_VERSION}
LABEL dt_image_version="${DT_IMAGE_VERSION}"

# Start supervisor
CMD ["/usr/bin/supervisord"]
