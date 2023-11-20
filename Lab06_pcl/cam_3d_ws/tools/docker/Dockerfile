FROM osrf/ros:humble-desktop


# Customize bash
COPY git-prompt.sh /tmp/
COPY bashrc-git-prompt /tmp/
RUN cat /tmp/git-prompt.sh >> /etc/skel/.bashrc \
    && cat /tmp/bashrc-git-prompt >> /etc/skel/.bashrc \
    && echo "source /opt/ros/${ROS_DISTRO}/setup.bash" >> /etc/skel/.bashrc \
    && rm -rf /tmp/*


# Copy entrypoint
COPY entrypoint /entrypoint


# Clean up
RUN apt-get update && apt-get clean && DEBIAN_FRONTEND=noninteractive apt-get -y dist-upgrade \
    && rm -rf /var/lib/apt/lists/* \
    /root/.cache \
    /tmp/*


ENTRYPOINT ["/entrypoint"]
CMD ["/bin/bash"]
LABEL maintainer="Name Surname (name.surname@student.put.poznan.pl)"
