FROM ros


RUN apt-get update && apt-get install -y python3-pip \
      ros-${ROS_DISTRO}-demo-nodes-cpp \
      ros-${ROS_DISTRO}-demo-nodes-py && \
      rm -rf /var/lib/apt/lists/* && \
      pip install -i https://pypi.tuna.tsinghua.edu.cn/simple setuptools==58.2.0

RUN apt-get update && apt-get install -y iputils-ping procps vim net-tools nmap && rm -rf /var/lib/apt/lists/* 

ADD robo_msgs.tar /root/workspaces/src/
COPY nano /root/workspaces/src/nano
COPY map /home/map


RUN echo '#!/bin/bash \n \
source /ros_entrypoint.sh \n tail -f /dev/null' > entrypoint.sh && chmod +x entrypoint.sh


ENTRYPOINT [ "./entrypoint.sh" ]
