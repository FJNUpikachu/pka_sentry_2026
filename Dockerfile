FROM ros:humble-ros-base

# 1. 更新系统包并安装所有基础依赖 (新增了 ros-${ROS_DISTRO}-asio-cmake-module 等串口底层依赖)
RUN apt-get update && apt-get install -y \
    python3-pip \
    curl \
    wget \
    htop \
    vim \
    lsb-release \
    gnupg \
    libeigen3-dev \
    libomp-dev \
    libpcl-dev \
    ros-${ROS_DISTRO}-asio-cmake-module \
    ros-${ROS_DISTRO}-serial-driver \
    ros-${ROS_DISTRO}-rviz2 \
    && pip install xmacro vcstool2 \
    && rm -rf /var/lib/apt/lists/*

# 2. 添加 Gazebo/Ignition Fortress 官方源并安装
RUN curl -sSL https://packages.osrfoundation.org/gazebo.gpg --output /usr/share/keyrings/pkgs-osrf-archive-keyring.gpg && \
    echo "deb [arch=$(dpkg --print-architecture) signed-by=/usr/share/keyrings/pkgs-osrf-archive-keyring.gpg] http://packages.osrfoundation.org/gazebo/ubuntu-stable $(lsb_release -cs) main" > /etc/apt/sources.list.d/gazebo-stable.list && \
    apt-get update && \
    apt-get install -y ignition-fortress && \
    rm -rf /var/lib/apt/lists/*

# 3. 配置 Zsh终端环境
RUN sh -c "$(wget -O- https://github.com/deluan/zsh-in-docker/releases/download/v1.2.1/zsh-in-docker.sh)" -- \
    -t jispwoso -p git \
    -p https://github.com/zsh-users/zsh-autosuggestions \
    -p https://github.com/zsh-users/zsh-syntax-highlighting && \
    chsh -s /bin/zsh

# 4. 从源码安装 small_gicp
RUN mkdir -p /tmp/small_gicp && \
    cd /tmp && \
    git clone https://github.com/koide3/small_gicp.git && \
    cd small_gicp && \
    mkdir build && cd build && \
    cmake .. -DCMAKE_BUILD_TYPE=Release && \
    make -j$(nproc) && \
    make install && \
    rm -rf /tmp/small_gicp

# 5. 从源码安装 ndt_omp (核心修复：source setup.bash 以获取 ROS_VERSION 环境变量避免 CMake 报错)
RUN mkdir -p /tmp/ndt_omp && \
    cd /tmp && \
    git clone https://github.com/koide3/ndt_omp.git && \
    cd ndt_omp && \
    mkdir build && cd build && \
    /bin/bash -c "source /opt/ros/$ROS_DISTRO/setup.bash && cmake .. -DCMAKE_BUILD_TYPE=Release && make -j$(nproc) && make install" && \
    rm -rf /tmp/ndt_omp

# 6. 创建统一的工作空间并拷贝本地代码
RUN mkdir -p ~/ros_ws/src
WORKDIR /root/ros_ws

# 直接将宿主机的 src 目录拷贝进镜像的工作空间中
COPY ./src ./src

# 作为双保险：如果本地有遗漏的依赖模块，vcs import 会补充拉取（|| true 避免已存在的包报错停掉构建）
RUN vcs import src < src/rmu_gazebo_simulator/dependencies.repos || true

# 7. 使用 rosdepc (国内专供版) 替代原始的 rosdep，彻底解决网络超时问题
RUN pip install rosdepc && \
    rosdepc init || true

# 8. 更新 rosdepc 并统一安装 src 目录下所有的 ROS 2 依赖
RUN apt-get update && \
    rosdepc update && \
    rosdepc install -r --from-paths src --ignore-src --rosdistro $ROS_DISTRO -y && \
    rm -rf /var/lib/apt/lists/*

# 9. 统一编译工作空间
RUN /bin/bash -c "source /opt/ros/$ROS_DISTRO/setup.bash && colcon build --symlink-install --cmake-args -DCMAKE_BUILD_TYPE=Release --parallel-workers 2"

# 10. 配置环境变量 (.zshrc)
RUN echo 'export TERM=xterm-256color\n\
source ~/ros_ws/install/setup.zsh\n\
eval "$(register-python-argcomplete3 ros2)"\n\
eval "$(register-python-argcomplete3 colcon)"\n'\
>> /root/.zshrc

# 11. 将工作空间注入 ROS entrypoint
RUN sed --in-place --expression \
      '$isource "/root/ros_ws/install/setup.bash"' \
      /ros_entrypoint.sh

# 默认启动 Zsh
CMD [ "/bin/zsh" ]
