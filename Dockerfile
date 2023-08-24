FROM ros:humble

WORKDIR /root

# Change bash as default shell instead of sh
SHELL ["/bin/bash", "-c"]

# Install some tools, eigen and ceres installation dependencies.
RUN apt-get update && apt-get -y install \
    vim wget curl unzip \
    build-essential \
    cmake \
    make ninja-build \
    libgoogle-glog-dev libgflags-dev \
    libatlas-base-dev \
    libeigen3-dev \
    libsuitesparse-dev \
    && rm -rf /var/lib/apt/lists/*

# Install ceres 2.1.0
RUN mkdir ceres_tmp && cd ceres_tmp && \
    wget http://ceres-solver.org/ceres-solver-2.1.0.tar.gz && \
    tar zxf ceres-solver-2.1.0.tar.gz && \
    mkdir build && cd build && \
    cmake ../ceres-solver-2.1.0 && \
    make -j4 && \
    make install && \
    cd ../.. && rm -rf ./ceres_tmp

# Install openvino 2023 runtime (C++ API only)
RUN wget https://apt.repos.intel.com/intel-gpg-keys/GPG-PUB-KEY-INTEL-SW-PRODUCTS.PUB && \
    apt-key add GPG-PUB-KEY-INTEL-SW-PRODUCTS.PUB && \
    echo "deb https://apt.repos.intel.com/openvino/2023 ubuntu22 main" | tee /etc/apt/sources.list.d/intel-openvino-2023.list && \
    apt-get update && \
    apt-cache search openvino && \
    apt-get install -y openvino-2023.0.1 && \
    rm -rf /var/lib/apt/lists/* && \
    rm ./GPG-PUB-KEY-INTEL-SW-PRODUCTS.PUB

# Install HikCamera SDK (runtime libraries only)
RUN mkdir mvs_tmp && cd mvs_tmp && \
    wget https://www.hikrobotics.com/cn2/source/support/software/MVS_STD_GML_V2.1.2_221208.zip && \
    unzip MVS_STD_GML_V2.1.2_221208.zip && \
    tar -zxf MVS-2.1.2_x86_64_20221208.tar.gz && \
    tar -zxf MVS-2.1.2_x86_64_20221208/MVS.tar.gz && \
    mkdir -p /opt/MVS && cp -r ./MVS/{lib,include} /opt/MVS && \
    cd .. && rm -rf ./mvs_tmp

# Install opencv 4.8.0 with contrib
RUN mkdir opencv_tmp && cd opencv_tmp && \
    wget -O opencv.tar.gz https://github.com/opencv/opencv/archive/refs/tags/4.8.0.tar.gz && \
    wget -O opencv_contrib.tar.gz https://github.com/opencv/opencv_contrib/archive/refs/tags/4.8.0.tar.gz && \
    tar -zxf ./opencv.tar.gz && tar -zxf ./opencv_contrib.tar.gz && \
    mkdir build && cd build && \
    cmake -DOPENCV_EXTRA_MODULES_PATH=../opencv_contrib-4.8.0/modules ../opencv-4.8.0 && \
    make -j4 && make install && \
    cd ../.. && rm -rf ./opencv_tmp

# Install deploy tools, set root password and configure ssh server
RUN apt-get update && apt-get -y install \
    rsync \
    screen \
    net-tools \
    openssh-server \
    && rm -rf /var/lib/apt/lists/* && \
    echo 'root:alliance' | chpasswd && \
    echo "Port 22" >> /etc/ssh/sshd_config && \
    echo "PasswordAuthentication yes" >> /etc/ssh/sshd_config && \
    echo "PermitRootLogin yes" >> /etc/ssh/sshd_config

# Add .ugasrc
RUN tee .ugasrc <<_EOF_
export MVCAM_SDK_PATH=/opt/MVS
export MVCAM_COMMON_RUNENV=/opt/MVS/lib
export LD_LIBRARY_PATH=/opt/MVS/lib/64:/opt/MVS/lib/32:\$LD_LIBRARY_PATH
export LD_LIBRARY_PATH=\$LD_LIBRARY_PATH:/usr/local/lib
echo "UGAS Environment Initilized"
_EOF_

# Add tini
RUN wget -O /tini https://github.com/krallin/tini/releases/download/v0.19.0/tini && \
    chmod +x /tini

COPY shell/docker-init.sh /root/init.sh

EXPOSE 22

ENTRYPOINT ["/tini", "--"]
CMD ["/root/init.sh", "--entry"]
