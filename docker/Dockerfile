FROM osrf/ros:noetic-desktop-focal

# install xauth for gui applications
RUN apt-get update && apt-get install -q -y --no-install-recommends \
    xauth \
    && rm -rf /var/lib/apt/lists/*

# install packages
RUN apt-get update && apt-get install -q -y --no-install-recommends \
    build-essential \
    cmake \
    git \
    libpoco-dev \
    libeigen3-dev \
    net-tools \
    nano \
    wget \
    fonts-powerline \
    zsh \
    tree \
    qtbase5-dev \
    && rm -rf /var/lib/apt/lists/*


# create catkin_ws
RUN mkdir -p /home/catkin_ws/src

# install zsh and oh-my-zsh (https://gist.github.com/MichalZalecki/4a87880bbe7a3a5428b5aebebaa5cd97)
ENV TERM xterm
ENV ZSH_THEME agnoster

# run the installation script  
RUN wget https://github.com/robbyrussell/oh-my-zsh/raw/master/tools/install.sh -O - | zsh || true    

# zsh
COPY ./setup.zsh /
RUN echo "source /setup.zsh">>/root/.zshrc

CMD [ "zsh" ]
