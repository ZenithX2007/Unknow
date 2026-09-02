FROM ros:humble

# Avoid interactive prompts during build
ENV DEBIAN_FRONTEND=noninteractive

# Install curl first
RUN apt update && apt install -y curl jq wget

# Create workspace
RUN mkdir -p /root/gen0_gz_sim_ros2/src

# Copy to workspace
COPY . /root/gen0_gz_sim_ros2/src/gen0_gz_sim_ros2/

WORKDIR /root/gen0_gz_sim_ros2

# Download san_full.obj from GitHub release
RUN wget -O /tmp/san_full.obj $(curl -s https://api.github.com/repos/AV-Lab/gen0_gz_sim_ros2/releases/tags/0.1.0 | \
    jq -r '.assets[] | select(.name == "san_full.obj") | .browser_download_url') && \
    cp /tmp/san_full.obj /root/gen0_gz_sim_ros2/src/gen0_gz_sim_ros2/gen0_gz_sim_ros2/gen0_main/worlds/san_full/san_full.obj

# Add OSRF GPG key and Gazebo apt repository
RUN curl -sSL https://packages.osrfoundation.org/gazebo.gpg -o /usr/share/keyrings/pkgs-osrf-archive-keyring.gpg && \
    echo "deb [arch=$(dpkg --print-architecture) signed-by=/usr/share/keyrings/pkgs-osrf-archive-keyring.gpg] http://packages.osrfoundation.org/gazebo/ubuntu-stable $(lsb_release -cs) main" \
    > /etc/apt/sources.list.d/gazebo-stable.list

# Install dependencies
RUN apt update && \
    apt install -y \
    python3-pip \
    lsb-release \
    gnupg \
    ignition-fortress \
    ros-humble-ros-gz \
    ros-humble-tf-transformations \
    ros-humble-vision-msgs

# Install Python dependencies
COPY requirements.txt /tmp/
RUN pip3 install -r /tmp/requirements.txt

# Build the packages
RUN . /opt/ros/humble/setup.sh && \
    colcon build

# Add workspace setup to .bashrc for convenience
RUN echo "source ~/gen0_gz_sim_ros2/install/setup.bash" >> /root/.bashrc

# Use the default entrypoint from the ros:humble image
CMD ["bash"]

