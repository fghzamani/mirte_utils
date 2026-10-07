FROM althack/ros2:humble-cuda-gazebo-2026-07-01

RUN apt update
RUN apt upgrade -y
RUN apt install git -y
RUN apt install ros-humble-nav2* -y
RUN apt install ros-humble-pal* -y
RUN apt install ros-humble-tiago-simulation -y
# Install Scikit-Learn, Pandas, SciPy, and NumPy for Causal Model Learning
RUN apt-get update && apt-get install -y \
    python3-sklearn \
    python3-pandas \
    python3-scipy \
    python3-numpy \
    && rm -rf /var/lib/apt/lists/*
RUN echo "source /opt/ros/humble/setup.bash" >> /root/.bashrc


ENV NVIDIA_VISIBLE_DEVICES=all
ENV NVIDIA_DRIVER_CAPABILITIES=all
ENV __GLX_VENDOR_LIBRARY_NAME=nvidia
ENV __NV_PRIME_RENDER_OFFLOAD=1



# COPY ./test.py /app/test.py
# CMD ["python3", "/app/test.py"]

