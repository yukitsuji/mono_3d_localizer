# Autoware Docker
[Docker installation](https://docs.docker.com/engine/installation/linux/docker-ce/ubuntu/)
[nvidia-docker installation](https://github.com/NVIDIA/nvidia-docker)

## How to Build
```
docker build -t ubuntu-kinetic -f . --no-cache
```

## How to Run
```
# Default shared directory path is /home/$USER/shared_dir

# Ubuntu 16.04 (Kinetic)
$ sh run.sh kinetic

# If you select your shared directory path
$ sh run.sh indigo|kinetic {SHARED_DIR_PATH}
```
