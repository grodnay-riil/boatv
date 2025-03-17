#!/bin/bash
# Update package index and install Docker
apt-get update -y
apt-get install -y docker.io

# Start and enable the Docker service
systemctl start docker
systemctl enable docker

# Pull and run the Zenoh router container with PSK authentication
docker run -d --restart unless-stopped --name zenoh-router -p 7447:7447/udp eclipse/zenoh:latest zenohd --mode router --listen udp/0.0.0.0:7447 --security psk:stingray
