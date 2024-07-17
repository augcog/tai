#!/bin/bash

# pull from github
git pull

# Stop and remove the current container if it exists
sudo docker-compose down

# Build the new image
sudo docker build -t berkeley_tai/frontend .

# Start the new container
sudo docker-compose up -d