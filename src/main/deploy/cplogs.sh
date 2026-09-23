#!/bin/bash
scp -r lvuser@10.94.96.2:logs/\* . && scp -r photon@10.94.96.11:/opt/photonvision/photonvision_config/imgSaves/ . && scp -r photon@10.94.96.13:/opt/photonvision/photonvision_config/imgSaves/ . && rm imgSaves/*/*_None-*.jpg
