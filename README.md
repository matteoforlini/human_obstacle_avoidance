# Experimental Implementation of Skeleton Tracking for Collision Avoidance in Collaborative Robotics

In this repository are uploaded all the codes to perform an human obstacle avoidance with the Universal Robot 5e manipulator. The human pose detection is performed with 3 Intelrealsense D455 RGB-D cameras using a tool based on Machine Learining technique,
Mediapipe as skeleton detection system.

The project is composed of three main agents:
- Skeleton Detection performed by a standard PC (PC 1);
- Obstacle avoidance algorithm performed by another standard PC (PC 2);
- Universal Robot 5e with his programm used to receive commands from (PC 2).

Among the three components there is TCP/IP connection, if you set the following IP adresses to the agents you can use the programm directly without changing adresses inside the codes:
- PC 1 ->169.254.0.20
- PC 2 ->169.254.0.25
- Robot UR5e ->169.254.0.4

## A fully functional EXAMPLE project written in Go showing how to create a REST API and deploy to Kubernetes!

This project is an example that was built along with a Youtube Series showing how to create a custom REST API in Golang. Every part of this project is sample code which shows how to do the following:

- Create a custom web server with Golang using HTTPRouter
- Create a simple REST API that serves Youtube stats (as an example use case)
- Automatically containerize and push a Docker container to Docker hub using Github Actions
- Create a Kubernetes Helm chart for deploying your API to Kubernetes
- Push your Helm chart to a Helm Chart repo using Github Actions and Github Pages (coming soon)
- How to write a good README (you are reading it now!)

## Watch the full series!

<a href="https://www.youtube.com/playlist?list=PLSvCAHoiHC_rqKbcu1ummWVpLTDBNZHH7" target="_blank">
<img src="http://img.youtube.com/vi/SiGxu2N9ndU/mqdefault.jpg" alt="Watch the series" width="240" height="180" border="10" />
</a>

## How to install this example Golang project on Kubernetes

The easiest way to see how this works is to watch the series, but here's the short version.

1. clone this project
2. Set up a local kubernetes cluster - https://askcloudarchitech.com/posts/tutorials/local-kubernetes-startup-script/
3. Install helm - `brew install helm`
4. Setup your keys - see the video here - https://www.youtube.com/watch?v=k0L_yR30PqI&list=PLSvCAHoiHC_rqKbcu1ummWVpLTDBNZHH7&index=5
5. Install with Helm

## How to tweak this project for your own uses

Since this is an example project, I'd encourage you to clone and rename this project to use for your own puposes. It's a good starter boilerplate

## Find a bug?

If you found an issue or would like to submit an improvement to this project, please submit an issue using the issues tab above. If you would like to submit a PR with a fix, reference the issue you created!

## Known issues (Work in progress)

This tutorial is till ongoing. The automation of the helm chart repo has not been completed yet. This is coming soon!

## Like this project?

If you are feeling generous, buy me a coffee! - https://www.buymeacoffee.com/askcloudtech
